// Package replayimu implements a replay imu that can return point cloud data.
package replayimu

import (
	"bytes"
	"compress/gzip"
	"context"
	"sync"
	"time"

	"github.com/edaniels/golog"
	"github.com/golang/geo/r3"
	geo "github.com/kellydunn/golang-geo"
	"github.com/pkg/errors"
	datapb "go.viam.com/api/app/data/v1"
	goutils "go.viam.com/utils"
	"go.viam.com/utils/rpc"
	"google.golang.org/grpc"
	"google.golang.org/grpc/metadata"
	"google.golang.org/protobuf/types/known/timestamppb"

	"go.viam.com/rdk/components/movementsensor"
	"go.viam.com/rdk/internal/cloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/utils/contextutils"
)

const (
	timeFormat            = time.RFC3339
	grpcConnectionTimeout = 10 * time.Second
	downloadTimeout       = 30 * time.Second
	maxCacheSize          = 100
)

var (
	// model is the model of a replay imu.
	model           = resource.DefaultModelFamily.WithModel("replay_imu")
	errEndOfDataset = errors.New("reached end of dataset")
)

func init() {
	resource.RegisterComponent(movementsensor.API, model, resource.Registration[movementsensor.MovementSensor, *Config]{
		Constructor: newIMU,
	})
}

// Config describes how to configure the replay imu component.
type Config struct {
	Source    string       `json:"source,omitempty"`
	RobotID   string       `json:"robot_id,omitempty"`
	Interval  TimeInterval `json:"time_interval,omitempty"`
	BatchSize *uint64      `json:"batch_size,omitempty"`
}

// TimeInterval holds the start and end time used to filter data.
type TimeInterval struct {
	Start string `json:"start,omitempty"`
	End   string `json:"end,omitempty"`
}

// cacheEntry stores data that was downloaded from a previous operation but has not yet been passed
// to the caller.
type cacheEntry struct {
	id                 *datapb.BinaryID
	linearAcceleration r3.Vector
	angularVelocity    spatialmath.AngularVelocity
	timeRequested      *timestamppb.Timestamp
	timeReceived       *timestamppb.Timestamp
	err                error
}

// Validate checks that the config attributes are valid for a replay camera.
func (cfg *Config) Validate(path string) ([]string, error) {
	if cfg.Source == "" {
		return nil, goutils.NewConfigValidationFieldRequiredError(path, "source")
	}

	var err error
	var startTime time.Time
	if cfg.Interval.Start != "" {
		startTime, err = time.Parse(timeFormat, cfg.Interval.Start)
		if err != nil {
			return nil, errors.New("invalid time format for start time (UTC), use RFC3339")
		}
		if startTime.After(time.Now()) {
			return nil, errors.New("invalid config, start time (UTC) must be in the past")
		}
	}

	var endTime time.Time
	if cfg.Interval.End != "" {
		endTime, err = time.Parse(timeFormat, cfg.Interval.End)
		if err != nil {
			return nil, errors.New("invalid time format for end time (UTC), use RFC3339")
		}
		if endTime.After(time.Now()) {
			return nil, errors.New("invalid config, end time (UTC) must be in the past")
		}
	}

	if cfg.Interval.Start != "" && cfg.Interval.End != "" && startTime.After(endTime) {
		return nil, errors.New("invalid config, end time (UTC) must be after start time (UTC)")
	}

	if cfg.BatchSize != nil && (*cfg.BatchSize > uint64(maxCacheSize) || *cfg.BatchSize == 0) {
		return nil, errors.Errorf("batch_size must be between 1 and %d", maxCacheSize)
	}

	return []string{cloud.InternalServiceName.String()}, nil
}

// pcdCamera is a camera model that plays back pre-captured point cloud data.
type imu struct {
	resource.Named
	logger golog.Logger

	cloudConnSvc cloud.ConnectionService
	cloudConn    rpc.ClientConn
	dataClient   datapb.DataServiceClient

	lastData string
	limit    uint64
	filter   *datapb.Filter

	cache []*cacheEntry

	mu     sync.RWMutex
	closed bool
}

// newPCDCamera creates a new replay camera based on the inputted config and dependencies.
func newIMU(ctx context.Context, deps resource.Dependencies, conf resource.Config, logger golog.Logger) (movementsensor.MovementSensor, error) {
	replay := &imu{
		Named:  conf.ResourceName().AsNamed(),
		logger: logger,
	}

	if err := replay.Reconfigure(ctx, deps, conf); err != nil {
		return nil, err
	}

	return replay, nil
}

func (replay *imu) Readings(ctx context.Context, extra map[string]interface{}) (map[string]interface{}, error) {

	if len(replay.cache) == 0 {
		if err := replay.GetMoreData(ctx); err != nil {
			return nil, err
		}
	}

	readings, err := movementsensor.Readings(ctx, replay, extra)
	if err != nil {
		return nil, err
	}
	replay.cache = replay.cache[1:]

	return readings, err
}

func (replay *imu) AngularVelocity(ctx context.Context, extra map[string]interface{}) (spatialmath.AngularVelocity, error) {
	if len(replay.cache) == 0 {
		return spatialmath.AngularVelocity{}, errors.New("no data in cache")
	}
	return replay.cache[0].angularVelocity, nil
}

func (replay *imu) LinearAcceleration(ctx context.Context, extra map[string]interface{}) (r3.Vector, error) {
	if len(replay.cache) == 0 {
		return r3.Vector{}, errors.New("no data in cache")
	}
	return replay.cache[0].linearAcceleration, nil
}

func (replay *imu) Properties(ctx context.Context, extra map[string]interface{}) (*movementsensor.Properties, error) {
	replay.mu.Lock()
	defer replay.mu.Unlock()

	return &movementsensor.Properties{
		AngularVelocitySupported:    true,
		LinearAccelerationSupported: true,
	}, nil
}

// NextPointCloud returns the next point cloud retrieved from cloud storage based on the applied filter.
func (replay *imu) GetMoreData(ctx context.Context) error {
	// Retrieve data from the cloud. If the batch size is > 1, only metadata is returned here, otherwise
	// IncludeBinary can be set to true and the data can be downloaded directly via BinaryDataByFilter
	resp, err := replay.dataClient.BinaryDataByFilter(ctx, &datapb.BinaryDataByFilterRequest{
		DataRequest: &datapb.DataRequest{
			Filter:    replay.filter,
			Limit:     replay.limit,
			Last:      replay.lastData,
			SortOrder: datapb.Order_ORDER_ASCENDING,
		},
		CountOnly:     false,
		IncludeBinary: replay.limit == 1,
	})
	if err != nil {
		return err
	}

	if len(resp.GetData()) == 0 {
		return errEndOfDataset
	}
	replay.lastData = resp.GetLast()

	// If using a batch size of 1, we already received the data itself, so decode and return the
	// binary data directly
	if replay.limit == 1 {
		acc, vel, err := decodeResponseData(resp.GetData(), replay.logger)
		if err != nil {
			return err
		}
		if err := addGRPCMetadata(ctx,
			resp.GetData()[0].GetMetadata().GetTimeRequested(),
			resp.GetData()[0].GetMetadata().GetTimeReceived()); err != nil {
			return err
		}

		// TODO: More work here
		entry := &cacheEntry{
			linearAcceleration: acc,
			angularVelocity:    vel,
		}
		replay.cache = append(replay.cache, entry)
		return nil
	}

	// Otherwise if using a batch size > 1, use the metadata from BinaryDataByFilter to download
	// data in parallel and cache the results
	replay.cache = make([]*cacheEntry, len(resp.Data))
	for i, dataResponse := range resp.Data {
		md := dataResponse.GetMetadata()
		replay.cache[i] = &cacheEntry{id: &datapb.BinaryID{
			FileId:         md.GetId(),
			OrganizationId: md.GetCaptureMetadata().GetOrganizationId(),
			LocationId:     md.GetCaptureMetadata().GetLocationId(),
		}}
	}

	ctxTimeout, cancelTimeout := context.WithTimeout(ctx, downloadTimeout)
	defer cancelTimeout()
	replay.downloadBatch(ctxTimeout)
	if ctxTimeout.Err() != nil {
		return errors.Wrap(ctxTimeout.Err(), "failed to download batch")
	}

	return nil
}

// downloadBatch iterates through the current cache, performing the download of the respective data in
// parallel and adds all of them to the cache before returning.
func (replay *imu) downloadBatch(ctx context.Context) {
	// Parallelize download of data based on ids in cache
	var wg sync.WaitGroup
	wg.Add(len(replay.cache))
	for _, dataToCache := range replay.cache {
		data := dataToCache

		goutils.PanicCapturingGo(func() {
			defer wg.Done()

			var resp *datapb.BinaryDataByIDsResponse
			resp, data.err = replay.dataClient.BinaryDataByIDs(ctx, &datapb.BinaryDataByIDsRequest{
				BinaryIds:     []*datapb.BinaryID{data.id},
				IncludeBinary: true,
			})
			if data.err != nil {
				return
			}

			// Decode response data
			data.linearAcceleration, data.angularVelocity, data.err = decodeResponseData(resp.GetData(), replay.logger)
			if data.err == nil {
				data.timeRequested = resp.GetData()[0].GetMetadata().GetTimeRequested()
				data.timeReceived = resp.GetData()[0].GetMetadata().GetTimeReceived()
			}
		})
	}
	wg.Wait()
}

// addGRPCMetadata adds timestamps from the data response to the gRPC response header if one is
// found in the context.
func addGRPCMetadata(ctx context.Context, timeRequested, timeReceived *timestamppb.Timestamp) error {
	if stream := grpc.ServerTransportStreamFromContext(ctx); stream != nil {
		var grpcMetadata metadata.MD = make(map[string][]string)
		if timeRequested != nil {
			grpcMetadata.Set(contextutils.TimeRequestedMetadataKey, timeRequested.AsTime().Format(time.RFC3339Nano))
		}
		if timeReceived != nil {
			grpcMetadata.Set(contextutils.TimeReceivedMetadataKey, timeReceived.AsTime().Format(time.RFC3339Nano))
		}
		if err := grpc.SetHeader(ctx, grpcMetadata); err != nil {
			return err
		}
	}
	return nil
}

// Close stops replay camera, closes the channels and its connections to the cloud.
func (replay *imu) Close(ctx context.Context) error {
	replay.mu.Lock()
	defer replay.mu.Unlock()

	replay.closed = true
	// Close cloud connection
	replay.closeCloudConnection(ctx)
	return nil
}

// Reconfigure finishes the bring up of the replay imu by evaluating given arguments and setting up the required cloud
// connection.
func (replay *imu) Reconfigure(ctx context.Context, deps resource.Dependencies, conf resource.Config) error {
	replay.mu.Lock()
	defer replay.mu.Unlock()
	if replay.closed {
		return errors.New("session closed")
	}

	replayCamConfig, err := resource.NativeConfig[*Config](conf)
	if err != nil {
		return err
	}

	cloudConnSvc, err := resource.FromDependencies[cloud.ConnectionService](deps, cloud.InternalServiceName)
	if err != nil {
		return err
	}

	// Update cloud connection if needed
	if replay.cloudConnSvc != cloudConnSvc {
		replay.closeCloudConnection(ctx)
		replay.cloudConnSvc = cloudConnSvc

		if err := replay.initCloudConnection(ctx); err != nil {
			replay.closeCloudConnection(ctx)
			return errors.Wrap(err, "failure to connect to the cloud")
		}
	}

	if replayCamConfig.BatchSize == nil {
		replay.limit = 1
	} else {
		replay.limit = *replayCamConfig.BatchSize
	}
	replay.cache = nil

	replay.filter = &datapb.Filter{
		ComponentName: replayCamConfig.Source,
		RobotId:       replayCamConfig.RobotID,
		Interval:      &datapb.CaptureInterval{},
	}
	replay.lastData = ""

	if replayCamConfig.Interval.Start != "" {
		startTime, err := time.Parse(timeFormat, replayCamConfig.Interval.Start)
		if err != nil {
			replay.closeCloudConnection(ctx)
			return errors.New("invalid time format for start time, missed during config validation")
		}
		replay.filter.Interval.Start = timestamppb.New(startTime)
	}

	if replayCamConfig.Interval.End != "" {
		endTime, err := time.Parse(timeFormat, replayCamConfig.Interval.End)
		if err != nil {
			replay.closeCloudConnection(ctx)
			return errors.New("invalid time format for end time, missed during config validation")
		}
		replay.filter.Interval.End = timestamppb.New(endTime)
	}

	return nil
}

// closeCloudConnection closes all parts of the cloud connection used by the replay camera.
func (replay *imu) closeCloudConnection(ctx context.Context) {
	if replay.cloudConn != nil {
		goutils.UncheckedError(replay.cloudConn.Close())
	}

	if replay.cloudConnSvc != nil {
		goutils.UncheckedError(replay.cloudConnSvc.Close(ctx))
	}
}

// initCloudConnection creates a rpc client connection and data service.
func (replay *imu) initCloudConnection(ctx context.Context) error {
	ctx, cancel := context.WithTimeout(ctx, grpcConnectionTimeout)
	defer cancel()

	_, conn, err := replay.cloudConnSvc.AcquireConnection(ctx)
	if err != nil {
		return err
	}
	dataServiceClient := datapb.NewDataServiceClient(conn)

	replay.cloudConn = conn
	replay.dataClient = dataServiceClient
	return nil
}

// decodeResponseData decompresses the gzipped byte array.
func decodeResponseData(respData []*datapb.BinaryData, logger golog.Logger) (r3.Vector, spatialmath.AngularVelocity, error) {
	if len(respData) == 0 {
		return r3.Vector{}, spatialmath.AngularVelocity{}, errors.New("no response data; this should never happen")
	}

	r, err := gzip.NewReader(bytes.NewBuffer(respData[0].GetBinary()))
	if err != nil {
		return r3.Vector{}, spatialmath.AngularVelocity{}, err
	}

	defer func() {
		if err = r.Close(); err != nil {
			logger.Warnw("Failed to close gzip reader", "warn", err)
		}
	}()

	// pc, err := pointcloud.ReadPCD(r) // TODO IMPLEMENT
	// if err != nil {
	// 	return r3.Vector{}, spatialmath.AngularVelocity{}, err
	// }

	return r3.Vector{}, spatialmath.AngularVelocity{}, nil
}

func (replay *imu) Position(ctx context.Context, extra map[string]interface{}) (*geo.Point, float64, error) {
	return nil, 0, movementsensor.ErrMethodUnimplementedPosition
}
func (replay *imu) LinearVelocity(ctx context.Context, extra map[string]interface{}) (r3.Vector, error) {
	return r3.Vector{}, movementsensor.ErrMethodUnimplementedLinearVelocity
}
func (replay *imu) CompassHeading(ctx context.Context, extra map[string]interface{}) (float64, error) {
	return 0, movementsensor.ErrMethodUnimplementedCompassHeading
}
func (replay *imu) Orientation(ctx context.Context, extra map[string]interface{}) (spatialmath.Orientation, error) {
	return nil, movementsensor.ErrMethodUnimplementedOrientation
}

func (replay *imu) Accuracy(ctx context.Context, extra map[string]interface{}) (map[string]float32, error) {
	return nil, movementsensor.ErrMethodUnimplementedAccuracy
}
