package replay

import (
	"context"
	"fmt"
	"math"
	"net"
	"strconv"
	"testing"
	"time"

	"github.com/edaniels/golog"
	"github.com/golang/geo/r3"
	"github.com/pkg/errors"
	datapb "go.viam.com/api/app/data/v1"
	"go.viam.com/test"
	"go.viam.com/utils/rpc"
	"google.golang.org/protobuf/types/known/structpb"
	"google.golang.org/protobuf/types/known/timestamppb"

	"go.viam.com/rdk/components/movementsensor"
	viamgrpc "go.viam.com/rdk/grpc"
	"go.viam.com/rdk/internal/cloud"
	cloudinject "go.viam.com/rdk/internal/testutils/inject"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/testutils/inject"
)

const (
	testTime   = "2000-01-01T12:00:%02dZ"
	orgID      = "org_id"
	locationID = "location_id"
)

// mockDataServiceServer is a struct that includes unimplemented versions of all the Data Service endpoints. These
// can be overwritten to allow developers to trigger desired behaviors during testing.
type mockDataServiceServer struct {
	datapb.UnimplementedDataServiceServer
}

// TabularDataByFilter is a mocked version of the Data Service function of a similar name. It returns a response with
// data corresponding to the stored data associated with that function and index.
func (mDServer *mockDataServiceServer) TabularDataByFilter(ctx context.Context, req *datapb.TabularDataByFilterRequest,
) (*datapb.TabularDataByFilterResponse, error) {
	filter := req.DataRequest.GetFilter()
	last := req.DataRequest.GetLast()
	limit := req.DataRequest.GetLimit()

	// Construct response
	var dataset []*datapb.TabularData
	var index int
	var err error
	for i := 0; i < int(limit); i++ {
		index, err = getNextDataAfterFilter(filter, last)
		if err != nil {
			if i == 0 {
				return nil, err
			}
			continue
		}

		data := createDataByFunction(filter.Method, index)

		timeReq, timeRec, err := timestampsFromIndex(index)
		if err != nil {
			return nil, err
		}

		last = fmt.Sprint(index)

		tabularData := &datapb.TabularData{
			Data:          data,
			TimeRequested: timeReq,
			TimeReceived:  timeRec,
		}
		dataset = append(dataset, tabularData)
	}

	resp := &datapb.TabularDataByFilterResponse{
		Data: dataset,
		Last: last,
	}

	return resp, nil
}

func timestampsFromIndex(index int) (*timestamppb.Timestamp, *timestamppb.Timestamp, error) {
	timeReq, err := time.Parse(time.RFC3339, fmt.Sprintf(testTime, index))
	if err != nil {
		return nil, nil, errors.Wrap(err, "failed parsing time")
	}
	timeRec := timeReq.Add(time.Second)
	return timestamppb.New(timeReq), timestamppb.New(timeRec), nil
}

// getNextDataAfterFilter returns the data point to be return based on the provided filter and last returned data index.
func getNextDataAfterFilter(filter *datapb.Filter, last string) (int, error) {
	// Basic component part (source) filter
	if filter.ComponentName != "" && filter.ComponentName != "source" {
		return 0, ErrEndOfDataset
	}

	// Basic robot_id filter
	if filter.RobotId != "" && filter.RobotId != "robot_id" {
		return 0, ErrEndOfDataset
	}

	// Basic location_id filter
	if filter.LocationIds[0] != "" && filter.LocationIds[0] != "location_id" {
		return 0, ErrEndOfDataset
	}

	// Basic organization_id filter
	if filter.OrganizationIds[0] != "" && filter.OrganizationIds[0] != "organization_id" {
		return 0, ErrEndOfDataset
	}

	// Apply the time-based filter based on the seconds value in the start and end fields. Because our mock data
	// does not have timestamps associated with them but are ordered we can approximate the filtering
	// by sorting for the data in the list whose index is after the start second count and before the end second count.
	// For example, if there are 15 entries the start time is 2000-01-01T12:00:10Z and the end time is 2000-01-01T12:00:14Z,
	// we will return data from indices 10 to 14.
	start := 0
	end := defaultMaxDataLength[filter.Method]
	if filter.Interval.Start != nil {
		start = filter.Interval.Start.AsTime().Second()
	}
	if filter.Interval.End != nil {
		end = int(math.Min(float64(filter.Interval.End.AsTime().Second()), float64(end)))
	}
	if last == "" {
		return getData(start, end)
	}
	lastFileNum, err := strconv.Atoi(last)
	if err != nil {
		return 0, err
	}

	return getData(lastFileNum+1, end)
}

// getData will return the next data to be returned after checking it satisfies the end condition.
func getData(i, end int) (int, error) {
	if i < end {
		return i, nil
	}
	return 0, ErrEndOfDataset
}

// createMockCloudDependencies creates a mockDataServiceServer and rpc client connection to it which is then
// stored in a mockCloudConnectionService.
func createMockCloudDependencies(ctx context.Context, t *testing.T, logger golog.Logger, b bool) (resource.Dependencies, func() error) {
	listener, err := net.Listen("tcp", "localhost:0")
	test.That(t, err, test.ShouldBeNil)
	rpcServer, err := rpc.NewServer(logger, rpc.WithUnauthenticated())
	test.That(t, err, test.ShouldBeNil)

	test.That(t, rpcServer.RegisterServiceServer(
		ctx,
		&datapb.DataService_ServiceDesc,
		&mockDataServiceServer{},
		datapb.RegisterDataServiceHandlerFromEndpoint,
	), test.ShouldBeNil)

	go rpcServer.Serve(listener)

	conn, err := viamgrpc.Dial(ctx, listener.Addr().String(), logger)
	test.That(t, err, test.ShouldBeNil)

	mockCloudConnectionService := &cloudinject.CloudConnectionService{
		Named: cloud.InternalServiceName.AsNamed(),
		Conn:  conn,
	}
	if !b {
		mockCloudConnectionService.AcquireConnectionErr = errors.New("cloud connection error")
	}

	r := &inject.Robot{}
	rs := map[resource.Name]resource.Resource{}
	rs[cloud.InternalServiceName] = mockCloudConnectionService
	r.MockResourcesFromMap(rs)

	return resourcesFromDeps(t, r, []string{cloud.InternalServiceName.String()}), rpcServer.Stop
}

// createNewReplayMovementSensor will create a new replay movement sensor based on the provided config with either
// a valid or invalid data client.
func createNewReplayMovementSensor(ctx context.Context, t *testing.T, replayMovementSensorCfg *Config, validDeps bool,
) (movementsensor.MovementSensor, resource.Dependencies, func() error, error) {
	logger := golog.NewTestLogger(t)

	resources, closeRPCFunc := createMockCloudDependencies(ctx, t, logger, validDeps)

	cfg := resource.Config{ConvertedAttributes: replayMovementSensorCfg}
	replay, err := newReplayMovementSensor(ctx, resources, cfg, logger)

	return replay, resources, closeRPCFunc, err
}

// resourcesFromDeps returns a list of dependencies from the provided robot.
func resourcesFromDeps(t *testing.T, r robot.Robot, deps []string) resource.Dependencies {
	t.Helper()
	resources := resource.Dependencies{}
	for _, dep := range deps {
		resName, err := resource.NewFromString(dep)
		test.That(t, err, test.ShouldBeNil)
		res, err := r.ResourceByName(resName)
		if err == nil {
			// some resources are weakly linked
			resources[resName] = res
		}
	}
	return resources
}

// createDataByFunction will create the mocked struct returned by calls in tabular data.
func createDataByFunction(method string, index int) *structpb.Struct {
	var data structpb.Struct
	switch method {
	case "Position":
		data.Fields = map[string]*structpb.Value{
			"Latitude":  structpb.NewNumberValue(positionPointData[index].Lat()),
			"Longitude": structpb.NewNumberValue(positionPointData[index].Lng()),
			"Altitude":  structpb.NewNumberValue(positionAltitudeData[index]),
		}
	case "LinearAcceleration":
		data.Fields = map[string]*structpb.Value{
			"X": structpb.NewNumberValue(linearAccelerationData[index].X),
			"Y": structpb.NewNumberValue(linearAccelerationData[index].Y),
			"Z": structpb.NewNumberValue(linearAccelerationData[index].Z),
		}
	case "AngularVelocity":
		data.Fields = map[string]*structpb.Value{
			"X": structpb.NewNumberValue(angularVelocityData[index].X),
			"Y": structpb.NewNumberValue(angularVelocityData[index].Y),
			"Z": structpb.NewNumberValue(angularVelocityData[index].Z),
		}
	case "LinearVelocity":
		data.Fields = map[string]*structpb.Value{
			"X": structpb.NewNumberValue(linearVelocityData[index].X),
			"Y": structpb.NewNumberValue(linearVelocityData[index].Y),
			"Z": structpb.NewNumberValue(linearVelocityData[index].Z),
		}
	case "Orientation":
		data.Fields = map[string]*structpb.Value{
			"OX":    structpb.NewNumberValue(orientationData[index].OX),
			"OY":    structpb.NewNumberValue(orientationData[index].OY),
			"OZ":    structpb.NewNumberValue(orientationData[index].OZ),
			"Theta": structpb.NewNumberValue(orientationData[index].Theta),
		}
	case "CompassHeading":
		data.Fields = map[string]*structpb.Value{
			"Compass": structpb.NewNumberValue(compassHeadingData[index]),
		}
	default:
		data.Fields = map[string]*structpb.Value{
			"X": structpb.NewNumberValue(linearAccelerationData[index].X),
			"Y": structpb.NewNumberValue(linearAccelerationData[index].Y),
			"Z": structpb.NewNumberValue(linearAccelerationData[index].Z),
		}
	}
	return &data
}

// testMovementSensorFunction tests the specified replay movement sensor function, both success and failure cases.
func testReplayMovementSensorFunction(ctx context.Context, t *testing.T, replay movementsensor.MovementSensor, method string,
	i int, success bool,
) {
	var extra map[string]interface{}
	switch method {
	case "Position":
		point, altitude, err := replay.Position(ctx, extra)
		if success {
			test.That(t, err, test.ShouldBeNil)
			test.That(t, point, test.ShouldResemble, positionPointData[i])
			test.That(t, altitude, test.ShouldResemble, positionAltitudeData[i])
		} else {
			test.That(t, err, test.ShouldNotBeNil)
			test.That(t, err.Error(), test.ShouldContainSubstring, ErrEndOfDataset.Error())
			test.That(t, point, test.ShouldBeNil)
			test.That(t, altitude, test.ShouldEqual, 0)
		}
	case "LinearVelocity":
		data, err := replay.LinearVelocity(ctx, extra)
		if success {
			test.That(t, err, test.ShouldBeNil)
			test.That(t, data, test.ShouldResemble, linearVelocityData[i])
		} else {
			test.That(t, err, test.ShouldNotBeNil)
			test.That(t, err.Error(), test.ShouldContainSubstring, ErrEndOfDataset.Error())
			test.That(t, data, test.ShouldResemble, r3.Vector{})
		}
	case "LinearAcceleration":
		data, err := replay.LinearAcceleration(ctx, extra)
		if success {
			test.That(t, err, test.ShouldBeNil)
			test.That(t, data, test.ShouldResemble, linearAccelerationData[i])
		} else {
			test.That(t, err, test.ShouldNotBeNil)
			test.That(t, err.Error(), test.ShouldContainSubstring, ErrEndOfDataset.Error())
			test.That(t, data, test.ShouldResemble, r3.Vector{})
		}
	case "AngularVelocity":
		data, err := replay.AngularVelocity(ctx, extra)
		if success {
			test.That(t, err, test.ShouldBeNil)
			test.That(t, data, test.ShouldResemble, angularVelocityData[i])
		} else {
			test.That(t, err, test.ShouldNotBeNil)
			test.That(t, err.Error(), test.ShouldContainSubstring, ErrEndOfDataset.Error())
			test.That(t, data, test.ShouldResemble, spatialmath.AngularVelocity{})
		}
	case "CompassHeading":
		data, err := replay.CompassHeading(ctx, extra)
		if success {
			test.That(t, err, test.ShouldBeNil)
			test.That(t, data, test.ShouldEqual, compassHeadingData[i])
		} else {
			test.That(t, err, test.ShouldNotBeNil)
			test.That(t, err.Error(), test.ShouldContainSubstring, ErrEndOfDataset.Error())
			test.That(t, data, test.ShouldEqual, 0)
		}
	case "Orientation":
		data, err := replay.Orientation(ctx, extra)
		if success {
			test.That(t, err, test.ShouldBeNil)
			test.That(t, err, test.ShouldBeNil)
			test.That(t, data, test.ShouldResemble, orientationData[i])
		} else {
			test.That(t, err, test.ShouldNotBeNil)
			test.That(t, err.Error(), test.ShouldContainSubstring, ErrEndOfDataset.Error())
			test.That(t, data, test.ShouldBeNil)
		}
	default:
		data, err := replay.LinearAcceleration(ctx, extra)
		if success {
			test.That(t, err, test.ShouldBeNil)
			test.That(t, err, test.ShouldBeNil)
			test.That(t, data, test.ShouldResemble, linearAccelerationData[i])
		} else {
			test.That(t, err, test.ShouldNotBeNil)
			test.That(t, err.Error(), test.ShouldContainSubstring, ErrEndOfDataset.Error())
			test.That(t, data, test.ShouldResemble, r3.Vector{})
		}
	}
}
