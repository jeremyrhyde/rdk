// Package fake implements a fake camera which always returns the same image with a user specified resolution.
package fake_lidar

import (
	"context"
	"errors"
	"fmt"
	"image"
	"os"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/gostream"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/rimage/transform"
	"go.viam.com/utils/artifact"
)

var model = resource.DefaultModelFamily.WithModel("fake_lidar")

const (
	mockDataPath        = "slam/mock_lidar"
	numPCDArtifacts int = 15
)

func init() {
	resource.RegisterComponent(
		camera.API,
		model,
		resource.Registration[camera.Camera, *Config]{
			Constructor: func(
				ctx context.Context,
				_ resource.Dependencies,
				cfg resource.Config,
				logger logging.Logger,
			) (camera.Camera, error) {
				return NewCamera(ctx, cfg, logger)
			},
		})
}

// NewCamera returns a new fake camera.
func NewCamera(
	ctx context.Context,
	conf resource.Config,
	logger logging.Logger,
) (camera.Camera, error) {

	cam := &Camera{
		Named:  conf.ResourceName().AsNamed(),
		logger: logger,
	}

	return cam, nil
}

// Config are the attributes of the fake camera config.
type Config struct{}

// Validate checks that the config attributes are valid for a fake camera.
func (conf *Config) Validate(path string) ([]string, error) {
	return nil, nil
}

// Camera is a fake camera that always returns the same image.
type Camera struct {
	resource.Named
	resource.AlwaysRebuild
	i      int
	logger logging.Logger
}

// NextPointCloud returns a pcd artifact created by an rplidar.
func (c *Camera) NextPointCloud(ctx context.Context) (pointcloud.PointCloud, error) {
	if c.i >= numPCDArtifacts {
		return nil, errors.New("end of dataset")
	}

	file, err := os.Open(artifact.MustPath(fmt.Sprintf("%v/%v.pcd", mockDataPath, c.i)))
	if err != nil {
		return nil, err
	}
	pc, err := pointcloud.ReadPCD(file)
	if err != nil {
		return nil, err
	}

	return pc, nil
}

// Properties returns support_pcd as true.
func (c *Camera) Properties(ctx context.Context) (camera.Properties, error) {
	return camera.Properties{SupportsPCD: true}, nil
}

// Close does nothing.
func (c *Camera) Close(ctx context.Context) error {
	return nil
}

// Read is unimplemented for fake lidar.
func (c *Camera) Read(ctx context.Context) (image.Image, func(), error) {
	return nil, nil, errors.New("unimplemented")
}

// Stream is unimplemented for fake lidar.
func (c *Camera) Stream(ctx context.Context, errHandlers ...gostream.ErrorHandler) (gostream.VideoStream, error) {
	return nil, errors.New("unimplemented")
}

// Images is unimplemented for fake lidar.
func (c *Camera) Images(ctx context.Context) ([]camera.NamedImage, resource.ResponseMetadata, error) {
	return nil, resource.ResponseMetadata{}, errors.New("unimplemented")
}

// Projector is unimplemented for fake lidar.
func (c *Camera) Projector(ctx context.Context) (transform.Projector, error) {
	return nil, errors.New("unimplemented")
}
