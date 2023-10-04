package builtin

import (
	"context"
	"math/rand"

	"github.com/golang/geo/r3"
	"github.com/pkg/errors"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils"
)

const (
	// Name of path geometry in which we look for obstacles
	immediateBasePathGeometryName = "immediateBasePath"
	// Distance in front of base to look for obstacle
	pathLengthMM float64 = 100
)

func createImmediatePathGeometry(ctx context.Context, svc *builtIn) (spatialmath.Geometry, error) {
	baseProperties, err := svc.base.Properties(ctx, nil)
	if err != nil {
		return nil, errors.Wrapf(err, "unable to get properties from the base")
	}

	return spatialmath.NewBox(spatialmath.NewPoseFromPoint(
		r3.Vector{X: 0, Y: pathLengthMM / 2, Z: 0}),
		r3.Vector{
			X: baseProperties.WidthMeters,
			Y: pathLengthMM,
			Z: 3, //?????????
		},
		immediateBasePathGeometryName)
}

// startExploreMode begins a background process which implements a random walk algorithm. This random walk algorithm
// will move the base forward until it detects an obstacle. When an obstacle is detected, the base will stop and
// spin a random angle from [-180, 180] and check for an obstacle before moving forward again. Obstacle detection is
// done using GetObjectPointCloud in the associated the vision service a with the user defined camera.
func (svc *builtIn) startExploreMode(ctx context.Context) {
	svc.logger.Debug("startExploreMode called")

	// Create a geometric region representing the immediate path the base will be moving along. Obstacles will be
	// searched for in this region.
	baseImmediatePathGeometry, err := createImmediatePathGeometry(ctx, svc)
	if err != nil {
		svc.logger.Errorf("unable to build geometric region to look for obstacles: %v", err)
	}

	// Begin background process
	svc.activeBackgroundWorkers.Add(1)
	utils.PanicCapturingGo(func() {
		defer svc.activeBackgroundWorkers.Done()

		// Function used to detect if an obstacle is in the immediate path of the base.
		detectObstacle := func(ctx context.Context) (bool, error) {
			objs, err := svc.vision[0].GetObjectPointClouds(ctx, camera, nil)
			if err != nil {
				return false, err
			}

			var isObstacle bool
			for _, obj := range objs {
				isObstacle, err = baseImmediatePathGeometry.CollidesWith(obj.Geometry)
				if err != nil {
					return false, err
				}
			}
			return isObstacle, nil
		}

		for {
			if ctx.Err() != nil {
				// Stop motor
				if err := svc.base.Stop(ctx, nil); err != nil {
					svc.logger.Error("issue stopping base when exiting explore mode")
				}
				return
			}

			// Look for obstacle in immediate path
			isObstacle, err := detectObstacle(ctx)
			if err != nil {
				svc.logger.Error("failed to determine in obstacle is in front of base")
			}

			if isObstacle {
				// If obstacle exists, stop the base and spin a random angle [-180, 180]
				isMoving, err := svc.base.IsMoving(ctx)
				if err != nil {
					svc.logger.Warn("issue checking if base is moving")
				}
				if isMoving {
					if err = svc.base.Stop(ctx, nil); err != nil {
						svc.logger.Error("issue stopping base when obstacle is detected")
					}
				}

				randomAngle := float64(rand.Intn(360) - 180)
				if err := svc.base.Spin(ctx, randomAngle, svc.motionCfg.AngularDegsPerSec, nil); err != nil {
					svc.logger.Error("issue spinning base when obstacle is detected")
				}
			} else {
				// If obstacle does not exists, move forward at given velocity
				isMoving, err := svc.base.IsMoving(ctx)
				if err != nil {
					svc.logger.Warn("issue checking if base is moving")
				}
				if !isMoving {
					if err := svc.base.SetVelocity(ctx, r3.Vector{X: 0, Y: svc.motionCfg.LinearMPerSec * 1000}, r3.Vector{}, nil); err != nil { // METERS PER SEC OR MM PER SEC
						svc.logger.Error("issue setting velocity base when no obstacle is detected")
					}
				}
			}
		}
	})
}
