package builtin

import (
	"context"
	"fmt"
	"math"
	"math/rand"

	"github.com/golang/geo/r3"
	frame "go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils"
)

const defaultDistanceMM = 100 * 1000

//nolint:unparam
func (svc *builtIn) startExploreMode(ctx context.Context) {
	svc.logger.Debug("startExploreMode called")

	svc.activeBackgroundWorkers.Add(1)
	utils.PanicCapturingGo(func() {
		defer svc.activeBackgroundWorkers.Done()

		// Send montion configure parameters through extra until motionCfg can be added to Move
		extra := make(map[string]interface{})
		extra["angular_degs_per_sec"] = svc.motionCfg.AngularDegsPerSec
		extra["linear_m_per_sec"] = svc.motionCfg.LinearMPerSec
		extra["obstacle_polling_frequency_hz"] = svc.motionCfg.ObstaclePollingFreqHz
		extra["obstacle_detectors"] = svc.motionCfg.ObstacleDetectors

		for {
			if ctx.Err() != nil {
				return
			}

			randAngle := (rand.Float64() - 0.5) * math.Pi

			newPose := frame.NewPoseInFrame(svc.base.Name().Name, spatialmath.NewPose(
				r3.Vector{
					X: defaultDistanceMM * math.Sin(randAngle),
					Y: defaultDistanceMM * math.Sin(randAngle),
					Z: 0,
				}, spatialmath.NewOrientationVector()))
			fmt.Println(newPose.Pose().Point())
			_, err := svc.exploreMotionService.Move(ctx, svc.base.Name(), newPose, nil, nil, extra) // worldState, constraints, extra)
			if err != nil {
				svc.logger.Debug("error occurred when moving")
			}
		}
	})
}
