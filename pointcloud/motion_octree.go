package pointcloud

import (
	"fmt"

	"github.com/golang/geo/r3"
	"github.com/pkg/errors"

	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/spatialmath"
)

type MotionOctree struct {
	BasicOctree
	pose spatialmath.Pose
}

func NewMotionOctree(pose spatialmath.Pose, sideLength float64) (*MotionOctree, error) {
	if sideLength <= 0 {
		return nil, errors.Errorf("invalid side length (%.2f) for octree", sideLength)
	}

	basicOct, err := NewBasicOctree(pose.Point(), sideLength)
	if err != nil {
		return nil, err
	}

	motionOct := &MotionOctree{
		BasicOctree: *basicOct,
		pose:        pose,
	}

	return motionOct, nil
}

// CollidesWithGeometry will return whether a given geometry is in collision with a given point.
func (mOct *MotionOctree) CollidesWithGeometry(geom spatialmath.Geometry, threshold int, buffer float64) (bool, error) {

	if mOct.MaxVal() < threshold {
		return false, nil
	}
	switch mOct.node.nodeType {
	case internalNode:
		ocbox, err := spatialmath.NewBox(
			spatialmath.NewPoseFromPoint(mOct.center),
			r3.Vector{X: mOct.sideLength + buffer, Y: mOct.sideLength + buffer, Z: mOct.sideLength + buffer},
			"",
		)
		if err != nil {
			return false, err
		}

		// Check whether our geom collides with the area represented by the octree. If false, we can skip
		collide, err := geom.CollidesWith(ocbox)
		if err != nil {
			return false, err
		}
		if !collide {
			return false, nil
		}
		for _, child := range mOct.node.children {
			collide, err = child.CollidesWithGeometry(geom, threshold, buffer)
			if err != nil {
				return false, err
			}
			if collide {
				return true, nil
			}
		}
		return false, nil
	case leafNodeEmpty:
		return false, nil
	case leafNodeFilled:
		ptGeom, err := spatialmath.NewSphere(spatialmath.NewPoseFromPoint(mOct.node.point.P), buffer, "")
		if err != nil {
			return false, err
		}

		ptCollide, err := geom.CollidesWith(ptGeom)
		if err != nil {
			return false, err
		}
		return ptCollide, nil
	}
	return false, errors.New("unknown octree node type")
}

func (octree *MotionOctree) ToProtobuf() *commonpb.Geometry {
	// TODO
	return nil
}

// Label returns the label of this octree.
func (mOct *MotionOctree) Label() string {
	return mOct.node.label
}

// String returns a human readable string that represents this octree.
func (mOct *MotionOctree) String() string {
	return fmt.Sprintf("octree with center at %v and side length of %v", mOct.pose, mOct.sideLength)
}

// ToPoints converts an octree geometry into []r3.Vector.
func (mOct *MotionOctree) ToPoints(resolution float64) []r3.Vector {
	// TODO
	return nil
}
