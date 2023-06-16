package pointcloud

import (
	"fmt"

	"github.com/golang/geo/r3"
	"github.com/pkg/errors"

	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/spatialmath"
)

type MotionOctree struct {
	basicOct *BasicOctree
	pose     spatialmath.Pose
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
		basicOct: basicOct,
		pose:     pose,
	}

	return motionOct, nil
}

func (mOct *MotionOctree) Size() int {
	return mOct.basicOct.size
}

func (mOct *MotionOctree) MaxVal() int {
	return mOct.basicOct.MaxVal()
}

func (mOct *MotionOctree) Set(p r3.Vector, d Data) error {
	return mOct.basicOct.Set(p, d)
}

func (mOct *MotionOctree) At(x, y, z float64) (Data, bool) {
	return mOct.basicOct.At(x, y, z)
}

func (mOct *MotionOctree) Iterate(numBatches, currentBatch int, fn func(p r3.Vector, d Data) bool) {
	mOct.basicOct.Iterate(numBatches, currentBatch, fn)
}

func (mOct *MotionOctree) MetaData() MetaData {
	return mOct.basicOct.meta
}

// CollidesWithGeometry will return whether a given geometry is in collision with a given point.
func (mOct *MotionOctree) CollidesWithGeometry(geom spatialmath.Geometry, threshold int, buffer float64) (bool, error) {

	octree := mOct.basicOct

	if octree.MaxVal() < threshold {
		return false, nil
	}
	switch octree.node.nodeType {
	case internalNode:
		ocbox, err := spatialmath.NewBox(
			spatialmath.NewPoseFromPoint(octree.center),
			r3.Vector{X: octree.sideLength + buffer, Y: octree.sideLength + buffer, Z: octree.sideLength + buffer},
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
		for _, child := range octree.node.children {
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
		ptGeom, err := spatialmath.NewSphere(spatialmath.NewPoseFromPoint(octree.node.point.P), buffer, "")
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
func (octree *MotionOctree) Label() string {
	return octree.basicOct.node.label
}

// String returns a human readable string that represents this octree.
func (octree *MotionOctree) String() string {
	return fmt.Sprintf("octree with center at %v and side length of %v", octree.pose, octree.basicOct.sideLength)
}

// ToPoints converts an octree geometry into []r3.Vector.
func (octree *MotionOctree) ToPoints(resolution float64) []r3.Vector {
	// TODO
	return nil
}
