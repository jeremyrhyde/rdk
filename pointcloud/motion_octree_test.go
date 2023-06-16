package pointcloud

import (
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/test"

	"go.viam.com/rdk/spatialmath"
)

// Test the functionalities involved with converting a pointcloud into a basic octree.
func TestMotionOctreeCollision(t *testing.T) {
	startPC, err := makeFullPointCloudFromArtifact(
		t,
		"pointcloud/collision_pointcloud_0.pcd",
		BasicType,
	)
	test.That(t, err, test.ShouldBeNil)

	center := getCenterFromPcMetaData(startPC.MetaData())
	maxSideLength := getMaxSideLengthFromPcMetaData(startPC.MetaData())

	basicOct, err := NewMotionOctree(spatialmath.NewPoseFromPoint(center), maxSideLength)
	test.That(t, err, test.ShouldBeNil)

	startPC.Iterate(0, 0, func(p r3.Vector, d Data) bool {
		// Blue channel is used to determine probability in pcds produced by cartographer
		_, _, blueProb := d.RGB255()
		d.SetValue(int(blueProb))
		if err = basicOct.Set(p, d); err != nil {
			return false
		}
		return true
	})

	test.That(t, startPC.Size(), test.ShouldEqual, basicOct.Size())

	t.Run("no collision with box far from octree points", func(t *testing.T) {
		// create a non-colliding obstacle far away from any octree point
		far, err := spatialmath.NewBox(spatialmath.NewZeroPose(), r3.Vector{1, 2, 3}, "far")
		test.That(t, err, test.ShouldBeNil)
		collides, err := basicOct.CollidesWithGeometry(far, 80, 1.0)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, collides, test.ShouldBeFalse)
	})

	t.Run("no collision with box near octree points", func(t *testing.T) {
		// create a non-colliding obstacle near an octree point
		near, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(r3.Vector{-2443, 0, 3855}), r3.Vector{1, 2, 3}, "near")
		test.That(t, err, test.ShouldBeNil)
		collides, err := basicOct.CollidesWithGeometry(near, 80, 1.0)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, collides, test.ShouldBeFalse)
	})

	t.Run("collision with box near octree points when a large buffer is used", func(t *testing.T) {
		// create a non-colliding obstacle near an octree point
		near, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(r3.Vector{-2443, 0, 3855}), r3.Vector{1, 2, 3}, "near")
		test.That(t, err, test.ShouldBeNil)
		collides, err := basicOct.CollidesWithGeometry(near, 80, 10.0)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, collides, test.ShouldBeTrue)
	})

	t.Run("no collision with box overlapping low-probability octree points", func(t *testing.T) {
		// create a colliding obstacle overlapping an octree point that has sub-threshold probability
		lowprob, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(r3.Vector{-2471, 0, 3790}), r3.Vector{3, 2, 3}, "lowprob")
		test.That(t, err, test.ShouldBeNil)
		collides, err := basicOct.CollidesWithGeometry(lowprob, 80, 1.0)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, collides, test.ShouldBeFalse)
	})

	t.Run("collision with box overlapping octree points", func(t *testing.T) {
		// create a colliding obstacle overlapping an octree point
		hit, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(r3.Vector{-2443, 0, 3855}), r3.Vector{12, 2, 30}, "hit")
		test.That(t, err, test.ShouldBeNil)
		collides, err := basicOct.CollidesWithGeometry(hit, 80, 1.0)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, collides, test.ShouldBeTrue)
	})
}
