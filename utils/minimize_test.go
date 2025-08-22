package utils

import (
	"context"
	"maps"
	"math"
	"testing"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

func TestData1(t *testing.T) {
	ctx := context.Background()
	logger := logging.NewTestLogger(t)

	data, res, err := ReadData("data/test1.json")
	test.That(t, err, test.ShouldBeNil)

	x, err := Minimize(ctx, defaultLimits, data, spatialmath.NewZeroPose(), logger)
	test.That(t, err, test.ShouldBeNil)

	xx := x[0].Pose()

	logger.Infof("got: %v %v", xx.Point(), xx.Orientation().OrientationVectorDegrees())
	logger.Infof("res: %v %v", res.Point(), res.Orientation().OrientationVectorDegrees())

	test.That(t, xx.Point().X, test.ShouldAlmostEqual, res.Point().X, .01)
	test.That(t, xx.Point().Y, test.ShouldAlmostEqual, res.Point().Y, .01)
	test.That(t, xx.Point().Z, test.ShouldAlmostEqual, res.Point().Z, .01)

	aa := xx.Orientation().OrientationVectorDegrees()
	bb := res.Orientation().OrientationVectorDegrees()
	test.That(t, aa.OX, test.ShouldAlmostEqual, bb.OX, .01)
	test.That(t, aa.OY, test.ShouldAlmostEqual, bb.OY, .01)
	test.That(t, aa.OZ, test.ShouldAlmostEqual, bb.OZ, .01)
	test.That(t, aa.Theta, test.ShouldAlmostEqual, bb.Theta, .01)
}

func TestData1LessTags(t *testing.T) {
	// this shows that with only 1 tag, you still get reasonable results in some cases

	ctx := context.Background()
	logger := logging.NewTestLogger(t)

	data, res, err := ReadData("data/test1.json")
	test.That(t, err, test.ShouldBeNil)

	for idx, d := range data {
		data[idx].Tags = map[string]SimplePose{
			"11": d.Tags["11"],
		}
	}

	x, err := Minimize(ctx, defaultLimits, data, spatialmath.NewZeroPose(), logger)
	test.That(t, err, test.ShouldBeNil)

	xx := x[0].Pose()

	test.That(t, xx.Point().X, test.ShouldAlmostEqual, res.Point().X, 5)
	test.That(t, xx.Point().Y, test.ShouldAlmostEqual, res.Point().Y, 5)
	test.That(t, xx.Point().Z, test.ShouldAlmostEqual, res.Point().Z, 5)

	aa := xx.Orientation().OrientationVectorDegrees()
	bb := res.Orientation().OrientationVectorDegrees()
	test.That(t, aa.OX, test.ShouldAlmostEqual, bb.OX, .01)
	test.That(t, aa.OY, test.ShouldAlmostEqual, bb.OY, .01)
	test.That(t, aa.OZ, test.ShouldAlmostEqual, bb.OZ, .01)
	test.That(t, aa.Theta, test.ShouldAlmostEqual, bb.Theta, 1)
}

func TestVikingA(t *testing.T) {
	ctx := context.Background()
	logger := logging.NewTestLogger(t)

	data, res, err := ReadData("data/viking20250822a.json")
	test.That(t, err, test.ShouldBeNil)

	allTags := map[string]bool{}

	for idx, d := range data {
		logger.Infof("data %d", idx)
		for k, v := range d.Tags {
			logger.Infof("\t %v %v", k, v)
			allTags[k] = true
		}
	}

	tags := maps.Keys(allTags)

	limits := []referenceframe.Limit{
		{Min: 60, Max: 90},
		{Min: -30, Max: 0},
		{Min: 45, Max: 65},
		{Min: -.1, Max: .1},
		{Min: -.1, Max: .1},
		{Min: 1.5, Max: 1.65},
	}

	x, err := Minimize(ctx, limits, data, spatialmath.NewZeroPose(), logger)
	test.That(t, err, test.ShouldBeNil)

	xx := x[0].Pose()

	logger.Infof("got: %v %v", xx.Point(), xx.Orientation().OrientationVectorDegrees())
	logger.Infof("res: %v %v", res.Point(), res.Orientation().OrientationVectorDegrees())

	data2 := []ArmAndPoses{}
	for _, d := range data {
		data2 = append(data2, ArmAndPoses{Joints: d.Joints, Pose: d.Pose, Tags: map[string]SimplePose{}})
	}

	for tag := range tags {
		logger.Infof("tag %v", tag)

		// first compute the average post

		totals := make([]float64, 7)
		num := 0.0
		for idx, d := range data {
			p, ok := d.Tags[tag]
			if !ok {
				continue
			}
			num++
			tag0world := spatialmath.Compose(spatialmath.Compose(&d.Pose, xx), &p)
			logger.Infof("\tdata %d %v", idx, tag0world)
			totals[0] += tag0world.Point().X
			totals[1] += tag0world.Point().Y
			totals[2] += tag0world.Point().Z
			totals[3] += tag0world.Orientation().OrientationVectorDegrees().OX
			totals[4] += tag0world.Orientation().OrientationVectorDegrees().OY
			totals[5] += tag0world.Orientation().OrientationVectorDegrees().OZ
			totals[6] += tag0world.Orientation().OrientationVectorDegrees().Theta
		}

		for idx, v := range totals {
			totals[idx] = v / num
		}

		avgPose := spatialmath.NewPose(
			r3.Vector{totals[0], totals[1], totals[2]},
			&spatialmath.OrientationVectorDegrees{OX: totals[3], OY: totals[4], OZ: totals[5], Theta: totals[6]},
		)
		logger.Infof("avgPose %v", avgPose)

		// now we figure out an ok variance

		deltas := make([]float64, 2)
		for _, d := range data {
			p, ok := d.Tags[tag]
			if !ok {
				continue
			}
			tag0world := spatialmath.Compose(spatialmath.Compose(&d.Pose, xx), &p)
			deltas[0] += math.Pow(spatialmath.PoseDelta(avgPose, tag0world).Point().Norm(), 2)
			deltas[1] += math.Pow(avgPose.Orientation().OrientationVectorDegrees().Theta-tag0world.Orientation().OrientationVectorDegrees().Theta, 2)
		}

		for idx, v := range deltas {
			deltas[idx] = math.Pow(v/num, .5)
		}

		// find outliers

		for idx, d := range data {
			p, ok := d.Tags[tag]
			if !ok {
				continue
			}
			tag0world := spatialmath.Compose(spatialmath.Compose(&d.Pose, xx), &p)

			good := true
			d := math.Abs(spatialmath.PoseDelta(avgPose, tag0world).Point().Norm())
			if d > deltas[0] {
				logger.Infof("d: %v deltas[0]: %v", d, deltas[0])
				good = false
			}
			d2 := math.Abs(avgPose.Orientation().OrientationVectorDegrees().Theta - tag0world.Orientation().OrientationVectorDegrees().Theta)
			if d2 > deltas[1] {
				logger.Infof("d2: %v deltas[1]: %v", d2, deltas[1])
				good = false
			}

			logger.Infof("\tdata %d %v %v", idx, tag0world, good)

			if good {
				data2[idx].Tags[tag] = p
			}
		}

	}

	y, err := Minimize(ctx, limits, data2, spatialmath.NewZeroPose(), logger)
	test.That(t, err, test.ShouldBeNil)

	yy := y[0].Pose()

	logger.Infof("xx: %v", xx)
	logger.Infof("yy: %v", yy)

	t.Fail()
}
