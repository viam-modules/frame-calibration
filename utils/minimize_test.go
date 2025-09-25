package utils

import (
	"context"
	"path/filepath"
	"testing"

	"go.viam.com/rdk/logging"
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

func TestMinimizeResilienceToNoise(t *testing.T) {
	logger := logging.NewTestLogger(t)

	files := []string{
		"fuzz_0_mm.json",
		"fuzz_1_mm.json",
		"fuzz_3_mm.json",
		"fuzz_5_mm.json",
		"fuzz_10_mm.json",
	}
	location := "data"

	for _, s := range files {
		data, res, err := ReadData(filepath.Join(location, s))
		test.That(t, err, test.ShouldBeNil)
		logger.Infof("desired output of minimize function: %v", res)

		sol, err := Minimize(context.Background(), defaultLimits, data, spatialmath.NewZeroPose(), logger)
		test.That(t, err, test.ShouldBeNil)
		logger.Infof("with %s file and this seed guess: %v, this was the output of minimize: %v", s, spatialmath.NewZeroPose(), sol[0].Pose())
		logger.Infof("delta between desired and outcome: %v", spatialmath.PoseDelta(res, sol[0].Pose()))

		sol, err = Minimize(context.Background(), defaultLimits, data, res, logger)
		test.That(t, err, test.ShouldBeNil)
		logger.Infof("with %s file and this seed guess: %v, this was the output of minimize: %v", s, res, sol[0].Pose())
		logger.Infof("delta between desired and outcome: %v", spatialmath.PoseDelta(res, sol[0].Pose()))
		logger.Info("----------------------------------------------------------------")
	}
}
