package utils

import (
	"context"
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

	type testCase struct {
		file                                    string
		allowedErrorTranslation, allowedErrorOV float64
	}

	cases := []testCase{
		{
			file:                    "data/fuzz_0_mm.json",
			allowedErrorTranslation: 1e-3,
			allowedErrorOV:          1e-6,
		},
		{
			file:                    "data/fuzz_1_mm.json",
			allowedErrorTranslation: 0.15,
			allowedErrorOV:          1e-6,
		},
		{
			file:                    "data/fuzz_3_mm.json",
			allowedErrorTranslation: 0.45,
			allowedErrorOV:          1e-6,
		},
		{
			file:                    "data/fuzz_5_mm.json",
			allowedErrorTranslation: 0.5,
			allowedErrorOV:          1e-5,
		},
		{
			file:                    "data/fuzz_10_mm.json",
			allowedErrorTranslation: 0.82,
			allowedErrorOV:          1e-5,
		},
	}

	for _, testCase := range cases {
		t.Run(testCase.file, func(t *testing.T) {
			data, res, err := ReadData(testCase.file)
			test.That(t, err, test.ShouldBeNil)

			// test with zero pose
			sol, err := Minimize(context.Background(), defaultLimits, data, spatialmath.NewZeroPose(), logger)
			test.That(t, err, test.ShouldBeNil)
			test.That(t, spatialmath.R3VectorAlmostEqual(sol[0].Pose().Point(), res.Point(), testCase.allowedErrorTranslation), test.ShouldBeTrue)
			test.That(t, spatialmath.OrientationAlmostEqualEps(sol[0].Pose().Orientation(), res.Orientation(), testCase.allowedErrorOV), test.ShouldBeTrue)
			logger.Infof("with %s file and this seed guess: %v, this was the output of minimize: %v", testCase.file, spatialmath.NewZeroPose(), sol[0].Pose())
			logger.Infof("delta between desired and outcome: %v", spatialmath.PoseDelta(res, sol[0].Pose()))

			// test with result
			sol, err = Minimize(context.Background(), defaultLimits, data, res, logger)
			test.That(t, err, test.ShouldBeNil)
			test.That(t, spatialmath.R3VectorAlmostEqual(sol[0].Pose().Point(), res.Point(), testCase.allowedErrorTranslation), test.ShouldBeTrue)
			test.That(t, spatialmath.OrientationAlmostEqualEps(sol[0].Pose().Orientation(), res.Orientation(), testCase.allowedErrorOV), test.ShouldBeTrue)
			logger.Infof("with %s file and this seed guess: %v, this was the output of minimize: %v", testCase.file, spatialmath.NewZeroPose(), sol[0].Pose())
			logger.Infof("delta between desired and outcome: %v\n", spatialmath.PoseDelta(res, sol[0].Pose()))
		})
	}
}
