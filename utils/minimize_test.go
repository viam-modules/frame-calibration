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

	x, err := Minimize(ctx, data, spatialmath.NewZeroPose(), logger)
	test.That(t, err, test.ShouldBeNil)

	xx := x[0].Pose()

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
