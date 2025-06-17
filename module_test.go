package framecalibration

import (
	"context"
	"fmt"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/test"

	"go.viam.com/rdk/components/arm"
	armFake "go.viam.com/rdk/components/arm/fake"
	ur "go.viam.com/rdk/components/arm/universalrobots"
	"go.viam.com/rdk/components/posetracker"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/testutils/inject"
	injectmotion "go.viam.com/rdk/testutils/inject/motion"
)

const (
	ptName        = "test-pt"
	armName       = "test-arm"
	armCamCalName = "test-cal"
	motionName    = "builtin"
)

func TestNewArmCamera(t *testing.T) {
	ctx := context.Background()
	logger := logging.NewTestLogger(t)

	injectArmName := arm.Named(armName)

	armCfg := resource.Config{
		Name:  armName,
		API:   arm.API,
		Model: resource.DefaultModelFamily.WithModel("ur5e"),
		ConvertedAttributes: &armFake.Config{
			ArmModel: "ur5e",
		},
		Frame: &referenceframe.LinkConfig{
			Parent: "world",
		},
	}

	fakeArm, err := armFake.NewArm(ctx, nil, armCfg, logger)
	test.That(t, err, test.ShouldBeNil)

	injectArm := &inject.Arm{
		Arm: fakeArm,
	}

	injectArm.KinematicsFunc = func(ctx context.Context) (referenceframe.Model, error) {
		model, _ := ur.MakeModelFrame("ur5e")
		return model, nil
	}

	injectPT := &inject.PoseTracker{}
	injectPTName := posetracker.Named(ptName)

	injectMS := injectmotion.NewMotionService(motionName)

	deps := resource.Dependencies{
		injectArmName:   injectArm,
		injectPTName:    injectPT,
		injectMS.Name(): injectMS,
	}

	cfg := &Config{Arm: armName, PoseTracker: ptName}

	calService, err := NewArmCamera(ctx, deps, resource.NewName(generic.API, armCamCalName), cfg, logger)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, calService.Name().Name, test.ShouldEqual, armCamCalName)

}

func TestValidate(t *testing.T) {
	t.Run("No pt", func(t *testing.T) {
		cfg := &Config{Arm: armName}
		deps, optDeps, err := cfg.Validate("")
		test.That(t, err, test.ShouldBeError, resource.NewConfigValidationFieldRequiredError("", "tracker"))
		test.That(t, deps, test.ShouldBeEmpty)
		test.That(t, optDeps, test.ShouldBeEmpty)
	})
	t.Run("No arm", func(t *testing.T) {
		cfg := &Config{PoseTracker: ptName}
		deps, optDeps, err := cfg.Validate("")
		test.That(t, err, test.ShouldBeError, resource.NewConfigValidationFieldRequiredError("", "arm"))
		test.That(t, deps, test.ShouldBeEmpty)
		test.That(t, optDeps, test.ShouldBeEmpty)
	})
	t.Run("good config", func(t *testing.T) {
		cfg := &Config{PoseTracker: ptName, Arm: armName}
		deps, optDeps, err := cfg.Validate("")
		test.That(t, err, test.ShouldBeNil)
		test.That(t, deps, test.ShouldResemble, []string{armName, ptName, resource.NewName(motion.API, resource.DefaultServiceName).String()})
		test.That(t, optDeps, test.ShouldBeEmpty)
	})
}

func TestGetConvAttr(t *testing.T) {
	positions := [][]float64{{10}, {20}, {30}, {40}, {50}, {60}}
	orientation := spatialmath.NewOrientationVectorDegrees()
	orientation.OX = 1
	orientation.OY = 2
	orientation.OZ = 4
	orientation.Theta = 45
	point := r3.Vector{X: 100, Y: 200, Z: -300}
	pose := spatialmath.NewPose(point, orientation)
	guess := makeFrameCfg(armName, pose)
	cfg := &Config{Arm: armName, PoseTracker: ptName, JointPositions: positions, Guess: guess}
	attr := cfg.getConvertedAttributes()
	armAttr, ok := attr["arm"].(string)
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, armAttr, test.ShouldEqual, armName)
	ptAttr, ok := attr["tracker"].(string)
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, ptAttr, test.ShouldEqual, ptName)
	jointsAttr, ok := attr["joint_positions"].([][]float64)
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, len(jointsAttr), test.ShouldEqual, 6)
	test.That(t, jointsAttr[3][0], test.ShouldEqual, positions[3][0])
	// the link config is tested in TestMakeFrameCfg
	_, ok = attr["guess"].(referenceframe.LinkConfig)
	test.That(t, ok, test.ShouldBeTrue)
}

func TestDeletePositionFromArr(t *testing.T) {
	positions := [][]referenceframe.Input{{{Value: 0}},
		{{Value: 1}},
		{{Value: 2}},
		{{Value: 3}},
		{{Value: 4}},
		{{Value: 5}},
		{{Value: 6}},
	}
	t.Run("delete a position", func(t *testing.T) {
		index := 2
		newPositions, err := deletePositionFromArr(positions, index)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, len(newPositions), test.ShouldEqual, len(positions)-1)
		test.That(t, newPositions[index], test.ShouldResemble, positions[index+1])
	})
	t.Run("index out of range", func(t *testing.T) {
		index := 10
		_, err := deletePositionFromArr(positions, index)
		test.That(t, err, test.ShouldBeError, fmt.Errorf("index %v out of range %v", index, len(positions)))
	})
}

func TestMakeFrameCfg(t *testing.T) {
	orientation := spatialmath.NewOrientationVectorDegrees()
	orientation.OX = 1
	orientation.OY = 2
	orientation.OZ = 4
	orientation.Theta = 45
	point := r3.Vector{X: 100.0, Y: 200.0, Z: -300.0}
	pose := spatialmath.NewPose(point, orientation)
	cfg := makeFrameCfg(armName, pose)
	test.That(t, cfg.Parent, test.ShouldEqual, armName)
	// we want to test that the cfg can be parsed back into a pose by a viam-server
	newPose, err := cfg.Pose()
	test.That(t, err, test.ShouldBeNil)
	test.That(t, spatialmath.PoseAlmostEqual(newPose, pose), test.ShouldBeTrue)
}
