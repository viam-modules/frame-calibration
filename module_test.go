package framecalibration

import (
	"context"
	"fmt"
	"testing"

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

	injectArm.ModelFrameFunc = func() referenceframe.Model {
		model, _ := ur.MakeModelFrame("ur5e")
		return model
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
		test.That(t, newPositions[index], test.ShouldEqual, positions[index+1])
	})
	t.Run("index out of range", func(t *testing.T) {
		index := 10
		_, err := deletePositionFromArr(positions, index)
		test.That(t, err, test.ShouldBeError, fmt.Errorf("index %v out of range %v", index, len(positions)))
	})
}
