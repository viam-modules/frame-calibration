package framecalibration

import (
	"context"
	"errors"
	"fmt"
	"reflect"
	"sync"
	"time"

	calutils "framecalibration/utils"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/posetracker"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
	rdkutils "go.viam.com/rdk/utils"
	"go.viam.com/utils"

	"github.com/erh/vmodutils"
)

var (
	ArmCamera        = resource.NewModel("viam", "frame-calibration", "camera-on-arm")
	errUnimplemented = errors.New("unimplemented")
	errNoPoses       = errors.New("no poses are configured for the arm to move through")
	errNoDo          = errors.New("no valid DoCommand submitted")
)

const (
	// check READMe DoCommands or Module docs for a full explanation of what these commands do.
	calibrateKey              = "runCalibration"
	moveArmKey                = "moveArm"
	moveArmIndexKey           = "moveArmToPosition"
	numSeenTagsKey            = "checkTags"
	saveAndUpdateKey          = "saveCalibrationPosition"
	getPositionsKey           = "getCalibrationPositions"
	deletePosKey              = "deleteCalibrationPosition"
	clearCalibrationPositions = "clearCalibrationPositions"
)

func init() {
	resource.RegisterService(generic.API, ArmCamera,
		resource.Registration[resource.Resource, *Config]{
			Constructor: newFrameCalibrationArmCamera,
		},
	)
}

type Config struct {
	Arm         string `json:"arm"`
	PoseTracker string `json:"tracker"`
	Motion      string

	// joint positions are the easiest field for a user to access, but we may want to use poses in the config anyways
	// or we use both with some predefined logic
	JointPositions [][]float64               `json:"joint_positions"`
	Guess          referenceframe.LinkConfig `json:"guess"`
}

func (cfg *Config) getConvertedAttributes() rdkutils.AttributeMap {
	attrMap := rdkutils.AttributeMap{
		"arm":             cfg.Arm,
		"tracker":         cfg.PoseTracker,
		"joint_positions": cfg.JointPositions,
		"guess":           cfg.Guess,
	}

	return attrMap

}

// Validate ensures all parts of the config are valid and important fields exist.
// Returns implicit dependencies based on the config.
// The path is the JSON path in your robot's config (not the `Config` struct) to the
// resource being validated; e.g. "components.0".
func (cfg *Config) Validate(path string) ([]string, []string, error) {
	// Add config validation code here
	var deps []string
	if cfg.Arm == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "arm")
	}
	deps = append(deps, cfg.Arm)

	if cfg.PoseTracker == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "tracker")
	}
	deps = append(deps, cfg.PoseTracker)

	if cfg.Motion != "" {
		deps = append(deps, motion.Named(cfg.Motion).String())
	}

	return deps, nil, nil
}

type frameCalibrationArmCamera struct {
	resource.AlwaysRebuild
	resource.TriviallyCloseable

	name   resource.Name
	cfg    *Config
	logger logging.Logger

	poseTracker posetracker.PoseTracker
	arm         arm.Arm
	armModel    referenceframe.Model
	motion      motion.Service // could be nil

	mu sync.Mutex
}

type positionOutput struct {
	Index    int       `json:"index"`
	Position []float64 `json:"position"`
}

type calOutput struct {
	Frame referenceframe.LinkConfig `json:"frame"`
	Cost  float64                   `json:"cost"`
}

func newFrameCalibrationArmCamera(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*Config](rawConf)
	if err != nil {
		return nil, err
	}

	return NewArmCamera(ctx, deps, rawConf.ResourceName(), conf, logger)

}

func NewArmCamera(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *Config, logger logging.Logger) (resource.Resource, error) {

	s := &frameCalibrationArmCamera{
		name:   name,
		logger: logger,
		cfg:    conf,
	}

	var err error

	s.arm, err = arm.FromDependencies(deps, conf.Arm)
	if err != nil {
		return nil, err
	}

	s.armModel, err = s.arm.Kinematics(ctx)
	if err != nil {
		return nil, err
	}

	s.poseTracker, err = posetracker.FromDependencies(deps, conf.PoseTracker)
	if err != nil {
		return nil, err
	}

	if conf.Motion != "" {
		s.motion, err = motion.FromDependencies(deps, conf.Motion)
		if err != nil {
			return nil, err
		}
	}

	s.cfg = conf

	return s, nil
}

func (s *frameCalibrationArmCamera) Name() resource.Name {
	return s.name
}

func (s *frameCalibrationArmCamera) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	resp := map[string]interface{}{}
	for key, value := range cmd {
		switch key {
		case calibrateKey:
			numAttempts, ok := value.(float64)
			if !ok {
				// check if it was a string
				strAttempts, ok := value.(string)
				// if it wasn't a number or an empty string, yell at the user
				if !ok || strAttempts != "" {
					resp["warn"] = "the input should be a positive integer or an empty string"
				}
				numAttempts = 1
			}
			if numAttempts < 1 {
				resp["warn"] = "number of attempts should be one or greater setting the number of calbration attempts to 1"
				numAttempts = 1
			}
			intNumAttempts := int(numAttempts)

			output := make([]calOutput, 0, intNumAttempts)
			for range intNumAttempts {
				output = append(output, calOutput{})
			}

			for i := range intNumAttempts {
				pose, cost, err := s.calibrate(ctx)
				if err != nil {
					s.logger.Error(err)
					return nil, err
				}
				output[intNumAttempts-i-1] = calOutput{Frame: makeFrameCfg(s.arm.Name().Name, pose), Cost: cost}
				// store the result to use in the next calibration
				s.cfg.Guess = makeFrameCfg(s.arm.Name().Name, pose)
			}
			// update the config with the calibration result
			if err := s.updateCfg(ctx); err != nil {
				s.logger.Error(err)
				return nil, err
			}
			resp["note - calibration"] = "use the frame with the lowest cost score. For more reliable results, increase the number of attempts"
			resp[calibrateKey] = output
		case moveArmKey:
			secondsFloat, ok := value.(float64)
			if !ok {
				s.logger.Infof("expected int got %v", reflect.TypeOf(value))
			}
			seconds := int(secondsFloat)
			numTags, err := s.moveArm(ctx, seconds)
			if err != nil {
				s.logger.Error(err)
				return nil, err
			}
			resp[moveArmKey] = "success"
			resp["tags seen"] = numTags
		case numSeenTagsKey:
			poses, err := s.poseTracker.Poses(ctx, nil, nil)
			if err != nil {
				s.logger.Error(err)
				return nil, err
			}
			resp[numSeenTagsKey] = fmt.Sprintf("number of tags seen: %v", len(poses))
		case saveAndUpdateKey:
			indexFloat, ok := value.(float64)
			if !ok {
				// check if it was a string
				indexStr, ok := value.(string)
				// if it wasn't a number or an empty string, yell at the user
				if !ok || indexStr != "" {
					resp["warn"] = "the input should be a positive integer or an empty string"
				}
				indexFloat = -1
			}
			index := int(indexFloat)

			pos, err := s.arm.JointPositions(ctx, nil)
			if err != nil {
				s.logger.Error(err)
				return nil, err
			}

			switch {
			case index < 0:
				s.cfg.JointPositions = append(s.cfg.JointPositions, referenceframe.InputsToFloats(pos))
				resp[saveAndUpdateKey] = fmt.Sprintf("joint position %v added to config", len(s.cfg.JointPositions)-1)
			case index >= len(s.cfg.JointPositions):
				return nil, fmt.Errorf("index %v is out of range, only %v positions are set", reflect.TypeOf(value), len(s.cfg.JointPositions))
			default:
				s.cfg.JointPositions[index] = referenceframe.InputsToFloats(pos)
				resp[saveAndUpdateKey] = fmt.Sprintf("joint position %v updated in config", index)
			}
			if err := s.updateCfg(ctx); err != nil {
				s.logger.Error(err)
				return nil, err
			}
			resp["note - config update"] = "config changes may take a few seconds to update"
		case moveArmIndexKey:
			indexFloat, ok := value.(float64)
			if !ok {
				return nil, fmt.Errorf("moving position, expected int got %v", reflect.TypeOf(value))
			}
			index := int(indexFloat)

			if err := s.MoveToSavedPosition(ctx, index); err != nil {
				s.logger.Error(err)
				return nil, err
			}
			// sleep to give time to check camera
			if !utils.SelectContextOrWait(ctx, 1*time.Second) {
				return nil, ctx.Err()
			}
			// discover tags for pose estimation
			tags, err := s.poseTracker.Poses(ctx, nil, nil)
			if err != nil {
				s.logger.Error(err)
				return nil, err
			}
			resp[moveArmIndexKey] = len(tags)

		case deletePosKey:
			indexFloat, ok := value.(float64)
			if !ok {
				return nil, fmt.Errorf("removing position, expected int got %v", reflect.TypeOf(value))
			}
			index := int(indexFloat)
			pos, err := deletePositionFromArr(s.cfg.JointPositions, index)
			if err != nil {
				return nil, err
			}
			s.cfg.JointPositions = pos
			resp[deletePosKey] = "position deleted"
		case clearCalibrationPositions:
			s.cfg.JointPositions = [][]float64{}
			s.cfg.Guess = makeFrameCfg(s.arm.Name().Name, spatialmath.NewZeroPose())
			resp[clearCalibrationPositions] = "positions removed"
			if err := s.updateCfg(ctx); err != nil {
				s.logger.Error(err)
				return nil, err
			}

		default:
			resp[key] = "unsupported key"
		}
	}

	if len(resp) == 0 {
		return nil, errNoDo
	}
	return resp, nil
}

func deletePositionFromArr(arr [][]float64, index int) ([][]float64, error) {
	if index >= len(arr) {
		return nil, fmt.Errorf("index %v out of range %v", index, len(arr))
	}
	newArr := make([][]float64, 0)
	newArr = append(newArr, arr[:index]...)
	newArr = append(newArr, arr[index+1:]...)
	return newArr, nil
}

func (s *frameCalibrationArmCamera) updateCfg(ctx context.Context) error {
	return vmodutils.UpdateComponentCloudAttributesFromModuleEnv(ctx, s.name, s.cfg.getConvertedAttributes(), s.logger)
}

func (s *frameCalibrationArmCamera) calibrate(ctx context.Context) (spatialmath.Pose, float64, error) {
	if len(s.cfg.JointPositions) == 0 {
		return nil, 0, errNoPoses
	}

	// move to initial pose.
	// why are we doing this?
	err := s.MoveToSavedPosition(ctx, 0)
	if err != nil {
		return nil, 0, err
	}

	seed, err := s.cfg.Guess.Pose()
	if err != nil {
		return nil, 0, fmt.Errorf("cannot parse guess %w", err)
	}

	estimateReq := calutils.ReqFramePose{
		Arm:         s.arm,
		PoseTracker: s.poseTracker,
		Mover:       s,
		SeedPose:    seed,
	}

	return calutils.EstimateFramePose(ctx, estimateReq, s.logger)
}

func (s *frameCalibrationArmCamera) moveArm(ctx context.Context, delay int) ([]int, error) {
	if len(s.cfg.JointPositions) == 0 {
		return nil, errNoPoses
	}

	if delay == 0 {
		delay = 1
	}

	numTags := make([]int, 0)
	for index := range s.NumPositions() {
		s.logger.Debugf("moving to position %v, pose %v", index, s.cfg.JointPositions[index])

		if err := s.MoveToSavedPosition(ctx, index); err != nil {
			return nil, err
		}
		// sleep to give time to check camera
		if !utils.SelectContextOrWait(ctx, time.Duration(delay)*time.Second) {
			return nil, ctx.Err()
		}
		tags, err := s.poseTracker.Poses(ctx, nil, nil)
		if err != nil {
			return nil, err
		}
		numTags = append(numTags, len(tags))

	}

	return numTags, nil

}

func (s *frameCalibrationArmCamera) NumPositions() int {
	return len(s.cfg.JointPositions)
}

func (s *frameCalibrationArmCamera) MoveToSavedPosition(ctx context.Context, pos int) error {
	if pos >= len(s.cfg.JointPositions) {
		return fmt.Errorf("pos %d invalid (%d)", pos, len(s.cfg.JointPositions))
	}

	inp := referenceframe.FloatsToInputs(s.cfg.JointPositions[pos])

	s.logger.Infof("MoveToSavedPosition pos: %d raw: %v input: %v", pos, s.cfg.JointPositions[pos], inp)

	if s.motion == nil {
		return s.arm.MoveToJointPositions(ctx, inp, nil)
	}

	goalPose, err := s.armModel.Transform(inp)
	if err != nil {
		return err
	}

	posInF := referenceframe.NewPoseInFrame(s.cfg.Arm, goalPose)

	req := motion.MoveReq{ComponentName: s.arm.Name(), Destination: posInF}
	if _, err := s.motion.Move(ctx, req); err != nil {
		return err
	}
	return nil
}

func makeFrameCfg(arm string, pose spatialmath.Pose) referenceframe.LinkConfig {
	orientationMap := map[string]any{}
	orientationMap["x"] = pose.Orientation().OrientationVectorDegrees().OX
	orientationMap["y"] = pose.Orientation().OrientationVectorDegrees().OY
	orientationMap["z"] = pose.Orientation().OrientationVectorDegrees().OZ
	orientationMap["th"] = pose.Orientation().OrientationVectorDegrees().Theta

	orientCfg := spatialmath.OrientationConfig{Type: spatialmath.OrientationVectorDegreesType, Value: orientationMap}
	return referenceframe.LinkConfig{Translation: pose.Point(), Orientation: &orientCfg, Parent: arm}
}
