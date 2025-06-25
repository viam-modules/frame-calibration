package framecalibration

import (
	"context"
	"errors"
	"fmt"
	"math"
	"reflect"
	"sync"
	"time"

	"github.com/golang/geo/r3"

	calutils "framecalibration/utils"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/posetracker"
	toggleswitch "go.viam.com/rdk/components/switch"
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
	Motion      string `json:"motion"`
	ArmParent   string `json:"arm_parent"`
	AutoStart   string `json:"auto_start"`

	// joint positions are the easiest field for a user to access, but we may want to use poses in the config anyways
	// or we use both with some predefined logic
	JointPositions [][]float64               `json:"joint_positions"`
	Guess          referenceframe.LinkConfig `json:"guess"`

	SleepSeconds float64 `json:"sleep_seconds"`
	ExpectedTags int     `json:"num_expected_tags"`
}

func (cfg *Config) sleepTime() time.Duration {
	if cfg.SleepSeconds <= 0 {
		return time.Second
	}

	return time.Duration(float64(time.Second) * cfg.SleepSeconds)
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
		if cfg.ArmParent == "" {
			return nil, nil, fmt.Errorf("if motion not empty, need an arm_parent")
		}
		deps = append(deps, motion.Named(cfg.Motion).String())
	}

	if cfg.AutoStart != "" {
		deps = append(deps, cfg.AutoStart)
	}

	return deps, nil, nil
}

type FrameCalibrationArmCamera struct {
	resource.AlwaysRebuild
	resource.TriviallyCloseable

	name   resource.Name
	cfg    *Config
	logger logging.Logger

	poseTracker posetracker.PoseTracker
	arm         arm.Arm
	armModel    referenceframe.Model

	motion    motion.Service      // could be nil
	autoStart toggleswitch.Switch // could be nil

	mu sync.Mutex
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

func NewArmCamera(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *Config, logger logging.Logger) (*FrameCalibrationArmCamera, error) {

	s := &FrameCalibrationArmCamera{
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

	if conf.AutoStart != "" {
		s.autoStart, err = toggleswitch.FromDependencies(deps, conf.AutoStart)
		if err != nil {
			return nil, err
		}
	}
	// default number of tags to see is 24.
	if conf.ExpectedTags == 0 {
		conf.ExpectedTags = 24
	}

	s.cfg = conf

	return s, nil
}

func (s *FrameCalibrationArmCamera) Name() resource.Name {
	return s.name
}

func (s *FrameCalibrationArmCamera) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	resp := map[string]interface{}{}
	for key, value := range cmd {
		switch key {
		case "runCalibration":
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
				pose, cost, err := s.Calibrate(ctx)
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
			resp["runCalibration"] = output
		case "moveArm":
			numTags, err := s.moveArm(ctx)
			if err != nil {
				s.logger.Error(err)
				return nil, err
			}
			resp["moveArm"] = "success"
			resp["tags seen"] = numTags
		case "checkTags":
			poses, err := s.poseTracker.Poses(ctx, nil, nil)
			if err != nil {
				s.logger.Error(err)
				return nil, err
			}
			resp["checkTags"] = fmt.Sprintf("number of tags seen: %v", len(poses))
		case "saveCalibrationPosition":
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
				resp["saveCalibrationPosition"] = fmt.Sprintf("joint position %v added to config", len(s.cfg.JointPositions)-1)
			case index >= len(s.cfg.JointPositions):
				return nil, fmt.Errorf("index %v is out of range, only %v positions are set", reflect.TypeOf(value), len(s.cfg.JointPositions))
			default:
				s.cfg.JointPositions[index] = referenceframe.InputsToFloats(pos)
				resp["saveCalibrationPosition"] = fmt.Sprintf("joint position %v updated in config", index)
			}
			if err := s.updateCfg(ctx); err != nil {
				s.logger.Error(err)
				return nil, err
			}
			resp["note - config update"] = "config changes may take a few seconds to update"
		case "moveArmToPosition":
			indexFloat, ok := value.(float64)
			if !ok {
				return nil, fmt.Errorf("moving position, expected int got %v", reflect.TypeOf(value))
			}
			index := int(indexFloat)

			if _, _, err := s.MoveToSavedPosition(ctx, index); err != nil {
				s.logger.Error(err)
				return nil, err
			}
			// sleep to give time to check camera
			if !utils.SelectContextOrWait(ctx, s.cfg.sleepTime()) {
				return nil, ctx.Err()
			}
			// discover tags for pose estimation
			tags, err := s.poseTracker.Poses(ctx, nil, nil)
			if err != nil {
				s.logger.Error(err)
				return nil, err
			}
			resp["moveArmToPosition"] = len(tags)

		case "deleteCalibrationPosition":
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
			resp["deleteCalibrationPosition"] = "position deleted"
		case "clearCalibrationPositions":
			s.cfg.JointPositions = [][]float64{}
			s.cfg.Guess = makeFrameCfg(s.arm.Name().Name, spatialmath.NewZeroPose())
			resp["clearCalibrationPositions"] = "positions removed"
			if err := s.updateCfg(ctx); err != nil {
				s.logger.Error(err)
				return nil, err
			}
		case "autoCalibrate":
			node, err := s.AutoCalibrate(ctx)
			if err != nil {
				return nil, err
			}
			resp["autoCalibrate"] = calOutput{Frame: makeFrameCfg(s.arm.Name().Name, node.Pose()), Cost: node.Cost}
		case "saveGuess":
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

func (s *FrameCalibrationArmCamera) updateCfg(ctx context.Context) error {
	return vmodutils.UpdateComponentCloudAttributesFromModuleEnv(ctx, s.name, s.cfg.getConvertedAttributes(), s.logger)
}

func (s *FrameCalibrationArmCamera) Calibrate(ctx context.Context) (spatialmath.Pose, float64, error) {
	if len(s.cfg.JointPositions) == 0 {
		return nil, 0, errNoPoses
	}

	seed, err := s.cfg.Guess.Pose()
	if err != nil {
		return nil, 0, fmt.Errorf("cannot parse guess %w", err)
	}

	estimateReq := calutils.ReqFramePose{
		PoseTracker: s.poseTracker,
		Mover:       s,
		SeedPose:    seed,
	}

	return calutils.EstimateFramePose(ctx, estimateReq, s.logger)
}

func (s *FrameCalibrationArmCamera) moveArm(ctx context.Context) ([]int, error) {
	if len(s.cfg.JointPositions) == 0 {
		return nil, errNoPoses
	}

	numTags := make([]int, 0)
	for index := range s.NumPositions() {
		s.logger.Debugf("moving to position %v, pose %v", index, s.cfg.JointPositions[index])

		if _, _, err := s.MoveToSavedPosition(ctx, index); err != nil {
			return nil, err
		}
		// sleep to give time to check camera
		if !utils.SelectContextOrWait(ctx, s.cfg.sleepTime()) {
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

func (s *FrameCalibrationArmCamera) NumPositions() int {
	return len(s.cfg.JointPositions)
}

func (s *FrameCalibrationArmCamera) ArmPosition(ctx context.Context) ([]referenceframe.Input, spatialmath.Pose, error) {
	j, err := calutils.AverageJointPosition(ctx, s.arm, 100) // TODO: is this really needed
	if err != nil {
		return nil, nil, err
	}
	p, err := s.armModel.Transform(j)
	if err != nil {
		return nil, nil, err
	}
	return j, p, nil
}

func (s *FrameCalibrationArmCamera) MoveToSavedPosition(ctx context.Context, pos int) ([]referenceframe.Input, spatialmath.Pose, error) {
	if pos >= len(s.cfg.JointPositions) {
		return nil, nil, fmt.Errorf("pos %d invalid (%d)", pos, len(s.cfg.JointPositions))
	}

	inp := referenceframe.FloatsToInputs(s.cfg.JointPositions[pos])

	s.logger.Debugf("MoveToSavedPosition pos: %d raw: %v input: %v", pos, s.cfg.JointPositions[pos], inp)

	if s.motion == nil {
		err := s.arm.MoveToJointPositions(ctx, inp, nil)
		if err != nil {
			return nil, nil, err
		}
		return s.ArmPosition(ctx)
	}

	goalPose, err := s.armModel.Transform(inp)
	if err != nil {
		return nil, nil, err
	}

	posInF := referenceframe.NewPoseInFrame(s.cfg.ArmParent, goalPose)

	req := motion.MoveReq{ComponentName: s.arm.Name(), Destination: posInF, Extra: map[string]any{"timeout": 30}}

	if _, err := s.motion.Move(ctx, req); err != nil {
		return nil, nil, err
	}

	return s.ArmPosition(ctx)
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

func (s *FrameCalibrationArmCamera) ArmPositionAndPoses(ctx context.Context) (calutils.ArmAndPoses, error) {

	poses, err := calutils.GetPoses(ctx, s.poseTracker)
	if err != nil {
		return calutils.ArmAndPoses{}, err
	}

	jj, pp, err := s.ArmPosition(ctx)
	if err != nil {
		return calutils.ArmAndPoses{}, err
	}

	return calutils.ArmAndPoses{jj, calutils.NewSimplePose(pp), poses}, nil
}

func (s *FrameCalibrationArmCamera) roughGuessHelp2(ctx context.Context, start []referenceframe.Input, joint int, jogAbsoluteStep, jogAbsoluteMax float64) (calutils.ArmAndPoses, error) {
	prev := calutils.ArmAndPoses{}

	initial := 1.0
	if jogAbsoluteStep < 0 {
		initial *= -1
	}

	for step := initial; math.Abs(step) <= math.Abs(jogAbsoluteMax); step += jogAbsoluteStep {
		j := make([]referenceframe.Input, len(start))
		for idx, jj := range start {
			j[idx] = jj
		}

		j[joint].Value += (step * math.Pi / 180)

		err := s.arm.MoveToJointPositions(ctx, j, nil)
		if err != nil {
			return calutils.ArmAndPoses{}, err
		}

		time.Sleep(s.cfg.sleepTime())

		data, err := s.ArmPositionAndPoses(ctx)

		s.logger.Infof("joint %d change: %0.2f -> %d", joint, step, len(data.Tags))

		if err != nil {
			if len(prev.Tags) > 0 {
				return prev, nil
			}
			return data, err
		}
		if len(data.Tags) < len(prev.Tags) {
			break
		}
		prev = data
	}
	return prev, nil
}

func (s FrameCalibrationArmCamera) roughGuessHelp(ctx context.Context, data []calutils.ArmAndPoses, joint int, jogAbsoluteStep, jogAbsoluteMax float64) ([]calutils.ArmAndPoses, error) {
	dd, err := s.roughGuessHelp2(ctx, data[0].Joints, joint, -1*jogAbsoluteStep, -1*jogAbsoluteMax)
	if err != nil {
		return nil, err
	}
	if len(dd.Tags) < 5 {
		return nil, fmt.Errorf("too few tags for negative look j:%d %d", joint, len(dd.Tags))
	}
	data = append(data, dd)

	dd, err = s.roughGuessHelp2(ctx, data[0].Joints, joint, jogAbsoluteStep, jogAbsoluteMax)
	if err != nil {
		return nil, err
	}
	if len(dd.Tags) < s.cfg.ExpectedTags-4 {
		return nil, fmt.Errorf("too few tags for positive look j:%d %d", joint, len(dd.Tags))
	}
	data = append(data, dd)

	return data, nil
}

func (s *FrameCalibrationArmCamera) RoughGuess(ctx context.Context) (*calutils.BasicNode, []calutils.ArmAndPoses, error) {
	if s.autoStart != nil {
		err := s.autoStart.SetPosition(ctx, 2, nil)
		if err != nil {
			return nil, nil, err
		}
		time.Sleep(s.cfg.sleepTime())
	}

	dd, err := s.ArmPositionAndPoses(ctx)
	if err != nil {
		return nil, nil, err
	}

	data := []calutils.ArmAndPoses{dd}

	for j := 0; j < len(s.armModel.DoF()); j++ {
		data, err = s.roughGuessHelp(ctx, data, j, 2, 20)
		if err != nil {
			return nil, nil, err
		}
	}

	s.logger.Infof("RoughGuess num data points: %d", len(data))

	res, err := calutils.Minimize(ctx, data, spatialmath.NewZeroPose(), s.logger)
	if err != nil {
		return nil, nil, err
	}

	err = calutils.WriteData(data, res[0].Pose(), s.logger)
	if err != nil {
		s.logger.Warnf("couldn't write data, but continuing: %v", err)
	}

	return &res[0], data, nil
}

func (s FrameCalibrationArmCamera) AutoCalibrate(ctx context.Context) (*calutils.BasicNode, error) {
	guessPose, data, err := s.RoughGuess(ctx)
	if err != nil {
		return guessPose, err
	}

	s.logger.Infof("initial guess of %v", guessPose)

	if s.motion == nil {
		return guessPose, nil
	}

	if s.autoStart != nil {
		err := s.autoStart.SetPosition(ctx, 2, nil)
		if err != nil {
			return guessPose, err
		}
		time.Sleep(s.cfg.sleepTime())
	}

	poses, err := calutils.GetPoses(ctx, s.poseTracker)
	if err != nil {
		return guessPose, err
	}

	if len(poses) < s.cfg.ExpectedTags {
		return guessPose, fmt.Errorf("got %d poses, expected at least %v", len(poses), s.cfg.ExpectedTags)
	}

	guess := referenceframe.NewLinkInFrame(
		s.cfg.Arm,
		guessPose.Pose(),
		"guess",
		nil,
	)

	ws, err := referenceframe.NewWorldState(nil, []*referenceframe.LinkInFrame{guess})
	if err != nil {
		return guessPose, err
	}

	start, err := s.motion.GetPose(ctx, resource.Name{Name: "guess"}, "world", []*referenceframe.LinkInFrame{guess}, nil)
	if err != nil {
		return guessPose, err
	}

	s.logger.Infof("start: %v", start)

	allPosesToTry := allPoseMods(start.Pose())

	for idx, mod := range allPosesToTry {
		next := referenceframe.NewPoseInFrame(
			"world",
			mod,
		)

		s.logger.Infof("mod:%v (%d/%d)", mod, idx, len(allPosesToTry))

		_, err = s.motion.Move(
			ctx,
			motion.MoveReq{
				ComponentName: resource.Name{Name: "guess"},
				Destination:   next,
				WorldState:    ws,
			},
		)
		if err != nil {
			s.logger.Debugf("cannot go there %v", err)
			continue
		}

		time.Sleep(s.cfg.sleepTime())

		dd, err := s.ArmPositionAndPoses(ctx)
		if err != nil {
			return guessPose, err
		}

		if len(dd.Tags) < len(poses)-6 {
			s.logger.Infof("\t bad poses after move %d", len(dd.Tags))
			continue
		}

		s.logger.Infof("GOOD!")

		data = append(data, dd)
	}

	s.logger.Infof("number of good positions: %d", len(data))

	sol, err := calutils.Minimize(ctx, data, guessPose.Pose(), s.logger)
	if err != nil {
		return guessPose, err
	}

	p := sol[0]
	s.logger.Info("Optimization Guess: ", p.Pose(), p.Pose)

	err = calutils.WriteData(data, p.Pose(), s.logger)
	if err != nil {
		s.logger.Warnf("couldn't write data, but continuing: %v", err)
	}

	return &p, nil
}

func allPoseMods(start spatialmath.Pose) []spatialmath.Pose {
	const r = .04
	const step = .04

	startPoint := start.Point()
	startOrientation := start.Orientation().OrientationVectorDegrees()

	all := []spatialmath.Pose{}

	didZero := false

	for a := -1 * r; a <= r; a += step {

		// these account for orientation offsets
		oChoices := []r3.Vector{
			{X: a},
			{Y: a},
			{Z: a},
		}

		// these account for lateral offsets
		pChoices := []r3.Vector{
			{},
		}

		for _, to := range oChoices {
			for _, tp := range pChoices {
				for th := -45.0; th <= 45; th += 45 {

					if tp.X == 0 && tp.Y == 0 && tp.Z == 0 && to.X == 0 && to.Y == 0 && to.Z == 0 && th == 0 {
						if didZero {
							continue
						}
						didZero = true
					}

					all = append(all, spatialmath.NewPose(
						r3.Vector{
							X: startPoint.X + tp.X,
							Y: startPoint.Y + tp.Y,
							Z: startPoint.Z + tp.Z,
						},
						&spatialmath.OrientationVectorDegrees{
							OX:    startOrientation.OX + to.X,
							OY:    startOrientation.OY + to.Y,
							OZ:    startOrientation.OZ + to.Z,
							Theta: startOrientation.Theta + th,
						},
					))
				}
			}
		}
	}

	return all
}
