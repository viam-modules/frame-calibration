package framecalibration

import (
	"context"
	"errors"
	"fmt"
	"os"
	"reflect"
	"sync"
	"time"

	calutils "framecalibration/utils"

	"github.com/golang/geo/r3"
	armPb "go.viam.com/api/component/arm/v1"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/posetracker"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/utils"

	"github.com/erh/vmodutils"
	vizclient "github.com/viam-labs/motion-tools/client/client"
)

var (
	ArmCamera        = resource.NewModel("viam", "frame-calibration", "arm-camera")
	errUnimplemented = errors.New("unimplemented")
	errNoPoses       = errors.New("no poses are configured for the arm to move through")
	errNoDo          = errors.New("no valid DoCommand submitted")
)

const (
	vizKey                    = "viz"
	calibrateKey              = "runCalibration"
	moveArmKey                = "moveArm"
	checkTagsKey              = "checkTags"
	savePosKey                = "saveCalibrationPosition"
	saveAndUpdateKey          = "saveAndUpdatePosition"
	getPositionsKey           = "getCalibrationPositions"
	deletePosKey              = "deleteCalibrationPosition" // takes an index
	clearCalibrationPositions = "clearCalibrationPositions"
	updateConfigKey           = "updateCalibrationConfig"
	vizAddress                = "http://localhost:5173/"
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
	// joint positions are the easiest field for a user to access, but we may want to use poses in the config anyways
	// or we use both with some predefined logic
	JointPositions [][]float64 `json:"joint_positions"`
	// TODO: add geometries
}

func (cfg *Config) getConvertedAttributes() utils.AttributeMap {
	attrMap := utils.AttributeMap{
		"arm":             cfg.Arm,
		"tracker":         cfg.PoseTracker,
		"joint_positions": cfg.JointPositions,
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

	// add motion dependency
	deps = append(deps, resource.NewName(motion.API, resource.DefaultServiceName).String())

	return deps, nil, nil
}

type frameCalibrationArmCamera struct {
	name resource.Name

	logger      logging.Logger
	cfg         *Config
	poseTracker posetracker.PoseTracker
	arm         arm.Arm
	positions   [][]referenceframe.Input
	// poses         []spatialmath.Pose
	guess         spatialmath.Pose
	motion        motion.Service
	ws            *referenceframe.WorldState
	obstacles     *referenceframe.GeometriesInFrame
	cachedPlanDir string

	cancelCtx  context.Context
	cancelFunc func()
	mu         sync.Mutex
}

type positionOutput struct {
	Index    int       `json:"index"`
	Position []float64 `json:"position"`
}

func newFrameCalibrationArmCamera(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*Config](rawConf)
	if err != nil {
		return nil, err
	}

	return NewArmCamera(ctx, deps, rawConf.ResourceName(), conf, logger)

}

func NewArmCamera(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *Config, logger logging.Logger) (resource.Resource, error) {

	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	s := &frameCalibrationArmCamera{
		name:       name,
		logger:     logger,
		cancelCtx:  cancelCtx,
		cancelFunc: cancelFunc,
		cfg:        &Config{},
		guess:      spatialmath.NewZeroPose(),
	}

	if err := s.reconfigureWithConfig(ctx, deps, conf); err != nil {
		return nil, err
	}
	return s, nil
}

func (s *frameCalibrationArmCamera) Reconfigure(ctx context.Context, deps resource.Dependencies, rawConf resource.Config) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	conf, err := resource.NativeConfig[*Config](rawConf)
	if err != nil {
		return err
	}
	if err = s.reconfigureWithConfig(ctx, deps, conf); err != nil {
		return err
	}

	return nil
}

func (s *frameCalibrationArmCamera) reconfigureWithConfig(ctx context.Context, deps resource.Dependencies, conf *Config) error {
	var err error

	s.cachedPlanDir = os.Getenv("VIAM_MODULE_DATA")

	s.arm, err = arm.FromDependencies(deps, conf.Arm)
	if err != nil {
		return err
	}

	s.poseTracker, err = posetracker.FromDependencies(deps, conf.PoseTracker)
	if err != nil {
		return err
	}

	s.motion, err = motion.FromDependencies(deps, "builtin")
	if err != nil {
		return err
	}

	// always reconfigure positions
	s.positions = [][]referenceframe.Input{}
	for _, jointPos := range conf.JointPositions {
		pbPos := armPb.JointPositions{Values: jointPos}
		// This will break when kinematics update
		inputs := s.arm.ModelFrame().InputFromProtobuf(&pbPos)
		s.positions = append(s.positions, inputs)
	}

	// obstacle dimensions in mm
	// hardcoding geometry for testing
	obstaclePose := spatialmath.NewPose(r3.Vector{0, 0, -300}, &spatialmath.OrientationVectorDegrees{OZ: -1})
	obstacle, err := spatialmath.NewBox(obstaclePose, r3.Vector{100, 100, 600}, "box1")
	if err != nil {
		return err
	}

	geoms := referenceframe.NewGeometriesInFrame(
		conf.Arm,
		[]spatialmath.Geometry{obstacle},
	)
	s.obstacles = geoms

	ws, err := referenceframe.NewWorldState(
		[]*referenceframe.GeometriesInFrame{geoms},
		[]*referenceframe.LinkInFrame{},
	)
	if err != nil {
		return err
	}
	s.ws = ws

	s.cfg = conf

	return nil
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
			numAttempts, ok := value.(int)
			if !ok {
				// check if it was a string
				strAttempts, ok := value.(string)
				// if it wasn't a string or it was't empty, yell at the user
				if !ok || strAttempts != "" {
					resp["warn"] = fmt.Sprintf("the input should be a positive integer or an empty string")
				}
			}
			for i := range numAttempts {
				pose, err := s.calibrate(ctx)
				if err != nil {
					return nil, err
				}
				respNum := fmt.Sprintf("calibration pose %v", i)
				resp[respNum] = pose
				s.guess = pose
			}

		case vizKey:
			err := s.viz(ctx)
			if err != nil {
				return nil, fmt.Errorf("check that the viz server is running on your machine: %s", err.Error())
			}
			resp[vizKey] = "http://localhost:5173/"
			resp["note - viz"] = "you may need to rerun the command if the page is empty"
		case moveArmKey:
			secondsFloat, ok := value.(float64)
			if !ok {
				s.logger.Infof("expected int got %v", reflect.TypeOf(value))
			}
			seconds := int(secondsFloat)
			err := s.moveArm(ctx, seconds)
			if err != nil {
				return nil, err
			}
			resp[moveArmKey] = "success"
		case savePosKey:
			pos, err := s.arm.JointPositions(ctx, nil)
			if err != nil {
				return nil, err
			}
			s.positions = append(s.positions, pos)
			resp[savePosKey] = "joint position saved"
		case checkTagsKey:
			tags, err := calutils.DiscoverTags(ctx, s.poseTracker)
			if err != nil {
				return nil, err
			}
			resp[checkTagsKey] = fmt.Sprintf("number of tags seen: %v", len(tags))
		case saveAndUpdateKey:
			pos, err := s.arm.JointPositions(ctx, nil)
			if err != nil {
				return nil, err
			}
			s.positions = append(s.positions, pos)

			if err := s.updateCfg(ctx); err != nil {
				return nil, err
			}

			resp[saveAndUpdateKey] = fmt.Sprintf("joint position %v added to config", len(s.cfg.JointPositions))
			resp["note - config update"] = "config changes may take a few seconds to update"
		case deletePosKey:
			indexFloat, ok := value.(float64)
			if !ok {
				return nil, fmt.Errorf("removing position, expected int got %v", reflect.TypeOf(value))
			}
			index := int(indexFloat)
			if index > len(s.positions) {
				return nil, fmt.Errorf("index %v is out of range, only %v positions are set", reflect.TypeOf(value), len(s.positions))
			}
			s.positions = deletePositionFromArr(s.positions, index)
			resp[deletePosKey] = "position deleted"
			// fallthrough to print updated positions
			fallthrough
		case getPositionsKey:
			outputs := []positionOutput{}
			if len(s.positions) == 0 {
				return nil, errors.New("no positions are set")
			}
			for index, pos := range s.positions {
				jointFloats, err := referenceframe.JointPositionsFromInputs(s.arm.ModelFrame(), pos)
				if err != nil {
					return nil, err
				}
				out := positionOutput{Index: index, Position: jointFloats.Values}
				outputs = append(outputs, out)
			}
			resp[getPositionsKey] = outputs

		case clearCalibrationPositions:
			s.positions = [][]referenceframe.Input{}
			resp[clearCalibrationPositions] = "positions removed"
		case updateConfigKey:
			if err := s.updateCfg(ctx); err != nil {
				return nil, err
			}
			resp[updateConfigKey] = "config updated"
			resp["note - config update"] = "config changes may take a few seconds to update"

		default:
		}

	}

	if len(resp) == 0 {
		return nil, errNoDo
	}
	return resp, nil
}

func deletePositionFromArr(arr [][]referenceframe.Input, index int) [][]referenceframe.Input {
	newArr := make([][]referenceframe.Input, 0)
	newArr = append(newArr, arr[:index]...)
	newArr = append(newArr, arr[index+1:]...)
	return newArr
}
func (s *frameCalibrationArmCamera) updateCfg(ctx context.Context) error {
	// ensure cfg matches the current set of positions
	s.cfg.JointPositions = make([][]float64, 0, len(s.positions))
	for _, pos := range s.positions {
		jointFloats, err := referenceframe.JointPositionsFromInputs(s.arm.ModelFrame(), pos)
		if err != nil {
			return err
		}
		s.cfg.JointPositions = append(s.cfg.JointPositions, jointFloats.Values)
	}

	return vmodutils.UpdateComponentCloudAttributesFromModuleEnv(ctx, s.name, s.cfg.getConvertedAttributes(), s.logger)
}

func (s *frameCalibrationArmCamera) Close(context.Context) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	// Put close code here
	s.cancelFunc()
	return nil
}

func (s *frameCalibrationArmCamera) calibrationPoses() ([]spatialmath.Pose, error) {
	poses := []spatialmath.Pose{}
	for _, jointPos := range s.positions {
		newPose, err := s.arm.ModelFrame().Transform(jointPos)
		if err != nil {
			return nil, err
		}
		poses = append(poses, newPose)
	}
	return poses, nil
}

func (s *frameCalibrationArmCamera) viz(ctx context.Context) error {
	armGeos, err := s.arm.Geometries(ctx, nil)
	if err != nil {
		return err
	}
	armGeomsInF := referenceframe.NewGeometriesInFrame(
		referenceframe.World,
		armGeos,
	)

	wsDrawing, err := referenceframe.NewWorldState(
		[]*referenceframe.GeometriesInFrame{s.obstacles, armGeomsInF},
		[]*referenceframe.LinkInFrame{},
	)
	if err != nil {
		return err
	}
	fs := referenceframe.NewEmptyFrameSystem("blah")
	frame0 := referenceframe.NewZeroStaticFrame(s.cfg.Arm)
	err = fs.AddFrame(frame0, fs.World())
	if err != nil {
		return err
	}
	if err := vizclient.RemoveAllSpatialObjects(); err != nil {
		return err
	}

	return vizclient.DrawWorldState(wsDrawing, fs, referenceframe.NewZeroInputs(fs))
}

func (s *frameCalibrationArmCamera) calibrate(ctx context.Context) (spatialmath.Pose, error) {
	if len(s.positions) == 0 {
		return nil, errNoPoses
	}
	// get poses
	poses, err := s.calibrationPoses()
	if err != nil {
		return nil, err
	}
	// move to initial pose.
	constraints := motionplan.NewEmptyConstraints()
	posInF := referenceframe.NewPoseInFrame(referenceframe.World, poses[0])
	req := motion.MoveReq{ComponentName: s.arm.Name(), Destination: posInF, WorldState: s.ws, Constraints: constraints}
	if _, err = s.motion.Move(ctx, req); err != nil {
		return nil, err
	}

	// discover tags for pose estimation
	tags, err := calutils.DiscoverTags(ctx, s.poseTracker)
	if err != nil {
		return nil, err
	}

	dataCfg := calutils.DataConfig{DataPath: s.cachedPlanDir, SaveNewData: false, LoadOldDataset: false}
	estimateReq := calutils.ReqFramePoseWithMotion{Arm: s.arm,
		Motion: s.motion, PoseTracker: s.poseTracker, ExpectedTags: tags, CalibrationPoses: poses, SeedPose: s.guess, WS: s.ws}

	return calutils.EstimateFramePoseWithMotion(ctx, estimateReq, dataCfg, s.logger)
}

func (s *frameCalibrationArmCamera) moveArm(ctx context.Context, delay int) error {
	if len(s.positions) == 0 {
		return errNoPoses
	}
	// get poses
	poses, err := s.calibrationPoses()
	if err != nil {
		return err
	}

	if delay == 0 {
		delay = 1
	}

	constraints := motionplan.NewEmptyConstraints()
	for index, pos := range poses {
		s.logger.Debugf("moving to position %v, pose %v", index, pos)
		posInF := referenceframe.NewPoseInFrame(referenceframe.World, pos)

		req := motion.MoveReq{ComponentName: s.arm.Name(), Destination: posInF, WorldState: s.ws, Constraints: constraints}
		_, err := s.motion.Move(ctx, req)
		if err != nil {
			return err
		}
		// sleep to give time to check camera
		time.Sleep(time.Duration(delay) * time.Second)

	}

	return nil

}
