package framecalibration

import (
	"context"
	"errors"
	"fmt"
	"os"
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

	vizclient "github.com/viam-labs/motion-tools/client/client"
)

var (
	ArmCamera        = resource.NewModel("viam", "frame-calibration", "arm-camera")
	errUnimplemented = errors.New("unimplemented")
	errNoPoses       = errors.New("no poses are configured for the arm to move through")
	errNoDo          = errors.New("no valid DoCommand submitted")
)

const (
	vizKey       = "viz"
	calibrateKey = "runCalibration"
	moveArmKey   = "moveArm"
	vizAddress   = "http://localhost:5173/"
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

	logger        logging.Logger
	cfg           *Config
	poseTracker   posetracker.PoseTracker
	arm           arm.Arm
	positions     [][]referenceframe.Input
	poses         []spatialmath.Pose
	guess         spatialmath.Pose
	motion        motion.Service
	ws            *referenceframe.WorldState
	obstacles     *referenceframe.GeometriesInFrame
	cachedPlanDir string

	cancelCtx  context.Context
	cancelFunc func()
	mu         sync.Mutex
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

	//hardcoding positions for testing
	s.positions = [][]referenceframe.Input{{{1.5687246322631836}, {-2.8689214191832484}, {0.8763092199908655}, {1.413437767619751}, {-0.9621284643756312}, {-0.009279553090230763}},
		{{1.5874707698822021}, {-3.372202535668844}, {0.8752110640155237}, {2.0592481332966304}, {-1.091687027608053}, {-0.009339634572164357}},
		{{1.599494218826294}, {-1.6834622822203578}, {-1.0425125360488892}, {2.058858795756958}, {-1.4727914969073692}, {0.3423517644405365}},
		{{1.0668467283248901}, {-2.0526958904662074}, {-1.5102773904800415}, {3.1842085558125}, {-1.9495976606952115}, {0.3422858119010926}},
		{{2.710968494415283}, {-1.8869949779906214}, {-1.8658866882324219}, {3.8552242952534175}, {0.2258821278810501}, {0.3423832952976227}},
		{{2.4088399410247803}, {-2.73096813778066}, {-0.4833677113056183}, {4.110974951381348}, {0.22581766545772552}, {0.3424190878868103}},
		{{2.2999038696289062}, {-2.7309800587096156}, {-0.4835145175457001}, {4.110954924220703}, {0.22584037482738495}, {0.34241911768913275}},
		{{2.338528156280518}, {-2.9684068165221156}, {-0.40146079659461975}, {2.9424406725117187}, {0.17261449992656708}, {0.2677871882915497}},
		{{2.387089252471924}, {-4.059374710122579}, {2.255565468464987}, {2.0867628294178466}, {-0.13651639619936162}, {-0.45258981386293584}},
		// {{2.383829116821289}, {0.6873907285877685}, {-2.3855693340301514}, {1.55326692640271}, {-0.6222108046161098}, {-0.07863790193666631}},
		// {{2.3837745189666752}, {1.1106418806263427}, {-2.4006066322326656}, {1.2128736215778808}, {-0.6221674124347132}, {-0.07856542268861944}},
		// {{2.787996292114258}, {0.9157830911823729}, {-2.39770245552063}, {1.5495359140583493}, {-0.3036978880511683}, {0.05714798346161842}},
		// {{2.8682186603546143}, {1.012371464366577}, {-2.4522807598114014}, {1.5496253210255126}, {-0.18606788316835576}, {-0.03227883974184209}},
		// {{4.093472480773926}, {0.2988439041325072}, {-0.4326653480529785}, {0.15900163232769768}, {1.1006063222885132}, {-0.03229362169374639}},
	}
	s.poses = []spatialmath.Pose{}
	for _, jointPos := range s.positions {
		newPose, err := s.arm.ModelFrame().Transform(jointPos)
		if err != nil {
			return err
		}
		s.poses = append(s.poses, newPose)
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
	if _, ok := cmd[calibrateKey]; ok {
		pose, err := s.calibrate(ctx)
		if err != nil {
			s.logger.Error(err)
			return nil, err
		}
		resp["guessed frame"] = pose
		s.guess = pose
	}
	if _, ok := cmd[vizKey]; ok {
		err := s.viz(ctx)
		if err != nil {
			return nil, fmt.Errorf("check that the viz server is running on your machine: %s", err.Error())
		}
		resp[vizKey] = vizAddress
		resp["note"] = "you may need to rerun the command if the page is empty"
	}
	if _, ok := cmd[moveArmKey]; ok {
		err := s.moveArm(ctx)
		if err != nil {
			return nil, err
		}
		resp[moveArmKey] = "success"
	}
	if len(resp) == 0 {
		return nil, errNoDo
	}
	return resp, nil
}

func (s *frameCalibrationArmCamera) Close(context.Context) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	// Put close code here
	s.cancelFunc()
	return nil
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
	if len(s.poses) == 0 {
		return nil, errNoPoses
	}
	// stop the arm at the end of calibration or an error occurring
	defer s.arm.Stop(ctx, nil)
	// move to initial pose.
	constraints := motionplan.NewEmptyConstraints()
	posInF := referenceframe.NewPoseInFrame(referenceframe.World, s.poses[0])
	req := motion.MoveReq{ComponentName: s.arm.Name(), Destination: posInF, WorldState: s.ws, Constraints: constraints}
	_, err := s.motion.Move(ctx, req)
	if err != nil {
		return nil, err
	}

	// discover tags for pose estimation
	tags, err := calutils.DiscoverTags(ctx, s.poseTracker)
	if err != nil {
		return nil, err
	}

	dataCfg := calutils.DataConfig{DataPath: s.cachedPlanDir, SaveNewData: false, LoadOldDataset: false}
	estimateReq := calutils.ReqFramePoseWithMotion{Arm: s.arm,
		Motion: s.motion, PoseTracker: s.poseTracker, ExpectedTags: tags, CalibrationPoses: s.poses, SeedPose: s.guess, WS: s.ws}

	return calutils.EstimateFramePoseWithMotion(ctx, estimateReq, dataCfg, s.logger)
}

func (s *frameCalibrationArmCamera) moveArm(ctx context.Context) error {
	if len(s.poses) == 0 {
		return errNoPoses
	}
	// stop the arm at the end of calibration or an error occurring
	defer s.arm.Stop(ctx, nil)
	constraints := motionplan.NewEmptyConstraints()
	for index, pos := range s.poses {
		s.logger.Debugf("moving to position %v, pose %v", index, pos)
		posInF := referenceframe.NewPoseInFrame(referenceframe.World, pos)

		req := motion.MoveReq{ComponentName: s.arm.Name(), Destination: posInF, WorldState: s.ws, Constraints: constraints}
		_, err := s.motion.Move(ctx, req)
		if err != nil {
			return err
		}
		// sleep to give time to check camera
		time.Sleep(1 * time.Second)
	}
	return nil

}
