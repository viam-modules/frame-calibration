package framecalibration

import (
	"context"
	"errors"

	calutils "framecalibration/utils"

	armPb "go.viam.com/api/component/arm/v1"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/components/posetracker"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/spatialmath"
)

var (
	ArmCamera        = resource.NewModel("viam", "frame-calibration", "arm-camera")
	errUnimplemented = errors.New("unimplemented")
)

func init() {
	resource.RegisterService(generic.API, ArmCamera,
		resource.Registration[resource.Resource, *Config]{
			Constructor: newFrameCalibrationArmCamera,
		},
	)
}

type Config struct {
	Arm            string      `json:"arm"`
	Camera         string      `json:"camera"`
	PoseTracker    string      `json:"tracker"`
	JointPositions [][]float64 `json:"joint_positions"`
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
	if cfg.Camera == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "camera")
	}
	deps = append(deps, cfg.Camera)
	if cfg.PoseTracker == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "tracker")
	}
	deps = append(deps, cfg.PoseTracker)
	return deps, nil, nil
}

type frameCalibrationArmCamera struct {
	name resource.Name

	logger      logging.Logger
	cfg         *Config
	poseTracker posetracker.PoseTracker
	arm         arm.Arm
	cam         camera.Camera
	positions   [][]referenceframe.Input
	guess       spatialmath.Pose

	cancelCtx  context.Context
	cancelFunc func()
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
	if s.cfg.Arm != conf.Arm {
		s.arm, err = arm.FromDependencies(deps, conf.Arm)
		if err != nil {
			return err
		}
	}
	if s.cfg.Camera != conf.Camera {
		s.cam, err = camera.FromDependencies(deps, conf.Camera)
		if err != nil {
			return err
		}
	}
	if s.cfg.PoseTracker != conf.PoseTracker {
		s.poseTracker, err = posetracker.FromDependencies(deps, conf.PoseTracker)
		if err != nil {
			return err
		}
	}

	// always reconfigure positions
	s.positions = [][]referenceframe.Input{}
	for _, jointPos := range conf.JointPositions {
		pbPos := armPb.JointPositions{Values: jointPos}
		// This will break when kinematics update
		inputs := s.arm.ModelFrame().InputFromProtobuf(&pbPos)
		s.positions = append(s.positions, inputs)
	}

	s.cfg = conf

	return nil
}

func (s *frameCalibrationArmCamera) Name() resource.Name {
	return s.name
}

func (s *frameCalibrationArmCamera) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	resp := map[string]interface{}{}
	if _, ok := cmd["runCalibration"]; ok {
		pose, err := s.calibrate(ctx)
		if err != nil {
			return nil, err
		}
		resp["poseGuess"] = pose
		s.guess = pose
	}
	return resp, nil
}

func (s *frameCalibrationArmCamera) Close(context.Context) error {
	// Put close code here
	s.cancelFunc()
	return nil
}

func (s *frameCalibrationArmCamera) calibrate(ctx context.Context) (spatialmath.Pose, error) {
	pos := [][]referenceframe.Input{{{1.5687246322631836}, {-2.8689214191832484}, {0.8763092199908655}, {1.413437767619751}, {-0.9621284643756312}, {-0.009279553090230763}},
		{{1.5874707698822021}, {-3.372202535668844}, {0.8752110640155237}, {2.0592481332966304}, {-1.091687027608053}, {-0.009339634572164357}},
		{{1.599494218826294}, {-1.6834622822203578}, {-1.0425125360488892}, {2.058858795756958}, {-1.4727914969073692}, {0.3423517644405365}},
		{{1.0668467283248901}, {-2.0526958904662074}, {-1.5102773904800415}, {3.1842085558125}, {-1.9495976606952115}, {0.3422858119010926}},
		{{2.710968494415283}, {-1.8869949779906214}, {-1.8658866882324219}, {3.8552242952534175}, {0.2258821278810501}, {0.3423832952976227}},
		{{2.4088399410247803}, {-2.73096813778066}, {-0.4833677113056183}, {4.110974951381348}, {0.22581766545772552}, {0.3424190878868103}},
		{{2.2999038696289062}, {-2.7309800587096156}, {-0.4835145175457001}, {4.110954924220703}, {0.22584037482738495}, {0.34241911768913275}},
		{{2.338528156280518}, {-2.9684068165221156}, {-0.40146079659461975}, {2.9424406725117187}, {0.17261449992656708}, {0.2677871882915497}},
		{{2.387089252471924}, {-4.059374710122579}, {2.255565468464987}, {2.0867628294178466}, {-0.13651639619936162}, {-0.45258981386293584}},
		{{2.383829116821289}, {0.6873907285877685}, {-2.3855693340301514}, {1.55326692640271}, {-0.6222108046161098}, {-0.07863790193666631}},
		{{2.3837745189666752}, {1.1106418806263427}, {-2.4006066322326656}, {1.2128736215778808}, {-0.6221674124347132}, {-0.07856542268861944}},
		{{2.787996292114258}, {0.9157830911823729}, {-2.39770245552063}, {1.5495359140583493}, {-0.3036978880511683}, {0.05714798346161842}},
		{{2.8682186603546143}, {1.012371464366577}, {-2.4522807598114014}, {1.5496253210255126}, {-0.18606788316835576}, {-0.03227883974184209}},
		{{4.093472480773926}, {0.2988439041325072}, {-0.4326653480529785}, {0.15900163232769768}, {1.1006063222885132}, {-0.03229362169374639}},
	}

	// move to initial position
	err := s.arm.MoveToJointPositions(ctx, pos[0], nil)
	if err != nil {
		return nil, err
	}
	// discover tags
	tags, err := calutils.DiscoverTags(ctx, s.poseTracker)
	if err != nil {
		return nil, err
	}

	return calutils.EstimateFramePose(ctx, s.arm, s.poseTracker, tags, pos, s.guess, false)
}
