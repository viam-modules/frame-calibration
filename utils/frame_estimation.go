package utils

import (
	"context"
	"encoding/gob"
	"errors"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"slices"
	"sync"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/posetracker"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/motionplan/ik"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils"

	commonpb "go.viam.com/api/common/v1"
)

const (
	randSeed               = 0
	orientationScaleFactor = 10.
	iterations             = 10000
	dof6                   = 6
)

var limits = []referenceframe.Limit{
	// R3 Vector
	{Min: -500, Max: 500}, // X(mm)
	{Min: -500, Max: 500}, // Y(mm)
	{Min: -500, Max: 500}, // Z(mm)
	// R3AA
	{Min: -2 * math.Pi, Max: 2 * math.Pi}, // X
	{Min: -2 * math.Pi, Max: 2 * math.Pi}, // Y
	{Min: -2 * math.Pi, Max: 2 * math.Pi}, // Z
}

func DiscoverTags(ctx context.Context, poseTracker posetracker.PoseTracker) ([]string, error) {
	poses, err := poseTracker.Poses(ctx, nil, nil)
	if err != nil {
		return nil, err
	}
	tags := []string{}
	for key, _ := range poses {
		tags = append(tags, key)
	}
	return tags, nil
}

// EstimateFramePose estimates the frame of a camera with respect to an arm using an arm's MoveToJointPositions.
// this method can be used if there are no obstacles that may cause a collision with the arm.
func EstimateFramePose(
	ctx context.Context,
	req ReqFramePoseWithoutMotion,
	dataCfg DataConfig,
	logger logging.Logger,
) (spatialmath.Pose, error) {
	var tagPoses []referenceframe.FrameSystemPoses
	calibrationPositions := req.CalibrationJointPositions
	var err error

	if !dataCfg.LoadOldDataset {
		if tagPoses, _, err = getTagPoses(ctx, req.Arm, req.PoseTracker, calibrationPositions); err != nil {
			return nil, err
		}

		if err := dataCfg.saveDataToFile(tagPoses, calibrationPositions); err != nil {
			return nil, err
		}
	} else {
		if tagPoses, calibrationPositions, err = dataCfg.loadDataFromFiles(); err != nil {
			return nil, err
		}
	}

	printWorldStatePoses(req.Arm, tagPoses, req.SeedPose, req.ExpectedTags, calibrationPositions, logger)

	sol, err := minimize(ctx, req.Arm.ModelFrame(), tagPoses, req.ExpectedTags, calibrationPositions, req.SeedPose, logger)
	if err != nil {
		return nil, err
	}
	logger.Debug("Guess:", req.SeedPose.Point(), req.SeedPose.Orientation().Quaternion())
	if len(sol[0].q) < dof6 {
		return nil, errors.New("invalid pose for solution")
	}
	p := floatsToPose(sol[0].q)
	logger.Info(p.Point(), p.Orientation().Quaternion(), sol[0].cost)
	printWorldStatePoses(req.Arm, tagPoses, req.SeedPose, req.ExpectedTags, calibrationPositions, logger)
	return p, nil
}

func EstimateFramePoseWithMotion(
	ctx context.Context,
	req ReqFramePoseWithMotion,
	dataCfg DataConfig,
	logger logging.Logger,
) (spatialmath.Pose, error) {
	var tagPoses []referenceframe.FrameSystemPoses
	var jointPositions [][]referenceframe.Input
	var err error
	// the user wants to collect a new set of data from the arm
	if !dataCfg.LoadOldDataset {
		if tagPoses, jointPositions, err = getTagPosesWithMotion(ctx, req, logger); err != nil {
			return nil, err
		}
		if err := dataCfg.saveDataToFile(tagPoses, jointPositions); err != nil {
			return nil, err
		}
	} else {
		if tagPoses, jointPositions, err = dataCfg.loadDataFromFiles(); err != nil {
			return nil, err
		}
	}

	printWorldStatePoses(req.Arm, tagPoses, req.SeedPose, req.ExpectedTags, jointPositions, logger)

	sol, err := minimize(ctx, req.Arm.ModelFrame(), tagPoses, req.ExpectedTags, jointPositions, req.SeedPose, logger)
	if err != nil {
		return nil, err
	}
	logger.Debug("Initial Guess: ", req.SeedPose.Point(), req.SeedPose.Orientation().Quaternion())
	if len(sol[0].q) < dof6 {
		return nil, fmt.Errorf("invalid pose for solution: %v", sol[0].q)
	}
	p := floatsToPose(sol[0].q)
	logger.Info("Optimization Guess: ", p.Point(), p.Orientation().Quaternion(), sol[0].cost)
	printWorldStatePoses(req.Arm, tagPoses, p, req.ExpectedTags, jointPositions, logger)
	return p, nil
}

// ReqFramePoseWithMotion is the request to determine a camera's frame position using a motion service on a viam-server.
// this method should be used if there are obstacles in the workspace that you wish to avoid.
type ReqFramePoseWithMotion struct {
	Arm              arm.Arm
	PoseTracker      posetracker.PoseTracker
	ExpectedTags     []string
	CalibrationPoses []spatialmath.Pose
	SeedPose         spatialmath.Pose
	Motion           motion.Service
	WS               *referenceframe.WorldState
}

// ReqFramePoseWithMotion is the request to estimate the frame of a camera with respect to an arm using an arm's MoveToJointPositions.
// this method can be used if there are no obstacles that may cause a collision with the arm.
type ReqFramePoseWithoutMotion struct {
	Arm                       arm.Arm
	PoseTracker               posetracker.PoseTracker
	ExpectedTags              []string
	CalibrationJointPositions [][]referenceframe.Input
	SeedPose                  spatialmath.Pose
}

type DataConfig struct {
	LoadOldDataset bool
	SaveNewData    bool
	DataPath       string
}

// SaveDataToFile saves data to files if SaveNewData is true
func (cfg DataConfig) saveDataToFile(tagPoses []referenceframe.FrameSystemPoses, jointPositions [][]referenceframe.Input) error {
	if !cfg.SaveNewData {
		return nil
	}
	tagPosesFile := filepath.Clean(filepath.Join(cfg.DataPath, "poses.gob"))
	armPositionsFile := filepath.Clean(filepath.Join(cfg.DataPath, "configurations.gob"))
	if err := saveToFile(tagPosesFile, posesToProtobuf(tagPoses)); err != nil {
		return err
	}
	if err := saveToFile(armPositionsFile, jointPositions); err != nil {
		return err
	}
	return nil
}

func (cfg DataConfig) loadDataFromFiles() ([]referenceframe.FrameSystemPoses, [][]referenceframe.Input, error) {
	var tagPosesFromFile []map[string]*commonpb.Pose
	var jointPositionsFromFile [][]referenceframe.Input
	tagPosesFile := filepath.Clean(filepath.Join(cfg.DataPath, "poses.gob"))
	armPositionsFile := filepath.Clean(filepath.Join(cfg.DataPath, "configurations.gob"))
	if err := loadFromFile(tagPosesFile, &tagPosesFromFile); err != nil {
		return nil, nil, err
	}
	tagPoses := protobufToPoses(tagPosesFromFile)
	if err := loadFromFile(armPositionsFile, &jointPositionsFromFile); err != nil {
		return nil, nil, err
	}
	return tagPoses, jointPositionsFromFile, nil
}

type basicNode struct {
	q    []float64
	cost float64
}

func minimize(
	ctx context.Context,
	model referenceframe.Model,
	poses []referenceframe.FrameSystemPoses,
	tags []string,
	calibrationPositions [][]referenceframe.Input,
	seedPose spatialmath.Pose,
	logger logging.Logger,
) ([]basicNode, error) {
	lossFunction := func(input []float64) float64 {
		if len(input) < dof6 {
			logger.Error("invalid pose for input: ", input)
			panic(fmt.Errorf("invalid pose for input: %v", input))
		}
		testPose := floatsToPose(input)

		// make a map of tags to the pose they are measured at in world coordinates
		tagPoseMap := make(map[string][]spatialmath.Pose)
		for i, pose := range poses {
			armPose, err := model.Transform(calibrationPositions[i])
			if err != nil {
				panic(err)
			}
			for _, tag := range tags {
				p, ok := pose[tag]
				if !ok {
					continue
				}
				tagPoseMap[tag] = append(tagPoseMap[tag], spatialmath.Compose(spatialmath.Compose(armPose, testPose), p.Pose()))
			}
		}

		// sum distances over all correspondences
		cumSum := 0.
		for _, tag := range tags {
			tagPoses := tagPoseMap[tag]
			n := len(tagPoses)
			count := (n - 1) * n / 2 // formula to calculate the ith triangle number
			for i := range n - 1 {
				for j := i + 1; j < n; j++ {
					delta := spatialmath.PoseDelta(tagPoses[i], tagPoses[j])
					tagDist := delta.Point().Norm2()
					tagDist += spatialmath.QuatToR3AA(delta.Orientation().Quaternion()).Mul(orientationScaleFactor).Norm2()
					cumSum += tagDist / float64(count)
				}
			}
		}
		// logger.Info(cumSum, input)
		return cumSum / float64(len(tags))
	}

	solver, err := ik.CreateNloptSolver(limits, logger, iterations, false, false)
	if err != nil {
		return nil, err
	}

	solutionGen := make(chan *ik.Solution, 1)
	ikErr := make(chan error, 1)

	var activeSolvers sync.WaitGroup
	defer activeSolvers.Wait()
	activeSolvers.Add(1)

	ctxWithCancel, cancel := context.WithCancel(ctx)
	defer cancel()

	// Spawn the IK solver to generate solutions until done
	utils.PanicCapturingGo(func() {
		defer close(ikErr)
		defer activeSolvers.Done()
		ikErr <- solver.Solve(ctxWithCancel, solutionGen, poseToFloats(seedPose), lossFunction, randSeed)
	})

	solutions := map[float64][]float64{}

	// Solve the IK solver. Loop labels are required because `break` etc in a `select` will break only the `select`.
IK:
	for {
		select {
		case <-ctx.Done():
			return nil, ctx.Err()
		default:
		}

		select {
		case solution := <-solutionGen:
			solutions[solution.Score] = solution.Configuration
			continue IK
		default:
		}

		select {
		case <-ikErr:
			// If we have a return from the IK solver, there are no more solutions, so we finish processing above
			// until we've drained the channel, handled by the `continue` above
			break IK
		default:
		}
	}

	// Cancel any ongoing processing within the IK solvers if we're done receiving solutions
	cancel()
	for done := false; !done; {
		select {
		case <-solutionGen:
		default:
			done = true
		}
	}

	if len(solutions) == 0 {
		return nil, errors.New("no solutions found")
	}

	keys := make([]float64, 0, len(solutions))
	for k := range solutions {
		keys = append(keys, k)
	}
	slices.Sort(keys)

	orderedSolutions := make([]basicNode, 0)
	for _, key := range keys {
		solution := solutions[key]
		// if a solution is empty, set it to a high cost
		if len(solution) == 0 {
			key = math.MaxFloat64
		}
		orderedSolutions = append(orderedSolutions, basicNode{q: solution, cost: key})
	}
	return orderedSolutions, nil
}

func getTagPoses(
	ctx context.Context,
	a arm.Arm,
	pt posetracker.PoseTracker,
	positions [][]referenceframe.Input,
) ([]referenceframe.FrameSystemPoses, [][]referenceframe.Input, error) {
	allPoses := make([]referenceframe.FrameSystemPoses, 0, len(positions))
	actualPositions := make([][]referenceframe.Input, 0)
	for _, position := range positions {
		err := a.MoveToJointPositions(ctx, position, nil)
		if err != nil {
			return nil, nil, err
		}
		time.Sleep(time.Second)
		j, err := averageJointPosition(ctx, a, 100)
		if err != nil {
			return nil, nil, err
		}
		actualPositions = append(actualPositions, j)

		poses, err := pt.Poses(ctx, nil, nil)
		if err != nil {
			return nil, nil, err
		}
		allPoses = append(allPoses, poses)
	}
	return allPoses, actualPositions, nil
}

func getTagPosesWithMotion(
	ctx context.Context,
	req ReqFramePoseWithMotion, logger logging.Logger,
) ([]referenceframe.FrameSystemPoses, [][]referenceframe.Input, error) {
	allPoses := make([]referenceframe.FrameSystemPoses, 0, len(req.CalibrationPoses))
	actualPositions := make([][]referenceframe.Input, 0, len(req.CalibrationPoses))
	constraints := motionplan.NewEmptyConstraints()

	for index, pos := range req.CalibrationPoses {
		logger.Debugf("moving to position %v, pose %v", index, pos)
		posInF := referenceframe.NewPoseInFrame(referenceframe.World, pos)
		motionReq := motion.MoveReq{ComponentName: req.Arm.Name(), Destination: posInF, WorldState: req.WS, Constraints: constraints}
		_, err := req.Motion.Move(ctx, motionReq)
		if err != nil {
			return nil, nil, err
		}

		time.Sleep(time.Second)
		j, err := averageJointPosition(ctx, req.Arm, 100)
		if err != nil {
			return nil, nil, err
		}
		actualPositions = append(actualPositions, j)

		poses, err := req.PoseTracker.Poses(ctx, nil, nil)
		if err != nil {
			return nil, nil, err
		}
		allPoses = append(allPoses, poses)
	}
	return allPoses, actualPositions, nil
}

func averageJointPosition(ctx context.Context, a arm.Arm, n int) ([]referenceframe.Input, error) {
	avg := make([]referenceframe.Input, len(a.ModelFrame().DoF()))
	for range n {
		j, err := a.JointPositions(ctx, nil)
		if err != nil {
			return nil, err
		}
		for i := range len(avg) {
			avg[i].Value += j[i].Value
		}
	}
	for i := range len(avg) {
		avg[i].Value /= float64(n)
	}
	return avg, nil
}

func printWorldStatePoses(
	a arm.Arm,
	poses []referenceframe.FrameSystemPoses,
	unknownPose spatialmath.Pose,
	tags []string,
	calibrationPositions [][]referenceframe.Input,
	logger logging.Logger,
) error {
	for _, tag := range tags {
		logger.Debugf("Tag #%s\n", tag)
		for i, pose := range poses {
			tagPose, ok := pose[tag]
			if !ok {
				continue
			}
			logger.Debugf("\tPose #%d\t", i)
			armPose, err := a.ModelFrame().Transform(calibrationPositions[i])
			if err != nil {
				return err
			}
			worldPose := spatialmath.Compose(spatialmath.Compose(armPose, unknownPose), tagPose.Pose())
			logger.Debugf("%.3v\n", worldPose)
		}
	}
	return nil
}

func saveToFile(filename string, data any) error {
	file, err := os.Create(filename)
	if err != nil {
		return err
	}
	defer file.Close()

	encoder := gob.NewEncoder(file)
	err = encoder.Encode(data)
	if err != nil {
		return err
	}
	return nil
}

func loadFromFile[T any](filename string, data *T) error {
	file, err := os.Open(filename)
	if err != nil {
		return err
	}
	defer file.Close()

	decoder := gob.NewDecoder(file)
	err = decoder.Decode(&data)
	if err != nil {
		return err
	}
	return nil
}

func posesToProtobuf(poses []referenceframe.FrameSystemPoses) []map[string]commonpb.Pose {
	pb := make([]map[string]commonpb.Pose, 0)
	for _, fsPose := range poses {
		m := make(map[string]commonpb.Pose)
		for key, value := range fsPose {
			m[key] = *spatialmath.PoseToProtobuf(value.Pose())
		}
		pb = append(pb, m)
	}
	return pb
}

func protobufToPoses(pb []map[string]*commonpb.Pose) []referenceframe.FrameSystemPoses {
	poses := make([]referenceframe.FrameSystemPoses, 0)
	for _, p := range pb {
		m := make(referenceframe.FrameSystemPoses)
		for key, value := range p {
			m[key] = referenceframe.NewPoseInFrame("cam", spatialmath.NewPoseFromProtobuf(value))
		}
		poses = append(poses, m)
	}
	return poses
}

func poseToFloats(p spatialmath.Pose) []float64 {
	pt := p.Point()
	o := p.Orientation().AxisAngles().ToR3()
	return []float64{pt.X, pt.Y, pt.Z, o.X, o.Y, o.Z}
}

func floatsToPose(f []float64) spatialmath.Pose {
	return spatialmath.NewPose(r3.Vector{X: f[0], Y: f[1], Z: f[2]}, spatialmath.R3ToR4(r3.Vector{f[3], f[4], f[5]}))
}
