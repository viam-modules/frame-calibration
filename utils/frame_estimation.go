package utils

import (
	"context"
	"errors"
	"fmt"
	"maps"
	"math"
	"slices"
	"sync"
	"time"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/posetracker"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan/ik"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils"
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

type SavedArmPositionGoer interface {
	NumPositions() int
	MoveToSavedPosition(ctx context.Context, pos int) error
}

// ReqFramePose is the request to determine a camera's frame position using a motion service on a viam-server.
type ReqFramePose struct {
	Arm         arm.Arm
	PoseTracker posetracker.PoseTracker
	SeedPose    spatialmath.Pose
	Mover       SavedArmPositionGoer
}

// EstimateFramePose estimates the frame of a camera with respect to an arm using an arm's MoveToJointPositions.
// this method can be used if there are no obstacles that may cause a collision with the arm.
// returns the pose and the optimizer's minimum cost value for that pose.
func EstimateFramePose(
	ctx context.Context,
	req ReqFramePose,
	logger logging.Logger,
) (spatialmath.Pose, float64, error) {

	tagPoses, positions, err := getTagPoses(ctx, req.Arm, req.PoseTracker, req.Mover)
	if err != nil {
		return nil, 0, err
	}

	printWorldStatePoses(req.Arm, tagPoses, req.SeedPose, positions, logger)

	armModel, err := req.Arm.Kinematics(ctx)
	if err != nil {
		return nil, 0, err
	}

	sol, err := minimize(ctx, armModel, tagPoses, positions, req.SeedPose, logger)
	if err != nil {
		return nil, 0, err
	}
	logger.Debug("Guess:", req.SeedPose.Point(), req.SeedPose.Orientation().Quaternion())

	if len(sol[0].q) < dof6 {
		return nil, 0, errors.New("invalid pose for solution")
	}

	p := floatsToPose(sol[0].q)
	logger.Info("Optimization Guess: ", p.Point(), p.Orientation().Quaternion(), sol[0].cost)
	printWorldStatePoses(req.Arm, tagPoses, req.SeedPose, positions, logger)

	return p, sol[0].cost, nil
}

type basicNode struct {
	q    []float64
	cost float64
}

func minimize(
	ctx context.Context,
	model referenceframe.Model,
	poses []referenceframe.FrameSystemPoses,
	calibrationPositions [][]referenceframe.Input,
	seedPose spatialmath.Pose,
	logger logging.Logger,
) ([]basicNode, error) {

	tags := maps.Keys(poses[0])

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
			for tag := range tags {
				p, ok := pose[tag]
				if !ok {
					continue
				}
				tagPoseMap[tag] = append(tagPoseMap[tag], spatialmath.Compose(spatialmath.Compose(armPose, testPose), p.Pose()))
			}
		}

		// sum distances over all correspondences
		cumSum := 0.
		for tag := range tags {
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
		return cumSum / float64(len(poses[0]))
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
		if len(solution) == 0 {
			continue
		}

		orderedSolutions = append(orderedSolutions, basicNode{q: solution, cost: key})
	}
	return orderedSolutions, nil
}

func getTagPoses(
	ctx context.Context,
	a arm.Arm,
	pt posetracker.PoseTracker,
	mover SavedArmPositionGoer,
) ([]referenceframe.FrameSystemPoses, [][]referenceframe.Input, error) {

	allPoses := []referenceframe.FrameSystemPoses{}
	actualPositions := [][]referenceframe.Input{}

	for idx := range mover.NumPositions() {
		err := mover.MoveToSavedPosition(ctx, idx)
		if err != nil {
			return nil, nil, err
		}

		time.Sleep(time.Second) // TODO: why???

		j, err := averageJointPosition(ctx, a, 100) // TODO: why???
		if err != nil {
			return nil, nil, err
		}
		actualPositions = append(actualPositions, j)

		poses, err := pt.Poses(ctx, nil, nil)
		if err != nil {
			return nil, nil, err
		}

		if idx > 0 && len(poses) != len(allPoses[0]) {
			return nil, nil, fmt.Errorf("poses %d and %d have different number of tags %d vs %d", 0, idx, len(allPoses[0]), len(poses))
		}

		allPoses = append(allPoses, poses)
	}
	return allPoses, actualPositions, nil
}

// TODO: why?????
func averageJointPosition(ctx context.Context, a arm.Arm, n int) ([]referenceframe.Input, error) {
	armModel, err := a.Kinematics(ctx)
	if err != nil {
		return nil, err
	}
	avg := make([]referenceframe.Input, len(armModel.DoF()))
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
	calibrationPositions [][]referenceframe.Input,
	logger logging.Logger,
) error {
	armModel, err := a.Kinematics(context.Background())
	if err != nil {
		return err
	}
	for tag := range maps.Keys(poses[0]) {
		logger.Debugf("Tag #%s\n", tag)
		for i, pose := range poses {
			tagPose, ok := pose[tag]
			if !ok {
				continue
			}
			logger.Debugf("\tPose #%d\t", i)
			armPose, err := armModel.Transform(calibrationPositions[i])
			if err != nil {
				return err
			}
			worldPose := spatialmath.Compose(spatialmath.Compose(armPose, unknownPose), tagPose.Pose())
			logger.Debugf("%.3v\n", worldPose)
		}
	}
	return nil
}

func floatsToPose(f []float64) spatialmath.Pose {
	return spatialmath.NewPose(r3.Vector{X: f[0], Y: f[1], Z: f[2]}, spatialmath.R3ToR4(r3.Vector{f[3], f[4], f[5]}))
}

func poseToFloats(p spatialmath.Pose) []float64 {
	pt := p.Point()
	o := p.Orientation().AxisAngles().ToR3()
	return []float64{pt.X, pt.Y, pt.Z, o.X, o.Y, o.Z}
}
