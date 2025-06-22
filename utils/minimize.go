package utils

import (
	"context"
	"errors"
	"maps"
	"math"
	"slices"
	"sync"

	"github.com/golang/geo/r3"

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

type BasicNode struct {
	q    []float64
	Cost float64
}

func (bn *BasicNode) Pose() spatialmath.Pose {
	return floatsToPose(bn.q)
}

func floatsToPose(f []float64) spatialmath.Pose {
	return spatialmath.NewPose(r3.Vector{X: f[0], Y: f[1], Z: f[2]}, spatialmath.R3ToR4(r3.Vector{f[3], f[4], f[5]}))
}

func poseToFloats(p spatialmath.Pose) []float64 {
	pt := p.Point()
	o := p.Orientation().AxisAngles().ToR3()
	return []float64{pt.X, pt.Y, pt.Z, o.X, o.Y, o.Z}
}

func Minimize(
	ctx context.Context,
	data []ArmAndPoses,
	seedPose spatialmath.Pose,
	logger logging.Logger,
) ([]BasicNode, error) {

	tags := maps.Keys(data[0].Tags)

	lossFunction := func(input []float64) float64 {
		testPose := floatsToPose(input)

		// make a map of tags to the pose they are measured at in world coordinates
		tagPoseMap := make(map[string][]spatialmath.Pose)
		for _, d := range data {
			for tag := range tags {
				p, ok := d.Tags[tag]
				if !ok {
					continue
				}
				tagPoseMap[tag] = append(tagPoseMap[tag], spatialmath.Compose(spatialmath.Compose(&d.Pose, testPose), &p))
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
		return cumSum / float64(len(data[0].Tags))
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

	orderedSolutions := make([]BasicNode, 0)
	for _, key := range keys {
		solution := solutions[key]
		if len(solution) == 0 {
			continue
		}
		orderedSolutions = append(orderedSolutions, BasicNode{q: solution, Cost: key})
	}
	return orderedSolutions, nil
}
