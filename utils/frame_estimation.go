package utils

import (
	"context"
	"fmt"
	"time"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/posetracker"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

type SavedArmPositionGoer interface {
	NumPositions() int
	MoveToSavedPosition(ctx context.Context, pos int) ([]referenceframe.Input, spatialmath.Pose, error)
}

// ReqFramePose is the request to determine a camera's frame position using a motion service on a viam-server.
type ReqFramePose struct {
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

	data, err := getTagPoses(ctx, req.PoseTracker, req.Mover)
	if err != nil {
		return nil, 0, err
	}

	sol, err := Minimize(ctx, data, req.SeedPose, logger)
	if err != nil {
		return nil, 0, err
	}
	logger.Debug("Guess:", req.SeedPose.Point(), req.SeedPose.Orientation().Quaternion())

	p := sol[0].Pose()
	logger.Info("Optimization Guess: ", p.Point(), p.Orientation().Quaternion(), sol[0].Cost)

	err = WriteData(data, p, logger)
	if err != nil {
		logger.Warnf("couldn't write data, but continuing: %v", err)
	}

	return p, sol[0].Cost, nil
}

func NewSimplePose(p spatialmath.Pose) SimplePose {
	return SimplePose{p.Point(), p.Orientation().OrientationVectorDegrees()}
}

type SimplePose struct {
	P r3.Vector
	O *spatialmath.OrientationVectorDegrees
}

func (sp *SimplePose) Point() r3.Vector {
	return sp.P
}

func (sp *SimplePose) Orientation() spatialmath.Orientation {
	return sp.O
}

type ArmAndPoses struct {
	Joints []referenceframe.Input
	Pose   SimplePose
	Tags   map[string]SimplePose
}

func getTagPoses(
	ctx context.Context,
	pt posetracker.PoseTracker,
	mover SavedArmPositionGoer,
) ([]ArmAndPoses, error) {

	data := []ArmAndPoses{}

	for idx := range mover.NumPositions() {
		joints, pose, err := mover.MoveToSavedPosition(ctx, idx)
		if err != nil {
			return nil, err
		}

		time.Sleep(time.Second) // TODO: wait for camera to settle....

		poses, err := GetPoses(ctx, pt)
		if err != nil {
			return nil, err
		}

		if idx > 0 && len(poses) < len(data[0].Tags)-4 {
			return nil, fmt.Errorf("poses %d and %d have different number of tags %d vs %d", 0, idx, len(data[0].Tags), len(poses))
		}

		data = append(data, ArmAndPoses{joints, NewSimplePose(pose), poses})
	}
	return data, nil
}

func GetPoses(ctx context.Context, pt posetracker.PoseTracker) (map[string]SimplePose, error) {
	raw, err := pt.Poses(ctx, nil, nil)
	if err != nil {
		return nil, err
	}
	m := map[string]SimplePose{}
	for k, v := range raw {
		m[k] = NewSimplePose(v.Pose())
	}
	return m, nil
}

// TODO: why?????
func AverageJointPosition(ctx context.Context, a arm.Arm, n int) ([]referenceframe.Input, error) {
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
