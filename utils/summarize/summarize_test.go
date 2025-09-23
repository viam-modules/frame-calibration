package utils

import (
	"encoding/json"
	"fmt"
	"math"
	"os"
	"sort"
	"strconv"
	"testing"

	"github.com/golang/geo/r3"
	vizClient "github.com/viam-labs/motion-tools/client/client"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

type config struct {
	CalibrationDataPath string  `json:"calibration_data_path"`
	ArmKinematicsPath   string  `json:"arm_kinematics_path"`
	ArmFrameOX          float64 `json:"arm_frame_ox"`
	ArmFrameOY          float64 `json:"arm_frame_oy"`
	ArmFrameOZ          float64 `json:"arm_frame_oz"`
	ArmFrameTheta       float64 `json:"arm_frame_theta"`
}

const (
	maxTags                     = 30 // apriltag board has 30 april tags on it
	numAprilTagImagingPositions = 11 // going to how many unique imaging locations
	changeInZ                   = 43 // mm
	changeInY                   = 42 // mm

	aprilTag16x = -125   // mm in world frame
	aprilTag16y = -1e-12 // mm in world frame
	aprilTag16z = -89.5  // mm in world frame
)

var (
	colors = [][3]uint8{
		{255, 165, 0}, // orange
		{255, 255, 0}, // yellow
		{0, 255, 0},   // green
		{0, 0, 255},   // blue
		{86, 5, 145},  // violet
		{128, 0, 255},
		{0, 255, 255},
		{150, 75, 0}, // brown
	}
)

var (
	idealAprilTagLocations = map[string]r3.Vector{
		"16": {aprilTag16x, aprilTag16y, aprilTag16z},
		"17": {aprilTag16x, aprilTag16y, aprilTag16z - changeInZ},
		"18": {aprilTag16x, aprilTag16y, aprilTag16z - changeInZ*2},
		"19": {aprilTag16x, aprilTag16y, aprilTag16z - changeInZ*3},
		"20": {aprilTag16x, aprilTag16y, aprilTag16z - changeInZ*4},

		"11": {aprilTag16x, aprilTag16y - changeInY, aprilTag16z},
		"12": {aprilTag16x, aprilTag16y - changeInY, aprilTag16z - changeInZ},
		"13": {aprilTag16x, aprilTag16y - changeInY, aprilTag16z - changeInZ*2},
		"14": {aprilTag16x, aprilTag16y - changeInY, aprilTag16z - changeInZ*3},
		"15": {aprilTag16x, aprilTag16y - changeInY, aprilTag16z - changeInZ*4},

		"6":  {aprilTag16x, aprilTag16y - changeInY*2, aprilTag16z},
		"7":  {aprilTag16x, aprilTag16y - changeInY*2, aprilTag16z - changeInZ},
		"8":  {aprilTag16x, aprilTag16y - changeInY*2, aprilTag16z - changeInZ*2},
		"9":  {aprilTag16x, aprilTag16y - changeInY*2, aprilTag16z - changeInZ*3},
		"10": {aprilTag16x, aprilTag16y - changeInY*2, aprilTag16z - changeInZ*4},

		"1": {aprilTag16x, aprilTag16y - changeInY*3, aprilTag16z},
		"2": {aprilTag16x, aprilTag16y - changeInY*3, aprilTag16z - changeInZ},
		"3": {aprilTag16x, aprilTag16y - changeInY*3, aprilTag16z - changeInZ*2},
		"4": {aprilTag16x, aprilTag16y - changeInY*3, aprilTag16z - changeInZ*3},
		"5": {aprilTag16x, aprilTag16y - changeInY*3, aprilTag16z - changeInZ*4},

		"21": {aprilTag16x, aprilTag16y + changeInY, aprilTag16z},
		"22": {aprilTag16x, aprilTag16y + changeInY, aprilTag16z - changeInZ},
		"23": {aprilTag16x, aprilTag16y + changeInY, aprilTag16z - changeInZ*2},
		"24": {aprilTag16x, aprilTag16y + changeInY, aprilTag16z - changeInZ*3},
		"25": {aprilTag16x, aprilTag16y + changeInY, aprilTag16z - changeInZ*4},

		"26": {aprilTag16x, aprilTag16y + changeInY*2, aprilTag16z},
		"27": {aprilTag16x, aprilTag16y + changeInY*2, aprilTag16z - changeInZ},
		"28": {aprilTag16x, aprilTag16y + changeInY*2, aprilTag16z - changeInZ*2},
		"29": {aprilTag16x, aprilTag16y + changeInY*2, aprilTag16z - changeInZ*3},
		"30": {aprilTag16x, aprilTag16y + changeInY*2, aprilTag16z - changeInZ*4},
	}
)

type Vec3 struct {
	X float64 `json:"X"`
	Y float64 `json:"Y"`
	Z float64 `json:"Z"`
}

type Orientation struct {
	Th float64 `json:"th"`
	X  float64 `json:"x"`
	Y  float64 `json:"y"`
	Z  float64 `json:"z"`
}

type Pose struct {
	P Vec3        `json:"P"`
	O Orientation `json:"O"`
}

type Joint struct {
	Value float64 `json:"Value"`
}

type TagMap map[string]Pose

type FrameEntry struct {
	Joints []Joint `json:"Joints"`
	Pose   Pose    `json:"Pose"`
	Tags   TagMap  `json:"Tags"`
}

type FrameCalibrationFile struct {
	Data   []FrameEntry `json:"Data"`
	Result Pose         `json:"Result"`
}

type FrameSet []FrameEntry

func (fe FrameEntry) jointsFloat64() []float64 {
	out := make([]float64, len(fe.Joints))
	for i, j := range fe.Joints {
		out[i] = j.Value
	}
	return out
}

func (fe FrameEntry) tag(id int) (Pose, bool) {
	key := strconv.Itoa(id)
	p, ok := fe.Tags[key]
	return p, ok
}

func loadFrameSet(path string) (FrameSet, *FrameCalibrationFile, error) {
	f, err := os.Open(path)
	if err != nil {
		return nil, nil, fmt.Errorf("open: %w", err)
	}
	defer f.Close()

	var fc FrameCalibrationFile
	if err := json.NewDecoder(f).Decode(&fc); err != nil {
		return nil, nil, fmt.Errorf("decode: %w", err)
	}

	// Take indices 0..7 if available
	n := min(len(fc.Data), numAprilTagImagingPositions)
	fs := FrameSet(fc.Data[:n])

	return fs, &fc, nil
}

func maxDiff(in []float64) float64 {
	if len(in) == 0 {
		return math.NaN()
	}
	minv, maxv := in[0], in[0]
	for _, v := range in[1:] {
		if v < minv {
			minv = v
		}
		if v > maxv {
			maxv = v
		}
	}
	return maxv - minv
}

func median(in []float64) float64 {
	n := len(in)
	if n == 0 {
		return math.NaN()
	}
	cp := make([]float64, n)
	copy(cp, in)
	sort.Float64s(cp)
	if n%2 == 1 {
		return cp[n/2]
	}
	return (cp[n/2-1] + cp[n/2]) / 2
}

func mean(in []float64) float64 {
	if len(in) == 0 {
		return math.NaN()
	}
	sum := 0.0
	for _, v := range in {
		sum += v
	}
	return sum / float64(len(in))
}

func stdDevSample(in []float64) float64 {
	n := len(in)
	if n <= 1 {
		return 0
	}
	m := mean(in)
	var ssq float64
	for _, v := range in {
		d := v - m
		ssq += d * d
	}
	return math.Sqrt(ssq / float64(n-1))
}

func loadConfig(path string) (config, error) {
	var cfg config
	file, err := os.Open(path)
	if err != nil {
		return config{}, err
	}
	defer file.Close()
	decoder := json.NewDecoder(file)
	if err := decoder.Decode(&cfg); err != nil {
		return config{}, err
	}
	return cfg, nil
}

func TestDetectionsAgainstReality(t *testing.T) {
	logger := logging.NewTestLogger(t)

	cfg, err := loadConfig("config.json")
	test.That(t, err, test.ShouldBeNil)

	err = vizClient.RemoveAllSpatialObjects()
	test.That(t, err, test.ShouldBeNil)

	m, err := referenceframe.ParseModelJSONFile(cfg.ArmKinematicsPath, "")
	test.That(t, err, test.ShouldBeNil)

	fs, fcFile, err := loadFrameSet(cfg.CalibrationDataPath)
	test.That(t, err, test.ShouldBeNil)

	cameraPoseOnArm := spatialmath.NewPose(
		r3.Vector{
			X: fcFile.Result.P.X,
			Y: fcFile.Result.P.Y,
			Z: fcFile.Result.P.Z,
		},
		&spatialmath.OrientationVectorDegrees{
			OX:    fcFile.Result.O.X,
			OY:    fcFile.Result.O.Y,
			OZ:    fcFile.Result.O.Z,
			Theta: fcFile.Result.O.Th,
		},
	)

	allXs := make([][]float64, maxTags)
	for i := range allXs {
		allXs[i] = make([]float64, numAprilTagImagingPositions)
	}
	allYs := make([][]float64, maxTags)
	for i := range allYs {
		allYs[i] = make([]float64, numAprilTagImagingPositions)
	}
	allZs := make([][]float64, maxTags)
	for i := range allZs {
		allZs[i] = make([]float64, numAprilTagImagingPositions)
	}

	rotatePose := spatialmath.NewPoseFromOrientation(&spatialmath.OrientationVectorDegrees{
		OX:    cfg.ArmFrameOX,
		OY:    cfg.ArmFrameOY,
		OZ:    cfg.ArmFrameOY,
		Theta: cfg.ArmFrameTheta,
	})

	for i := range fs {
		e := fs[i]
		js := e.jointsFloat64()
		armPose, err := m.Transform(referenceframe.FloatsToInputs(js))
		test.That(t, err, test.ShouldBeNil)
		rotatedArmPose := spatialmath.Compose(rotatePose, armPose)

		for tagID := 1; tagID <= maxTags; tagID++ {
			if t, ok := e.tag(tagID); ok {
				aprilTagPose := spatialmath.NewPose(
					r3.Vector(t.P),
					&spatialmath.OrientationVectorDegrees{
						OX:    t.O.X,
						OY:    t.O.Y,
						OZ:    t.O.Z,
						Theta: t.O.Th,
					},
				)

				camPoseInWrld := spatialmath.Compose(rotatedArmPose, cameraPoseOnArm)
				worldPose := spatialmath.Compose(camPoseInWrld, aprilTagPose)

				allXs[tagID-1][i] = worldPose.Point().X
				allYs[tagID-1][i] = worldPose.Point().Y
				allZs[tagID-1][i] = worldPose.Point().Z
			}
		}
	}

	for i, xs := range allXs {
		ys := allYs[i]
		zs := allZs[i]
		for j := 0; j < 8; j++ {
			individualPose := spatialmath.NewPoseFromPoint(r3.Vector{xs[j], ys[j], zs[j]})
			err := vizClient.DrawPoints(
				"individual pose #"+strconv.Itoa(i)+" from this imaging position: "+strconv.Itoa(j),
				[]spatialmath.Pose{individualPose},
				nil, &colors[j%8],
			)
			test.That(t, err, test.ShouldBeNil)
		}

		mDiffX := maxDiff(xs)
		medianX := median(xs)
		avgX := mean(xs)
		sdX := stdDevSample(xs)

		mDiffY := maxDiff(ys)
		medianY := median(ys)
		avgY := mean(ys)
		sdY := stdDevSample(ys)

		mDiffZ := maxDiff(zs)
		medianZ := median(zs)
		avgZ := mean(zs)
		sdZ := stdDevSample(zs)

		avgTagPose := spatialmath.NewPoseFromPoint(r3.Vector{avgX, avgY, avgZ})
		label := "frame calibration april tag #" + strconv.Itoa(i+1)
		err := vizClient.DrawPoints(
			label,
			[]spatialmath.Pose{avgTagPose},
			nil, &[3]uint8{255, 0, 0},
		)
		test.That(t, err, test.ShouldBeNil)

		ideal, ok := idealAprilTagLocations[strconv.Itoa(i+1)]
		test.That(t, ok, test.ShouldBeTrue)

		idealTagPose := spatialmath.NewPoseFromPoint(ideal)
		label = "frame calibration IDEAL april tag #" + strconv.Itoa(i+1)
		err = vizClient.DrawPoints(
			label,
			[]spatialmath.Pose{idealTagPose},
			nil, &[3]uint8{0, 0, 0},
		)
		test.That(t, err, test.ShouldBeNil)

		logger.Info("information regarding april tag number: ", i+1)
		logger.Info("all data for X: ", xs)
		logger.Info("all data for Y: ", ys)
		logger.Info("all data for Z: ", zs)
		logger.Info("difference in ideal position versus average position: ", avgTagPose.Point().Sub(idealTagPose.Point()))
		logger.Info("euclidean distance difference in ideal position versus average position: ", avgTagPose.Point().Sub(idealTagPose.Point()).Norm())
		logger.Info("max difference in X axis: ", mDiffX)
		logger.Info("max difference in Y axis: ", mDiffY)
		logger.Info("max difference in Z axis: ", mDiffZ)
		logger.Info("median value in X axis: ", medianX)
		logger.Info("median in Y axis: ", medianY)
		logger.Info("median in Z axis: ", medianZ)
		logger.Info("average value in X axis: ", avgX)
		logger.Info("average value in Y axis: ", avgY)
		logger.Info("average value in Z axis: ", avgZ)
		logger.Info("standard deviation in X axis: ", sdX)
		logger.Info("standard deviation in Y axis: ", sdY)
		logger.Info("standard deviation in Z axis: ", sdZ)
	}
}
