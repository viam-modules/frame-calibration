package utils

import (
	"encoding/json"
	"fmt"
	"os"
	"time"

	"github.com/erh/vmodutils"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/spatialmath"
)

type debugData struct {
	Data   []ArmAndPoses
	Result SimplePose
}

func getDataFileName() string {
	root := os.Getenv("HOME")
	md := os.Getenv("VIAM_MODULE_DATA")
	if md != "" {
		root = md
	}
	t := time.Now()
	return fmt.Sprintf("%s/frame-calibration-data-%s.json",
		root,
		t.Format("2006-01-02-15-04-05"),
	)
}

func WriteData(data []ArmAndPoses, result spatialmath.Pose, logger logging.Logger) error {
	fn := getDataFileName()

	logger.Infof("writing data to %s", fn)

	dd := debugData{data, NewSimplePose(result)}

	file, err := os.Create(fn)
	if err != nil {
		return fmt.Errorf("couldn't create %s: %w", fn, err)
	}
	defer file.Close()

	encoder := json.NewEncoder(file)
	return encoder.Encode(&dd)
}

func ReadData(fn string) ([]ArmAndPoses, spatialmath.Pose, error) {
	dd := &debugData{}
	err := vmodutils.ReadJSONFromFile(fn, dd)
	if err != nil {
		return nil, nil, err
	}
	return dd.Data, &dd.Result, nil
}
