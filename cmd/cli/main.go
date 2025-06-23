package main

import (
	"context"
	"flag"
	"fmt"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/services/generic"

	"github.com/erh/vmodutils"

	"framecalibration"
)

func main() {
	err := realMain()
	if err != nil {
		panic(err)
	}
}

func realMain() error {
	ctx := context.Background()
	logger := logging.NewLogger("cli")

	configFile := flag.String("config", "", "config file")
	host := flag.String("host", "", "host to connect to")

	flag.Parse()

	logger.Infof("using config file [%s] and host [%s]", *configFile, *host)

	if *configFile == "" {
		return fmt.Errorf("need a config file")
	}

	conf := &framecalibration.Config{}
	err := vmodutils.ReadJSONFromFile(*configFile, conf)
	if err != nil {
		return err
	}

	_, _, err = conf.Validate("")
	if err != nil {
		return err
	}

	client, err := vmodutils.ConnectToHostFromCLIToken(ctx, *host, logger)
	if err != nil {
		return err
	}
	defer client.Close(ctx)

	deps, err := vmodutils.MachineToDependencies(client)
	if err != nil {
		return err
	}

	thing, err := framecalibration.NewArmCamera(ctx, deps, generic.Named("foo"), conf, logger)
	if err != nil {
		return err
	}
	defer thing.Close(ctx)

	if len(conf.JointPositions) > 0 {
		p, cost, err := thing.Calibrate(ctx)
		if err != nil {
			return err
		}
		logger.Infof("p: %v cost: %0.2f", p, cost)
		return nil
	}

	p, err := thing.AutoCalibrate(ctx)
	if err != nil {
		return err
	}

	logger.Infof("found %v", p.Pose())

	return nil
}
