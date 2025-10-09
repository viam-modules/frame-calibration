# Module frame-calibration

This module is used for calibrating frames of cameras to be used in the frame system.

## Model viam:frame-calibration:camera-on-arm

This model is used to determine the frame of a camera mounted on an arm to be consumed by the frame system.

### Usage

To use this model, take the following steps:

1. Configure a posetracker component to use for calibration. The recommended module is the [apriltag module](https://app.viam.com/module/luddite/apriltag).
2. Print out an apriltag family to use with calibration. Other kinds of posetrackers may use a different object for tracking. The frame calibration github repo has an [example apriltag family](https://github.com/viam-modules/frame-calibration/blob/main/tag36h11_1-30.pdf) to use.
3. Configure the `camera-on-arm` model with an arm and pose tracker, along with any other optional settings that you want to include.
4. Move the arm into a position where the camera can see all of the expected poses. You can use the `checkTags`DoCommand to see the number of visible tags.
5. Use the `saveCalibrationPosition` DoCommand to add this position to your config.
6. Repeat steps 4 and 5 until you have 6-8 different arm positions. Try to select positions that cover different viewpoints while still having a straightforward motion plan.
7. use the `runCalibration` DoCommand to start calibration. Have this calibration multiple times to try getting a better result. Also make sure the numbers make sense!
8. Copy the calibration result into your camera's frame config.
9. (Optional) Use the `saveCalibration` DoCommand to save the calibration guess. This will add the calibration result to the `camera-on-arm` config, which will be used for future calibrations.

### Configuration

The following attribute template can be used to configure this model:

```json
{
"arm": <string>,
"tracker": <string>,
"joint_positions": [][]<float>,
}
```

#### Attributes

The following attributes are available for this model:

| Name          | Type   | Inclusion | Description                |
|---------------|--------|-----------|----------------------------|
| `arm` | string | Required  | arm the camera is mounted on |
| `tracker` | string | Required  | pose tracker configured to detect poses from a camera |
| `motion` | string | Optional  | optional motion service to move the arm with. The **builtin** motion service is always available to configure here. |
| `arm_parent` | string | Optional  | parent frame of the arm to use with motion planning. Required if motion is configured. **world** is usually a good frame to start with. |
| `sleep_seconds` | float64 | Optional  | number of seconds for the arm to sleep between moves. Default is 1 second |
| `num_expected_tags` | int | Optional  | number of poses that the camera/pose tracker should be able to see. Default is 24 tags. |
| `joint_positions` | [][]float  | Required  | joint positions that the arm should move through. Positions are in radians |
| `guess` | FrameConfig  | Optional  | the current guess of where the camera is mounted on the arm. Use the **saveGuess** DoCommand to store the latest calibration result here. |

#### Example Configuration

```json
{
  "arm": "my-arm",
  "tracker": "my-tracker",
  "motion": "builtin",
  "arm_parent": "world",
  "joint_positions": [
    [0.1, 0.2, 0.3, 0.4, 0.5, 0.6], 
    [0, 0, 0, 0, 0, 0], 
  ]
}
```

#### Obstacles

Please use the [erh:vmodutils:obstacle](https://app.viam.com/module/erh/vmodutils) model to add obstacles for the arm to plan around.

### DoCommand

#### runCalibration DoCommand

`runCalibration` will run calibration using the currently defined positions, running calibration the specified amount of times. Returns the calibrated frame and associated cost from the optimization.

```json
{
  "runCalibration": <int>
}
```

#### autoCalibrate DoCommand

`autoCalibrate` will generate a set of positions for the arm to test based on the current position of the arm, returning the calibration frame. This feature is experimental.

```json
{
  "autoCalibrate": ""
}
```

#### saveGuess DoCommand

`saveGuess` will add the current calibration result to the config of the camera-on-arm model. This result will be used in future calibrations to help improve the optimization.

```json
{
  "saveGuess": ""
}
```

#### moveArm DoCommand

`moveArm` will move the arm through any currently defined positions and takes the time delay between moves as an input. Calibration will not be performed, so this is useful to quickly validate all arm positions can see the expected number of tags.

```json
{
  "moveArm": <int>
}
```

#### getCalibrationPositions DoCommand

`getCalibrationPositions` will return the last saved list of calibration positions.

```json
{
  "getCalibrationPositions": ""
}
```

#### moveArmToPosition DoCommand

`moveArmToPosition` will move the arm to the specified position and return the number of tags that can be seen.

```json
{
  "moveArmToPosition": <int>
}
```

#### checkTags DoCommand

`checkTags` will return the number of tags that can be seen by the camera. This is useful when moving the arm to try and find different positions for calibration.

```json
{
  "checkTags": ""
}
```

#### saveCalibrationPosition DoCommand

`saveCalibrationPosition` will save the current arm position into the list of positions to calibrate with and update the `viam:frame-calibration:camera-on-arm`'s config. Passing in a positive integer will replace the specified index, while other inputs will append the positions at the end of the list. Use the config history feature if you wish to recover the removed positions.

```json
{
  "saveCalibrationPosition": <int> or ""
}
```

#### deleteCalibrationPosition DoCommand

`deleteCalibrationPosition` will remove the specified set of joint positions from the `viam:frame-calibration:camera-on-arm`'s config. Use the config history feature if you wish to recover the removed positions.

```json
{
  "deleteCalibrationPosition": <int>
}
```

#### clearCalibrationPositions DoCommand

`clearCalibrationPositions` will remove the all joint positions from the `viam:frame-calibration:camera-on-arm`'s config. Use the config history feature if you wish to recover the removed positions.

```json
{
  "clearCalibrationPositions": ""
}
```
