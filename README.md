# Module frame-calibration 

This module is used for calibrating frames of cameras to be used in the frame system.

## Model viam:frame-calibration:arm-camera

This model is used to determine the frame of a camera mounted on an arm to be consumed by the frame system.

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
| `joint_positions` | [][]float  | Required  | joint positions that the arm should move through |

#### Example Configuration

```json
{
  "arm": "my-arm",
  "tracker": "my-tracker",
  "joint_positions": [
    [10,20,30,40,50,60], 
    [60,50,40,30,20,10], 
  ]
}
```

### DoCommand

#### runCalibration DoCommand

`runCalibration` will run calibration using the currently defined positions, running calibration the specified amount of times. Returns the calibrated frame and associated cost from the optimization.

```json
{
  "runCalibration": <int>
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

`getCalibrationPositions` will return the current list of calibration positions.

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

`saveCalibrationPosition` will save the current arm position into the list of positions to calibrate with and update the `viam:frame-calibration:arm-camera`'s config. Passing in a positive integer will replace the specified index, while other inputs will append the positions at the end of the list. Use the config history feature if you wish to recover the removed positions.

```json
{
  "saveCalibrationPosition": <int> or ""
}
```

#### deleteCalibrationPosition DoCommand

`deleteCalibrationPosition` will remove the specified set of joint positions from the `viam:frame-calibration:arm-camera`'s config. Use the config history feature if you wish to recover the removed positions.

```json
{
  "deleteCalibrationPosition": <int>
}
```

#### clearCalibrationPositions DoCommand

`clearCalibrationPositions` will remove the all joint positions from the `viam:frame-calibration:arm-camera`'s config. Use the config history feature if you wish to recover the removed positions.

```json
{
  "clearCalibrationPositions": ""
}
```
