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

#### Obstacles

Please use the [erh:vmodutils:obstacle](https://app.viam.com/module/erh/vmodutils) model to add obstacles for the arm to plan around.

### DoCommand

#### moveArm DoCommand

This command will move the arm through any currently defined positions. Calibration will not be performed.

```json
{
  "moveArm": ""
}
```
