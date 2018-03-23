# CameraInfoPublisher is for stereo camera info generation.

## Stereo Camera
Run
```
    roslaunch undistort_images stereo.launch
```

### Subscribed Topics

/wide/left/image_raw

/wide/right/image_raw

### Published Topics

/wide/left/camera_info

/wide/right/camera_info

### Required File

Calibration files in *.yaml* format (left and right)


## Monocular Camera

Run
```
    roslaunch undistort_images mono.launch
```
