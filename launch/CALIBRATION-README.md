# About

This README tells you how to get camera calibration info

# Calibration

Replace logitech_04 with the name of the camera you are calibrating.
Replace 8x5 and 0.028 with the proper measurements for your calibration image.

```
roslaunch theora_webcams calibrate_camera.launch camera:=logitech_04
source devel/setup.bash
rosrun camera_calibration cameracalibrator.py --size 17x23 --square 0.05 image:=/usb_cam/image_raw camera:=usb_cam
```

Do the calibration

The results will be saved as a tar gz file. Extract it with the following command.

Replace <file> with the tar archive it tells you it saved the data to.

```
tar -xzf <file>
```

Keep the ost.yaml file that gets extracted.
