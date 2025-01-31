# Camera Setup
Documentation of the camera setup used by FRC 3414 Hackbots.

## Number of Cameras
Our preliminary testing suggests that one Orange Pi 5 (16GB ram) can support 6 arducams (OV9281) running at 320 x 240 pixels, at 120 requested FPS.
We saw FPS speeds around 90-120 FPS.
We connected the 6 cameras as follows:

**USB 2.0** was used for two cameras, each connected directly into the Orange Pi.

**USB 3.0** was used for a USB hub that connected 4 more cameras.

## Considerations for using ArduCams
It is imperitive that the ArduCam Serial Number Tool is used to set unique device names for each ArduCam.

## AprilTag detection range
We were able to accurately identify April tags from up to ~11 feet.
With 3D detection, we saw tolerable ambiguity values (<0.2) for about ~9 feet.

## Calibration
Calibration was performed with 25 images, covering the majority of the frame.
We saw excellent ambiguity values (< 0.05) for targets within 4 feet of the camera.

## Conclusion
Overall, we were very satisfied with this performance, and we expect that we will be able to use it very well for the remainder of the season.