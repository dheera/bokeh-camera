# bokeh-camera

If you think the Google/Zoom segmentation-based background blur effects look kind of fake, this attempts to simulate a more realistic-looking bokeh using a RealSense D455 camera on a virtual camera device that you can use from any conferencing software.

Work in progress, code not yet cleaned up, but roughly:

-1. Requirements: librealsense2-dev, libopencv-dev

0. Run ./build-this

1. Set up a v4l2 loopback device at /dev/video20
```
sudo modprobe -r v4l2loopback
sudo modprobe v4l2loopback devices=1 video_nr=20 exclusive_caps=1
```

2. Run ./BokehCamera

3. Select the virtual camera on Google Meet or Zoom or whatever you use

# Controls

While it is running:

Press 1 to send bokeh-fied image

Press 2 to send a map of the computed amount of blur (white = don't blur, black = full blur)

Press 3 to send the depth map

Press 4 to send no-bokeh RGB

Press \[ or \] to refocus the image

![image](/images/screenshot0.jpg "image")

![image](/images/screenshot1.jpg "image")

**Will it work on a D435/D415/L515?**

Not yet but go ahead and hack at it


