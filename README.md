# bokeh-camera

If you think the Google/Zoom segmentation-based background blur effects look kind of fake, this attempts to simulate a more realistic-looking bokeh using a RealSense D455 camera.

Work in progress, code not yet cleaned up, but roughly:

0. Run ./build-this

1. Set up a v4l2 loopback device at /dev/video20
```
sudo modprobe -r v4l2loopback
sudo modprobe v4l2loopback devices=1 video_nr=20 exclusive_caps=1
```

2. Run ./BokehCamera

![image](/images/screenshot0.jpg "image")

![image](/images/screenshot1.jpg "image")

**Will it work on a D435/D415/L515?**

Not yet but go ahead and hack at it


