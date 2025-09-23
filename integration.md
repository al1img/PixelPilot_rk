# Integration

## Stream camera from host to device

```console
gst-launch-1.0 v4l2src ! videoconvert ! video/x-raw,format=I420 ! x265enc tune=zerolatency bitrate=5000000 ! rtph265pay config-interval=1 ! udpsink host=orangepi5pro port=5600
```

## Launch PixelPilot

```console
build/pixelpilot --osd --osd-config config_osd.json --wfb-api-port 0
```
