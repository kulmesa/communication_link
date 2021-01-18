# Communication link - ROS2 Node

## Generate a device key pair
```
openssl req -x509 -newkey rsa:2048 -keyout rsa_private.pem -nodes -out rsa_cert.pem -subj "/CN=unused"
```
The `rsa_private.pem` will stay on the device side and the public part `rsa_cert.pem` will be uploaded to the MQTT broker.

## Building

The application can be built as follows
```
source ../../install/setup.bash
go build
```

In addition to communication link, the gstreamer node can be built as well from this repo.
Gstreamer node listens the video stream and streams it to rtsp server in cloud.
It can be built as follows:
```
cd gstnode
go build
```

## Running

```
./communication_link -device_id "<my-device-id>"
gstnode/gstnode -device_id "<my-device-id>"
```


