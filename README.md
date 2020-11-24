# Communication link - ROS2 Node

## Generate a device key pair
```
openssl req -x509 -newkey rsa:2048 -keyout rsa_private.pem -nodes -out rsa_cert.pem -subj "/CN=unused"
```
The `rsa_private.pem` will stay on the device side and the public part `rsa_cert.pem` will be uploaded to the MQTT broker.

## Building

The node uses private library from github.com/ssrc-tii/rclgo so to allow go build fetch dependencies we need to allow this. When using ssh keys with github the easiest way is to config git to use git@github.com instead of https
```
git config --global url."git@github.com:".insteadOf "https://github.com/"
```

On top of this we need to disable the version information fetch by marking the repository as private
```
export GOPRIVATE=github.com/ssrc-tii
```

Now we can build the application
```
source ../../install/setup.bash
cd roswrapper
mkdir build && cd build
cmake ..
make
cd ..
export LD_LIBRARY_PATH=./roswrapper/build:$LD_LIBRARY_PATH
go build
```

## Running

```
./communication_link -device_id "<my-device-id>"
```
