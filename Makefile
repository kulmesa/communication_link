ARCH := $(shell dpkg --print-architecture)
RELEASE := $(shell lsb_release -cs)

build:
	openssl req -x509 -newkey rsa:2048 -keyout rsa_private.pem -nodes -out rsa_cert.pem -subj "/CN=unused"
	go build .

install:
	# If the service has been previously installed, stop it so the running binaries can be replaced
	systemctl stop communication_link 2> /dev/null || true
	apt update

	# Setup timezone
	echo 'Etc/UTC' > /etc/timezone
	(test ! -f /etc/localtime && ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime) || true
	apt install -q -y tzdata

	# Install ROS 2
	apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
	echo "deb [arch=$(ARCH)] http://packages.ros.org/ros2/ubuntu $(RELEASE) main" > /etc/apt/sources.list.d/ros2-latest.list
	apt update
	apt install -y --no-install-recommends ros-foxy-ros-base ros-foxy-ros-core

	# Install build dependencies
	apt install -y --no-install-recommends build-essential fakeroot git-core golang ros-foxy-ros-base libgstreamer1.0-0 libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev libgstreamer-plugins-good1.0-dev

	# Install px4-msgs
	curl -fsSL https://github.com/tiiuae/fog_sw/releases/download/ver_ssrc_rel_2/ros-foxy-px4-msgs_2.0.1-0focal_amd64.deb -o ros-foxy-px4-msgs.deb
	dpkg -i ros-foxy-px4-msgs.deb
	rm ros-foxy-px4-msgs.deb

	# Install systemd service
	cp communication_link.service /etc/systemd/system/

	# Install systemd-service
	mkdir -p /opt/communication_link
	mkdir -p /enclave
	cp communication_link /opt/communication_link/
	cp communication_link.service.wrapper.sh /opt/communication_link/
	cp communication_link.service /etc/systemd/system/
	cp rsa_private.pem /enclave/rsa_private.pem
	systemctl daemon-reload
	systemctl enable communication_link
	systemctl start communication_link

uninstall:
	systemctl stop communication_link
	rm -r /opt/communication_link
	rm /etc/systemd/system/communication_link.service
	systemctl daemon-reload
