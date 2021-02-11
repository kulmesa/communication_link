#!/bin/bash
source /opt/ros/foxy/setup.bash

/opt/communication_link/communication_link -device_id "$DEVICE_ID" -mqtt_broker "$MQTT_BROKER_ADDRESS" -private_key "$PRIVATE_KEY_PATH"
