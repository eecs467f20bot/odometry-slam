#!/bin/bash

echo "Launching MBot stream with Monocular sensor"
bot-lcm-tunnel 192.168.3.1 &
./mono_inertial_mbot ../Vocabulary/ORBvoc.txt mbot.yaml
