#!/bin/bash

#------------------------------------
# Monocular 
echo "Launching MBot feed with Monocular sensor"
./mono_mbot ../Vocabulary/ORBvoc.txt mbot.yaml datasets/mono_mbot timestamps/mono_mbot.txt dataset-mono_mbot

#------------------------------------
# Monocular-Inertial
# echo "Launching MH01 with Monocular-Inertial sensor"
# ./Examples/Monocular-Inertial/mono_inertial_euroc ./Vocabulary/ORBvoc.txt ./Examples/Monocular-Inertial/EuRoC.yaml "$pathDatasetEuroc"/MH01 ./Examples/Monocular-Inertial/EuRoC_TimeStamps/MH01.txt dataset-MH01_monoi

