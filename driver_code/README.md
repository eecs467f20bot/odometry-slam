This repo must be cloned into the ORB-SLAM3 folder.
To build the mbot orbslam3 C code add these three lines to the CMakeLists.txt with the
other executable definitions in the ORB-SLAM3 directory.

```
find_package(lcm REQUIRED)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/eecs467-a3)

add_executable(mono_mbot eecs467-a3/mono_mbot.cc)
target_link_libraries(mono_mbot ${PROJECT_NAME})

add_executable(mono_mbot_stream eecs467-a3/mono_mbot_stream.cc)
target_link_libraries(mono_mbot_stream ${PROJECT_NAME} lcm)
```

To create a dataset from a video, run the code:

```
cd datasets
python3 create_dataset.py {new_dataset_name} {path_to_video}
```

To run the new dataset, change the path for the dataset folder and timestamp file in
the mbot_run.sh script. Also, modify the camera distortion parameters in the mbot.yaml
file.
