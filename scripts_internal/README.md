Scripts for Internal Use (Not to Release)
=========================================
All the script should be run in the git repo home directory.

## Prepare for Release
```
~/j7ros_home/ros_ws/src/jacinto_ros_perception$ ./scripts_internal/prepare_for_release.sh
```

## Replace `CMakeLists_release.txt` to `CMakeLists.txt`
```
~/j7ros_home/ros_ws/src/jacinto_ros_perception$ ./scripts_internal/rename_cmakelists_release.sh
```

To undo renaming,
```
~/j7ros_home/ros_ws/src/jacinto_ros_perception$ ./scripts_internal/rename_cmakelists_release.sh undo
```

