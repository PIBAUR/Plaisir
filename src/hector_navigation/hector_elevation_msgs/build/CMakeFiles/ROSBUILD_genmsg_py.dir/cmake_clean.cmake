FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/hector_elevation_msgs/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/hector_elevation_msgs/msg/__init__.py"
  "../src/hector_elevation_msgs/msg/_ElevationMapMetaData.py"
  "../src/hector_elevation_msgs/msg/_ElevationGrid.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
