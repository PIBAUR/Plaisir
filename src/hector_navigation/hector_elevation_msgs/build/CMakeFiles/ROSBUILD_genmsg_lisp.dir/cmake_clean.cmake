FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/hector_elevation_msgs/msg"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/ElevationGrid.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_ElevationGrid.lisp"
  "../msg_gen/lisp/ElevationMapMetaData.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_ElevationMapMetaData.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
