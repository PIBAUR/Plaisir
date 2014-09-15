FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_gencfg_cpp"
  "../cfg/cpp/hector_elevation_visualization/ElevationVisualizationConfig.h"
  "../docs/ElevationVisualizationConfig.dox"
  "../docs/ElevationVisualizationConfig-usage.dox"
  "../src/hector_elevation_visualization/cfg/ElevationVisualizationConfig.py"
  "../docs/ElevationVisualizationConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gencfg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
