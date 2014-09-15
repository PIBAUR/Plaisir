FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_gencfg_cpp"
  "../cfg/cpp/hector_costmap/CostMapCalculationConfig.h"
  "../docs/CostMapCalculationConfig.dox"
  "../docs/CostMapCalculationConfig-usage.dox"
  "../src/hector_costmap/cfg/CostMapCalculationConfig.py"
  "../docs/CostMapCalculationConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gencfg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
