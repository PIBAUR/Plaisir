FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_gencfg_cpp"
  "../cfg/cpp/hector_exploration_planner/ExplorationPlannerConfig.h"
  "../docs/ExplorationPlannerConfig.dox"
  "../docs/ExplorationPlannerConfig-usage.dox"
  "../src/hector_exploration_planner/cfg/ExplorationPlannerConfig.py"
  "../docs/ExplorationPlannerConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gencfg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
