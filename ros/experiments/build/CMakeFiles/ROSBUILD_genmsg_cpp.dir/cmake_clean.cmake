FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/experiments/msg"
  "../src/experiments/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/experiments/TriggerSettings.h"
  "../msg_gen/cpp/include/experiments/SaveSettings.h"
  "../msg_gen/cpp/include/experiments/MoveSettings.h"
  "../msg_gen/cpp/include/experiments/HomeSettings.h"
  "../msg_gen/cpp/include/experiments/ExperimentSettings.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
