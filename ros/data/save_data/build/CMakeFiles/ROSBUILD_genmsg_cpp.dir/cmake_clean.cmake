FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/save_data/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/save_data/BagInfo.h"
  "../msg_gen/cpp/include/save_data/VideoInfo.h"
  "../msg_gen/cpp/include/save_data/SaveSettings.h"
  "../msg_gen/cpp/include/save_data/CommandSavedata.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
