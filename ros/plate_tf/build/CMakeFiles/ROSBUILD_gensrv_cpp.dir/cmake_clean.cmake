FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/plate_tf/msg"
  "../src/plate_tf/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/plate_tf/PlateStageConversion.h"
  "../srv_gen/cpp/include/plate_tf/PlateCameraConversion.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
