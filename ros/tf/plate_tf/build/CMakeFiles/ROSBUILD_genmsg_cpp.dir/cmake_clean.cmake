FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/plate_tf/msg"
  "../src/plate_tf/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/plate_tf/Kinematics.h"
  "../msg_gen/cpp/include/plate_tf/FlyView.h"
  "../msg_gen/cpp/include/plate_tf/InBounds.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
