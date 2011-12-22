FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/track_image_contours/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/track_image_contours/ContourInfo.h"
  "../msg_gen/cpp/include/track_image_contours/Contour.h"
  "../msg_gen/cpp/include/track_image_contours/ArenaState.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
