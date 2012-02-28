FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/image_gui/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/image_gui/DrawObject.h"
  "../msg_gen/cpp/include/image_gui/CvLine.h"
  "../msg_gen/cpp/include/image_gui/CvSize.h"
  "../msg_gen/cpp/include/image_gui/CvColor.h"
  "../msg_gen/cpp/include/image_gui/CvCircle.h"
  "../msg_gen/cpp/include/image_gui/CvScalar.h"
  "../msg_gen/cpp/include/image_gui/DrawObjects.h"
  "../msg_gen/cpp/include/image_gui/CvPoint.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
