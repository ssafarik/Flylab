FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/image_gui/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/image_gui/msg/__init__.py"
  "../src/image_gui/msg/_DrawObject.py"
  "../src/image_gui/msg/_CvLine.py"
  "../src/image_gui/msg/_CvSize.py"
  "../src/image_gui/msg/_CvColor.py"
  "../src/image_gui/msg/_CvCircle.py"
  "../src/image_gui/msg/_CvScalar.py"
  "../src/image_gui/msg/_DrawObjects.py"
  "../src/image_gui/msg/_CvPoint.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
