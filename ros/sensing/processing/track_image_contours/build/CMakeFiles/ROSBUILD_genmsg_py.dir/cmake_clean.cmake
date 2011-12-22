FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/track_image_contours/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/track_image_contours/msg/__init__.py"
  "../src/track_image_contours/msg/_ContourInfo.py"
  "../src/track_image_contours/msg/_Contour.py"
  "../src/track_image_contours/msg/_ArenaState.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
