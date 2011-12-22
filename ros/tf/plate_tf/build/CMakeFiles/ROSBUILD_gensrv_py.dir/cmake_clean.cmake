FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/plate_tf/msg"
  "../src/plate_tf/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/plate_tf/srv/__init__.py"
  "../src/plate_tf/srv/_PlateStageConversion.py"
  "../src/plate_tf/srv/_PlateCameraConversion.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
