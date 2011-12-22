FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/plate_tf/msg"
  "../src/plate_tf/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/PlateStageConversion.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_PlateStageConversion.lisp"
  "../srv_gen/lisp/PlateCameraConversion.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_PlateCameraConversion.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
