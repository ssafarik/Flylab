FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/plate_tf/msg"
  "../src/plate_tf/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/Kinematics.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_Kinematics.lisp"
  "../msg_gen/lisp/FlyView.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_FlyView.lisp"
  "../msg_gen/lisp/InBounds.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_InBounds.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
