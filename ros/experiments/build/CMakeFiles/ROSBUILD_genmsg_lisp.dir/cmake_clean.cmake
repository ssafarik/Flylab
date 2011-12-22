FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/experiments/msg"
  "../src/experiments/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/TriggerSettings.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_TriggerSettings.lisp"
  "../msg_gen/lisp/SaveSettings.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_SaveSettings.lisp"
  "../msg_gen/lisp/MoveSettings.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_MoveSettings.lisp"
  "../msg_gen/lisp/HomeSettings.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_HomeSettings.lisp"
  "../msg_gen/lisp/ExperimentSettings.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_ExperimentSettings.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
