FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/flystage/msg"
  "../src/flystage/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/MsgFrameState.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_MsgFrameState.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
