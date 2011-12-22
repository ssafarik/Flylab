FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/flystage/msg"
  "../src/flystage/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/SrvStageState.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_SrvStageState.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
