FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/flycore/msg"
  "../src/flycore/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/flycore/srv/__init__.py"
  "../src/flycore/srv/_SrvFrameState.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
