FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/flycore/msg"
  "../src/flycore/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/flycore/msg/__init__.py"
  "../src/flycore/msg/_MsgFrameState.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
