FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/experiments/msg"
  "../src/experiments/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/experiments/srv/__init__.py"
  "../src/experiments/srv/_Trigger.py"
  "../src/experiments/srv/_ExperimentParams.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
