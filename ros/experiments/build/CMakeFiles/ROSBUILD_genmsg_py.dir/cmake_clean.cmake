FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/experiments/msg"
  "../src/experiments/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/experiments/msg/__init__.py"
  "../src/experiments/msg/_TriggerSettings.py"
  "../src/experiments/msg/_SaveSettings.py"
  "../src/experiments/msg/_MoveSettings.py"
  "../src/experiments/msg/_HomeSettings.py"
  "../src/experiments/msg/_ExperimentSettings.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
