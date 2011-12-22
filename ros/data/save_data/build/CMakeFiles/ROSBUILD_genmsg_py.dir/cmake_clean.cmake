FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/save_data/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/save_data/msg/__init__.py"
  "../src/save_data/msg/_BagInfo.py"
  "../src/save_data/msg/_VideoInfo.py"
  "../src/save_data/msg/_SaveSettings.py"
  "../src/save_data/msg/_CommandSavedata.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
