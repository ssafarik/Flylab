FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/save_data/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/BagInfo.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_BagInfo.lisp"
  "../msg_gen/lisp/VideoInfo.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_VideoInfo.lisp"
  "../msg_gen/lisp/SaveSettings.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_SaveSettings.lisp"
  "../msg_gen/lisp/CommandSavedata.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_CommandSavedata.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
