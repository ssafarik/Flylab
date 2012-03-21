FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/joystick_ps3/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/joystick_ps3/msg/__init__.py"
  "../src/joystick_ps3/msg/_JoystickValues.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
