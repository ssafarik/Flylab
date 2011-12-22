FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/stage_action_server/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genaction_msgs"
  "../msg/ActionStageStateAction.msg"
  "../msg/ActionStageStateGoal.msg"
  "../msg/ActionStageStateActionGoal.msg"
  "../msg/ActionStageStateResult.msg"
  "../msg/ActionStageStateActionResult.msg"
  "../msg/ActionStageStateFeedback.msg"
  "../msg/ActionStageStateActionFeedback.msg"
  "../msg/UpdateStagePositionAction.msg"
  "../msg/UpdateStagePositionGoal.msg"
  "../msg/UpdateStagePositionActionGoal.msg"
  "../msg/UpdateStagePositionResult.msg"
  "../msg/UpdateStagePositionActionResult.msg"
  "../msg/UpdateStagePositionFeedback.msg"
  "../msg/UpdateStagePositionActionFeedback.msg"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genaction_msgs.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
