
(cl:in-package :asdf)

(defsystem "stage_action_server-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :flycore-msg
               :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "UpdateStagePositionFeedback" :depends-on ("_package_UpdateStagePositionFeedback"))
    (:file "_package_UpdateStagePositionFeedback" :depends-on ("_package"))
    (:file "ActionStageStateActionGoal" :depends-on ("_package_ActionStageStateActionGoal"))
    (:file "_package_ActionStageStateActionGoal" :depends-on ("_package"))
    (:file "ActionStageStateFeedback" :depends-on ("_package_ActionStageStateFeedback"))
    (:file "_package_ActionStageStateFeedback" :depends-on ("_package"))
    (:file "ActionStageStateActionFeedback" :depends-on ("_package_ActionStageStateActionFeedback"))
    (:file "_package_ActionStageStateActionFeedback" :depends-on ("_package"))
    (:file "UpdateStagePositionAction" :depends-on ("_package_UpdateStagePositionAction"))
    (:file "_package_UpdateStagePositionAction" :depends-on ("_package"))
    (:file "UpdateStagePositionActionFeedback" :depends-on ("_package_UpdateStagePositionActionFeedback"))
    (:file "_package_UpdateStagePositionActionFeedback" :depends-on ("_package"))
    (:file "ActionStageStateGoal" :depends-on ("_package_ActionStageStateGoal"))
    (:file "_package_ActionStageStateGoal" :depends-on ("_package"))
    (:file "UpdateStagePositionGoal" :depends-on ("_package_UpdateStagePositionGoal"))
    (:file "_package_UpdateStagePositionGoal" :depends-on ("_package"))
    (:file "ActionStageStateActionResult" :depends-on ("_package_ActionStageStateActionResult"))
    (:file "_package_ActionStageStateActionResult" :depends-on ("_package"))
    (:file "UpdateStagePositionResult" :depends-on ("_package_UpdateStagePositionResult"))
    (:file "_package_UpdateStagePositionResult" :depends-on ("_package"))
    (:file "ActionStageStateAction" :depends-on ("_package_ActionStageStateAction"))
    (:file "_package_ActionStageStateAction" :depends-on ("_package"))
    (:file "ActionStageStateResult" :depends-on ("_package_ActionStageStateResult"))
    (:file "_package_ActionStageStateResult" :depends-on ("_package"))
    (:file "UpdateStagePositionActionGoal" :depends-on ("_package_UpdateStagePositionActionGoal"))
    (:file "_package_UpdateStagePositionActionGoal" :depends-on ("_package"))
    (:file "UpdateStagePositionActionResult" :depends-on ("_package_UpdateStagePositionActionResult"))
    (:file "_package_UpdateStagePositionActionResult" :depends-on ("_package"))
  ))