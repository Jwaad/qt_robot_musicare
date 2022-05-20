
(cl:in-package :asdf)

(defsystem "jwaad_test-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "FaceLockOnAction" :depends-on ("_package_FaceLockOnAction"))
    (:file "_package_FaceLockOnAction" :depends-on ("_package"))
    (:file "FaceLockOnActionFeedback" :depends-on ("_package_FaceLockOnActionFeedback"))
    (:file "_package_FaceLockOnActionFeedback" :depends-on ("_package"))
    (:file "FaceLockOnActionGoal" :depends-on ("_package_FaceLockOnActionGoal"))
    (:file "_package_FaceLockOnActionGoal" :depends-on ("_package"))
    (:file "FaceLockOnActionResult" :depends-on ("_package_FaceLockOnActionResult"))
    (:file "_package_FaceLockOnActionResult" :depends-on ("_package"))
    (:file "FaceLockOnFeedback" :depends-on ("_package_FaceLockOnFeedback"))
    (:file "_package_FaceLockOnFeedback" :depends-on ("_package"))
    (:file "FaceLockOnGoal" :depends-on ("_package_FaceLockOnGoal"))
    (:file "_package_FaceLockOnGoal" :depends-on ("_package"))
    (:file "FaceLockOnResult" :depends-on ("_package_FaceLockOnResult"))
    (:file "_package_FaceLockOnResult" :depends-on ("_package"))
  ))