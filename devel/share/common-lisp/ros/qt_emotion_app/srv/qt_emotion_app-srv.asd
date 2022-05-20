
(cl:in-package :asdf)

(defsystem "qt_emotion_app-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "suspend" :depends-on ("_package_suspend"))
    (:file "_package_suspend" :depends-on ("_package"))
  ))