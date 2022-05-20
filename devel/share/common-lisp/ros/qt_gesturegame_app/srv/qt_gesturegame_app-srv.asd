
(cl:in-package :asdf)

(defsystem "qt_gesturegame_app-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "suspend" :depends-on ("_package_suspend"))
    (:file "_package_suspend" :depends-on ("_package"))
  ))