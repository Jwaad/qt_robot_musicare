
(cl:in-package :asdf)

(defsystem "musi_care-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "qt_command" :depends-on ("_package_qt_command"))
    (:file "_package_qt_command" :depends-on ("_package"))
    (:file "sound_player_srv" :depends-on ("_package_sound_player_srv"))
    (:file "_package_sound_player_srv" :depends-on ("_package"))
  ))