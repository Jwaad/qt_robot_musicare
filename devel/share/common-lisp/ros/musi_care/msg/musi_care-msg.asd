
(cl:in-package :asdf)

(defsystem "musi_care-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SongData" :depends-on ("_package_SongData"))
    (:file "_package_SongData" :depends-on ("_package"))
  ))