
(cl:in-package :asdf)

(defsystem "fabot_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ArmMsg" :depends-on ("_package_ArmMsg"))
    (:file "_package_ArmMsg" :depends-on ("_package"))
  ))