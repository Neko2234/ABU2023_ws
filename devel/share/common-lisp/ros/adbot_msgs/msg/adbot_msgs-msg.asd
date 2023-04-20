
(cl:in-package :asdf)

(defsystem "adbot_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SprMsg" :depends-on ("_package_SprMsg"))
    (:file "_package_SprMsg" :depends-on ("_package"))
  ))