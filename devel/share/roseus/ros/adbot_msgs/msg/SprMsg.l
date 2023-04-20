;; Auto-generated. Do not edit!


(when (boundp 'adbot_msgs::SprMsg)
  (if (not (find-package "ADBOT_MSGS"))
    (make-package "ADBOT_MSGS"))
  (shadow 'SprMsg (find-package "ADBOT_MSGS")))
(unless (find-package "ADBOT_MSGS::SPRMSG")
  (make-package "ADBOT_MSGS::SPRMSG"))

(in-package "ROS")
;;//! \htmlinclude SprMsg.msg.html


(defclass adbot_msgs::SprMsg
  :super ros::object
  :slots (_isOn ))

(defmethod adbot_msgs::SprMsg
  (:init
   (&key
    ((:isOn __isOn) nil)
    )
   (send-super :init)
   (setq _isOn __isOn)
   self)
  (:isOn
   (&optional (__isOn :null))
   (if (not (eq __isOn :null)) (setq _isOn __isOn)) _isOn)
  (:serialization-length
   ()
   (+
    ;; bool _isOn
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _isOn
       (if _isOn (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _isOn
     (setq _isOn (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get adbot_msgs::SprMsg :md5sum-) "7815df4e3625e249c3731540caef0458")
(setf (get adbot_msgs::SprMsg :datatype-) "adbot_msgs/SprMsg")
(setf (get adbot_msgs::SprMsg :definition-)
      "bool isOn

")



(provide :adbot_msgs/SprMsg "7815df4e3625e249c3731540caef0458")


