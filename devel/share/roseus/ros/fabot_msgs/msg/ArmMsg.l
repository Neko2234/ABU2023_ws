;; Auto-generated. Do not edit!


(when (boundp 'fabot_msgs::ArmMsg)
  (if (not (find-package "FABOT_MSGS"))
    (make-package "FABOT_MSGS"))
  (shadow 'ArmMsg (find-package "FABOT_MSGS")))
(unless (find-package "FABOT_MSGS::ARMMSG")
  (make-package "FABOT_MSGS::ARMMSG"))

(in-package "ROS")
;;//! \htmlinclude ArmMsg.msg.html


(defclass fabot_msgs::ArmMsg
  :super ros::object
  :slots (_hand _arm _hand_duty _arm_duty ))

(defmethod fabot_msgs::ArmMsg
  (:init
   (&key
    ((:hand __hand) 0)
    ((:arm __arm) 0)
    ((:hand_duty __hand_duty) 0)
    ((:arm_duty __arm_duty) 0)
    )
   (send-super :init)
   (setq _hand (round __hand))
   (setq _arm (round __arm))
   (setq _hand_duty (round __hand_duty))
   (setq _arm_duty (round __arm_duty))
   self)
  (:hand
   (&optional __hand)
   (if __hand (setq _hand __hand)) _hand)
  (:arm
   (&optional __arm)
   (if __arm (setq _arm __arm)) _arm)
  (:hand_duty
   (&optional __hand_duty)
   (if __hand_duty (setq _hand_duty __hand_duty)) _hand_duty)
  (:arm_duty
   (&optional __arm_duty)
   (if __arm_duty (setq _arm_duty __arm_duty)) _arm_duty)
  (:serialization-length
   ()
   (+
    ;; int16 _hand
    2
    ;; int16 _arm
    2
    ;; int16 _hand_duty
    2
    ;; int16 _arm_duty
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _hand
       (write-word _hand s)
     ;; int16 _arm
       (write-word _arm s)
     ;; int16 _hand_duty
       (write-word _hand_duty s)
     ;; int16 _arm_duty
       (write-word _arm_duty s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _hand
     (setq _hand (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _arm
     (setq _arm (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _hand_duty
     (setq _hand_duty (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _arm_duty
     (setq _arm_duty (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get fabot_msgs::ArmMsg :md5sum-) "7e5814a5ce25cfec12c99674e013cee3")
(setf (get fabot_msgs::ArmMsg :datatype-) "fabot_msgs/ArmMsg")
(setf (get fabot_msgs::ArmMsg :definition-)
      "int16 hand
int16 arm
int16 hand_duty
int16 arm_duty

")



(provide :fabot_msgs/ArmMsg "7e5814a5ce25cfec12c99674e013cee3")


