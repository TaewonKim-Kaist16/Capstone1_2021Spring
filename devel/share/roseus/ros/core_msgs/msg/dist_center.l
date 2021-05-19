;; Auto-generated. Do not edit!


(when (boundp 'core_msgs::dist_center)
  (if (not (find-package "CORE_MSGS"))
    (make-package "CORE_MSGS"))
  (shadow 'dist_center (find-package "CORE_MSGS")))
(unless (find-package "CORE_MSGS::DIST_CENTER")
  (make-package "CORE_MSGS::DIST_CENTER"))

(in-package "ROS")
;;//! \htmlinclude dist_center.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass core_msgs::dist_center
  :super ros::object
  :slots (_header _size _dist ))

(defmethod core_msgs::dist_center
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:size __size) 0)
    ((:dist __dist) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _size (round __size))
   (setq _dist __dist)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:size
   (&optional __size)
   (if __size (setq _size __size)) _size)
  (:dist
   (&optional __dist)
   (if __dist (setq _dist __dist)) _dist)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _size
    4
    ;; float32[] _dist
    (* 4    (length _dist)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _size
       (write-long _size s)
     ;; float32[] _dist
     (write-long (length _dist) s)
     (dotimes (i (length _dist))
       (sys::poke (elt _dist i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _size
     (setq _size (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32[] _dist
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _dist (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _dist i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get core_msgs::dist_center :md5sum-) "1e652f8d15dd8848ad77cfc482f45e66")
(setf (get core_msgs::dist_center :datatype-) "core_msgs/dist_center")
(setf (get core_msgs::dist_center :definition-)
      "Header header
int32 size
float32[] dist

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :core_msgs/dist_center "1e652f8d15dd8848ad77cfc482f45e66")


