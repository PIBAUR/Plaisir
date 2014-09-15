; Auto-generated. Do not edit!


(cl:in-package test_suivi-srv)


;//! \htmlinclude SendPose-request.msg.html

(cl:defclass <SendPose-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SendPose-request (<SendPose-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SendPose-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SendPose-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name test_suivi-srv:<SendPose-request> is deprecated: use test_suivi-srv:SendPose-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SendPose-request>) ostream)
  "Serializes a message object of type '<SendPose-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SendPose-request>) istream)
  "Deserializes a message object of type '<SendPose-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SendPose-request>)))
  "Returns string type for a service object of type '<SendPose-request>"
  "test_suivi/SendPoseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendPose-request)))
  "Returns string type for a service object of type 'SendPose-request"
  "test_suivi/SendPoseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SendPose-request>)))
  "Returns md5sum for a message object of type '<SendPose-request>"
  "24dd8748a811e06ceaa50ae2d2754ada")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SendPose-request)))
  "Returns md5sum for a message object of type 'SendPose-request"
  "24dd8748a811e06ceaa50ae2d2754ada")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SendPose-request>)))
  "Returns full string definition for message of type '<SendPose-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SendPose-request)))
  "Returns full string definition for message of type 'SendPose-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SendPose-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SendPose-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SendPose-request
))
;//! \htmlinclude SendPose-response.msg.html

(cl:defclass <SendPose-response> (roslisp-msg-protocol:ros-message)
  ((p
    :reader p
    :initarg :p
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped)))
)

(cl:defclass SendPose-response (<SendPose-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SendPose-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SendPose-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name test_suivi-srv:<SendPose-response> is deprecated: use test_suivi-srv:SendPose-response instead.")))

(cl:ensure-generic-function 'p-val :lambda-list '(m))
(cl:defmethod p-val ((m <SendPose-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader test_suivi-srv:p-val is deprecated.  Use test_suivi-srv:p instead.")
  (p m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SendPose-response>) ostream)
  "Serializes a message object of type '<SendPose-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'p) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SendPose-response>) istream)
  "Deserializes a message object of type '<SendPose-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'p) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SendPose-response>)))
  "Returns string type for a service object of type '<SendPose-response>"
  "test_suivi/SendPoseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendPose-response)))
  "Returns string type for a service object of type 'SendPose-response"
  "test_suivi/SendPoseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SendPose-response>)))
  "Returns md5sum for a message object of type '<SendPose-response>"
  "24dd8748a811e06ceaa50ae2d2754ada")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SendPose-response)))
  "Returns md5sum for a message object of type 'SendPose-response"
  "24dd8748a811e06ceaa50ae2d2754ada")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SendPose-response>)))
  "Returns full string definition for message of type '<SendPose-response>"
  (cl:format cl:nil "geometry_msgs/PoseStamped p~%~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SendPose-response)))
  "Returns full string definition for message of type 'SendPose-response"
  (cl:format cl:nil "geometry_msgs/PoseStamped p~%~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SendPose-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'p))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SendPose-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SendPose-response
    (cl:cons ':p (p msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SendPose)))
  'SendPose-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SendPose)))
  'SendPose-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SendPose)))
  "Returns string type for a service object of type '<SendPose>"
  "test_suivi/SendPose")