; Auto-generated. Do not edit!


(cl:in-package cc_fabmap-msg)


;//! \htmlinclude keyframeGraphMsgStamped.msg.html

(cl:defclass <keyframeGraphMsgStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (numFrames
    :reader numFrames
    :initarg :numFrames
    :type cl:integer
    :initform 0)
   (frameData
    :reader frameData
    :initarg :frameData
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (numConstraints
    :reader numConstraints
    :initarg :numConstraints
    :type cl:integer
    :initform 0)
   (constraintsData
    :reader constraintsData
    :initarg :constraintsData
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass keyframeGraphMsgStamped (<keyframeGraphMsgStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <keyframeGraphMsgStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'keyframeGraphMsgStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cc_fabmap-msg:<keyframeGraphMsgStamped> is deprecated: use cc_fabmap-msg:keyframeGraphMsgStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <keyframeGraphMsgStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cc_fabmap-msg:header-val is deprecated.  Use cc_fabmap-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'numFrames-val :lambda-list '(m))
(cl:defmethod numFrames-val ((m <keyframeGraphMsgStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cc_fabmap-msg:numFrames-val is deprecated.  Use cc_fabmap-msg:numFrames instead.")
  (numFrames m))

(cl:ensure-generic-function 'frameData-val :lambda-list '(m))
(cl:defmethod frameData-val ((m <keyframeGraphMsgStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cc_fabmap-msg:frameData-val is deprecated.  Use cc_fabmap-msg:frameData instead.")
  (frameData m))

(cl:ensure-generic-function 'numConstraints-val :lambda-list '(m))
(cl:defmethod numConstraints-val ((m <keyframeGraphMsgStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cc_fabmap-msg:numConstraints-val is deprecated.  Use cc_fabmap-msg:numConstraints instead.")
  (numConstraints m))

(cl:ensure-generic-function 'constraintsData-val :lambda-list '(m))
(cl:defmethod constraintsData-val ((m <keyframeGraphMsgStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cc_fabmap-msg:constraintsData-val is deprecated.  Use cc_fabmap-msg:constraintsData instead.")
  (constraintsData m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <keyframeGraphMsgStamped>) ostream)
  "Serializes a message object of type '<keyframeGraphMsgStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'numFrames)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'numFrames)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'numFrames)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'numFrames)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'frameData))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'frameData))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'numConstraints)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'numConstraints)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'numConstraints)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'numConstraints)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'constraintsData))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'constraintsData))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <keyframeGraphMsgStamped>) istream)
  "Deserializes a message object of type '<keyframeGraphMsgStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'numFrames)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'numFrames)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'numFrames)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'numFrames)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'frameData) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'frameData)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'numConstraints)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'numConstraints)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'numConstraints)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'numConstraints)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'constraintsData) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'constraintsData)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<keyframeGraphMsgStamped>)))
  "Returns string type for a message object of type '<keyframeGraphMsgStamped>"
  "cc_fabmap/keyframeGraphMsgStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'keyframeGraphMsgStamped)))
  "Returns string type for a message object of type 'keyframeGraphMsgStamped"
  "cc_fabmap/keyframeGraphMsgStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<keyframeGraphMsgStamped>)))
  "Returns md5sum for a message object of type '<keyframeGraphMsgStamped>"
  "91770ce6cebb643af14be7d4b423d06f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'keyframeGraphMsgStamped)))
  "Returns md5sum for a message object of type 'keyframeGraphMsgStamped"
  "91770ce6cebb643af14be7d4b423d06f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<keyframeGraphMsgStamped>)))
  "Returns full string definition for message of type '<keyframeGraphMsgStamped>"
  (cl:format cl:nil "Header header~%~%# data as serialization of sim(3)'s: (int id, float[7] camToWorld)~%uint32 numFrames~%uint8[] frameData~%~%~%# constraints (int from, int to, float err)~%uint32 numConstraints~%uint8[] constraintsData~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'keyframeGraphMsgStamped)))
  "Returns full string definition for message of type 'keyframeGraphMsgStamped"
  (cl:format cl:nil "Header header~%~%# data as serialization of sim(3)'s: (int id, float[7] camToWorld)~%uint32 numFrames~%uint8[] frameData~%~%~%# constraints (int from, int to, float err)~%uint32 numConstraints~%uint8[] constraintsData~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <keyframeGraphMsgStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'frameData) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'constraintsData) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <keyframeGraphMsgStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'keyframeGraphMsgStamped
    (cl:cons ':header (header msg))
    (cl:cons ':numFrames (numFrames msg))
    (cl:cons ':frameData (frameData msg))
    (cl:cons ':numConstraints (numConstraints msg))
    (cl:cons ':constraintsData (constraintsData msg))
))
