; Auto-generated. Do not edit!


(cl:in-package sigsent-msg)


;//! \htmlinclude GPSList.msg.html

(cl:defclass <GPSList> (roslisp-msg-protocol:ros-message)
  ((goals
    :reader goals
    :initarg :goals
    :type (cl:vector sensor_msgs-msg:NavSatFix)
   :initform (cl:make-array 0 :element-type 'sensor_msgs-msg:NavSatFix :initial-element (cl:make-instance 'sensor_msgs-msg:NavSatFix))))
)

(cl:defclass GPSList (<GPSList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GPSList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GPSList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sigsent-msg:<GPSList> is deprecated: use sigsent-msg:GPSList instead.")))

(cl:ensure-generic-function 'goals-val :lambda-list '(m))
(cl:defmethod goals-val ((m <GPSList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sigsent-msg:goals-val is deprecated.  Use sigsent-msg:goals instead.")
  (goals m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GPSList>) ostream)
  "Serializes a message object of type '<GPSList>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'goals))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'goals))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GPSList>) istream)
  "Deserializes a message object of type '<GPSList>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'goals) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'goals)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'sensor_msgs-msg:NavSatFix))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GPSList>)))
  "Returns string type for a message object of type '<GPSList>"
  "sigsent/GPSList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GPSList)))
  "Returns string type for a message object of type 'GPSList"
  "sigsent/GPSList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GPSList>)))
  "Returns md5sum for a message object of type '<GPSList>"
  "e0b6ef8f3de100059e28b6377b1938ca")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GPSList)))
  "Returns md5sum for a message object of type 'GPSList"
  "e0b6ef8f3de100059e28b6377b1938ca")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GPSList>)))
  "Returns full string definition for message of type '<GPSList>"
  (cl:format cl:nil "sensor_msgs/NavSatFix[] goals~%================================================================================~%MSG: sensor_msgs/NavSatFix~%# Navigation Satellite fix for any Global Navigation Satellite System~%#~%# Specified using the WGS 84 reference ellipsoid~%~%# header.stamp specifies the ROS time for this measurement (the~%#        corresponding satellite time may be reported using the~%#        sensor_msgs/TimeReference message).~%#~%# header.frame_id is the frame of reference reported by the satellite~%#        receiver, usually the location of the antenna.  This is a~%#        Euclidean frame relative to the vehicle, not a reference~%#        ellipsoid.~%Header header~%~%# satellite fix status information~%NavSatStatus status~%~%# Latitude [degrees]. Positive is north of equator; negative is south.~%float64 latitude~%~%# Longitude [degrees]. Positive is east of prime meridian; negative is west.~%float64 longitude~%~%# Altitude [m]. Positive is above the WGS 84 ellipsoid~%# (quiet NaN if no altitude is available).~%float64 altitude~%~%# Position covariance [m^2] defined relative to a tangential plane~%# through the reported position. The components are East, North, and~%# Up (ENU), in row-major order.~%#~%# Beware: this coordinate system exhibits singularities at the poles.~%~%float64[9] position_covariance~%~%# If the covariance of the fix is known, fill it in completely. If the~%# GPS receiver provides the variance of each measurement, put them~%# along the diagonal. If only Dilution of Precision is available,~%# estimate an approximate covariance from that.~%~%uint8 COVARIANCE_TYPE_UNKNOWN = 0~%uint8 COVARIANCE_TYPE_APPROXIMATED = 1~%uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN = 2~%uint8 COVARIANCE_TYPE_KNOWN = 3~%~%uint8 position_covariance_type~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/NavSatStatus~%# Navigation Satellite fix status for any Global Navigation Satellite System~%~%# Whether to output an augmented fix is determined by both the fix~%# type and the last time differential corrections were received.  A~%# fix is valid when status >= STATUS_FIX.~%~%int8 STATUS_NO_FIX =  -1        # unable to fix position~%int8 STATUS_FIX =      0        # unaugmented fix~%int8 STATUS_SBAS_FIX = 1        # with satellite-based augmentation~%int8 STATUS_GBAS_FIX = 2        # with ground-based augmentation~%~%int8 status~%~%# Bits defining which Global Navigation Satellite System signals were~%# used by the receiver.~%~%uint16 SERVICE_GPS =     1~%uint16 SERVICE_GLONASS = 2~%uint16 SERVICE_COMPASS = 4      # includes BeiDou.~%uint16 SERVICE_GALILEO = 8~%~%uint16 service~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GPSList)))
  "Returns full string definition for message of type 'GPSList"
  (cl:format cl:nil "sensor_msgs/NavSatFix[] goals~%================================================================================~%MSG: sensor_msgs/NavSatFix~%# Navigation Satellite fix for any Global Navigation Satellite System~%#~%# Specified using the WGS 84 reference ellipsoid~%~%# header.stamp specifies the ROS time for this measurement (the~%#        corresponding satellite time may be reported using the~%#        sensor_msgs/TimeReference message).~%#~%# header.frame_id is the frame of reference reported by the satellite~%#        receiver, usually the location of the antenna.  This is a~%#        Euclidean frame relative to the vehicle, not a reference~%#        ellipsoid.~%Header header~%~%# satellite fix status information~%NavSatStatus status~%~%# Latitude [degrees]. Positive is north of equator; negative is south.~%float64 latitude~%~%# Longitude [degrees]. Positive is east of prime meridian; negative is west.~%float64 longitude~%~%# Altitude [m]. Positive is above the WGS 84 ellipsoid~%# (quiet NaN if no altitude is available).~%float64 altitude~%~%# Position covariance [m^2] defined relative to a tangential plane~%# through the reported position. The components are East, North, and~%# Up (ENU), in row-major order.~%#~%# Beware: this coordinate system exhibits singularities at the poles.~%~%float64[9] position_covariance~%~%# If the covariance of the fix is known, fill it in completely. If the~%# GPS receiver provides the variance of each measurement, put them~%# along the diagonal. If only Dilution of Precision is available,~%# estimate an approximate covariance from that.~%~%uint8 COVARIANCE_TYPE_UNKNOWN = 0~%uint8 COVARIANCE_TYPE_APPROXIMATED = 1~%uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN = 2~%uint8 COVARIANCE_TYPE_KNOWN = 3~%~%uint8 position_covariance_type~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/NavSatStatus~%# Navigation Satellite fix status for any Global Navigation Satellite System~%~%# Whether to output an augmented fix is determined by both the fix~%# type and the last time differential corrections were received.  A~%# fix is valid when status >= STATUS_FIX.~%~%int8 STATUS_NO_FIX =  -1        # unable to fix position~%int8 STATUS_FIX =      0        # unaugmented fix~%int8 STATUS_SBAS_FIX = 1        # with satellite-based augmentation~%int8 STATUS_GBAS_FIX = 2        # with ground-based augmentation~%~%int8 status~%~%# Bits defining which Global Navigation Satellite System signals were~%# used by the receiver.~%~%uint16 SERVICE_GPS =     1~%uint16 SERVICE_GLONASS = 2~%uint16 SERVICE_COMPASS = 4      # includes BeiDou.~%uint16 SERVICE_GALILEO = 8~%~%uint16 service~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GPSList>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'goals) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GPSList>))
  "Converts a ROS message object to a list"
  (cl:list 'GPSList
    (cl:cons ':goals (goals msg))
))
