; Auto-generated. Do not edit!


(cl:in-package netft_rdt_driver-srv)


;//! \htmlinclude Zero-request.msg.html

(cl:defclass <Zero-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Zero-request (<Zero-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Zero-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Zero-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name netft_rdt_driver-srv:<Zero-request> is deprecated: use netft_rdt_driver-srv:Zero-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Zero-request>) ostream)
  "Serializes a message object of type '<Zero-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Zero-request>) istream)
  "Deserializes a message object of type '<Zero-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Zero-request>)))
  "Returns string type for a service object of type '<Zero-request>"
  "netft_rdt_driver/ZeroRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Zero-request)))
  "Returns string type for a service object of type 'Zero-request"
  "netft_rdt_driver/ZeroRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Zero-request>)))
  "Returns md5sum for a message object of type '<Zero-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Zero-request)))
  "Returns md5sum for a message object of type 'Zero-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Zero-request>)))
  "Returns full string definition for message of type '<Zero-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Zero-request)))
  "Returns full string definition for message of type 'Zero-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Zero-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Zero-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Zero-request
))
;//! \htmlinclude Zero-response.msg.html

(cl:defclass <Zero-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Zero-response (<Zero-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Zero-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Zero-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name netft_rdt_driver-srv:<Zero-response> is deprecated: use netft_rdt_driver-srv:Zero-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Zero-response>) ostream)
  "Serializes a message object of type '<Zero-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Zero-response>) istream)
  "Deserializes a message object of type '<Zero-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Zero-response>)))
  "Returns string type for a service object of type '<Zero-response>"
  "netft_rdt_driver/ZeroResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Zero-response)))
  "Returns string type for a service object of type 'Zero-response"
  "netft_rdt_driver/ZeroResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Zero-response>)))
  "Returns md5sum for a message object of type '<Zero-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Zero-response)))
  "Returns md5sum for a message object of type 'Zero-response"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Zero-response>)))
  "Returns full string definition for message of type '<Zero-response>"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Zero-response)))
  "Returns full string definition for message of type 'Zero-response"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Zero-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Zero-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Zero-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Zero)))
  'Zero-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Zero)))
  'Zero-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Zero)))
  "Returns string type for a service object of type '<Zero>"
  "netft_rdt_driver/Zero")