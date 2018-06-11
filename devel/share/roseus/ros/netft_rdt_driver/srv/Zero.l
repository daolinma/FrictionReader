;; Auto-generated. Do not edit!


(when (boundp 'netft_rdt_driver::Zero)
  (if (not (find-package "NETFT_RDT_DRIVER"))
    (make-package "NETFT_RDT_DRIVER"))
  (shadow 'Zero (find-package "NETFT_RDT_DRIVER")))
(unless (find-package "NETFT_RDT_DRIVER::ZERO")
  (make-package "NETFT_RDT_DRIVER::ZERO"))
(unless (find-package "NETFT_RDT_DRIVER::ZEROREQUEST")
  (make-package "NETFT_RDT_DRIVER::ZEROREQUEST"))
(unless (find-package "NETFT_RDT_DRIVER::ZERORESPONSE")
  (make-package "NETFT_RDT_DRIVER::ZERORESPONSE"))

(in-package "ROS")





(defclass netft_rdt_driver::ZeroRequest
  :super ros::object
  :slots ())

(defmethod netft_rdt_driver::ZeroRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass netft_rdt_driver::ZeroResponse
  :super ros::object
  :slots ())

(defmethod netft_rdt_driver::ZeroResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass netft_rdt_driver::Zero
  :super ros::object
  :slots ())

(setf (get netft_rdt_driver::Zero :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get netft_rdt_driver::Zero :datatype-) "netft_rdt_driver/Zero")
(setf (get netft_rdt_driver::Zero :request) netft_rdt_driver::ZeroRequest)
(setf (get netft_rdt_driver::Zero :response) netft_rdt_driver::ZeroResponse)

(defmethod netft_rdt_driver::ZeroRequest
  (:response () (instance netft_rdt_driver::ZeroResponse :init)))

(setf (get netft_rdt_driver::ZeroRequest :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get netft_rdt_driver::ZeroRequest :datatype-) "netft_rdt_driver/ZeroRequest")
(setf (get netft_rdt_driver::ZeroRequest :definition-)
      "---


")

(setf (get netft_rdt_driver::ZeroResponse :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get netft_rdt_driver::ZeroResponse :datatype-) "netft_rdt_driver/ZeroResponse")
(setf (get netft_rdt_driver::ZeroResponse :definition-)
      "---


")



(provide :netft_rdt_driver/Zero "d41d8cd98f00b204e9800998ecf8427e")


