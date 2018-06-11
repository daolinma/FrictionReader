
(cl:in-package :asdf)

(defsystem "netft_rdt_driver-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Zero" :depends-on ("_package_Zero"))
    (:file "_package_Zero" :depends-on ("_package"))
  ))