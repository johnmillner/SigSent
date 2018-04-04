
(cl:in-package :asdf)

(defsystem "sigsent-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "GPSList" :depends-on ("_package_GPSList"))
    (:file "_package_GPSList" :depends-on ("_package"))
  ))