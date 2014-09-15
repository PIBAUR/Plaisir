
(cl:in-package :asdf)

(defsystem "test_suivi-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "SendPose" :depends-on ("_package_SendPose"))
    (:file "_package_SendPose" :depends-on ("_package"))
  ))