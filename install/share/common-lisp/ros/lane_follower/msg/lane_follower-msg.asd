
(cl:in-package :asdf)

(defsystem "lane_follower-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "StatusReport" :depends-on ("_package_StatusReport"))
    (:file "_package_StatusReport" :depends-on ("_package"))
    (:file "VeronicaStatusReport" :depends-on ("_package_VeronicaStatusReport"))
    (:file "_package_VeronicaStatusReport" :depends-on ("_package"))
  ))