
(cl:in-package :asdf)

(defsystem "cc_fabmap-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "keyframeGraphMsgStamped" :depends-on ("_package_keyframeGraphMsgStamped"))
    (:file "_package_keyframeGraphMsgStamped" :depends-on ("_package"))
    (:file "keyframeMsgStamped" :depends-on ("_package_keyframeMsgStamped"))
    (:file "_package_keyframeMsgStamped" :depends-on ("_package"))
    (:file "keyframeMsg" :depends-on ("_package_keyframeMsg"))
    (:file "_package_keyframeMsg" :depends-on ("_package"))
    (:file "keyframeGraphMsg" :depends-on ("_package_keyframeGraphMsg"))
    (:file "_package_keyframeGraphMsg" :depends-on ("_package"))
    (:file "similarityTransformStamped" :depends-on ("_package_similarityTransformStamped"))
    (:file "_package_similarityTransformStamped" :depends-on ("_package"))
  ))