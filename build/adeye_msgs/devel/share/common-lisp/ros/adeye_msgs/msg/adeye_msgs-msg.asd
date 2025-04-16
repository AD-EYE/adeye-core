
(cl:in-package :asdf)

(defsystem "adeye_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "categorized_pose" :depends-on ("_package_categorized_pose"))
    (:file "_package_categorized_pose" :depends-on ("_package"))
    (:file "categorized_poses" :depends-on ("_package_categorized_poses"))
    (:file "_package_categorized_poses" :depends-on ("_package"))
  ))