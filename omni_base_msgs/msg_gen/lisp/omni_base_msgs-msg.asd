
(cl:in-package :asdf)

(defsystem "omni_base_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "PowerState" :depends-on ("_package_PowerState"))
    (:file "_package_PowerState" :depends-on ("_package"))
  ))