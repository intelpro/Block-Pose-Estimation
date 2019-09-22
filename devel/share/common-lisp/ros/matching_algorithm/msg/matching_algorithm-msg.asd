
(cl:in-package :asdf)

(defsystem "matching_algorithm-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Data1d" :depends-on ("_package_Data1d"))
    (:file "_package_Data1d" :depends-on ("_package"))
    (:file "Data2d" :depends-on ("_package_Data2d"))
    (:file "_package_Data2d" :depends-on ("_package"))
  ))