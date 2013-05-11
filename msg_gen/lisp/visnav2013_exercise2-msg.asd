
(cl:in-package :asdf)

(defsystem "visnav2013_exercise2-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Tag" :depends-on ("_package_Tag"))
    (:file "_package_Tag" :depends-on ("_package"))
    (:file "Navdata" :depends-on ("_package_Navdata"))
    (:file "_package_Navdata" :depends-on ("_package"))
    (:file "Tags" :depends-on ("_package_Tags"))
    (:file "_package_Tags" :depends-on ("_package"))
  ))