
(cl:in-package :asdf)

(defsystem "limo_deeplearning-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "identify" :depends-on ("_package_identify"))
    (:file "_package_identify" :depends-on ("_package"))
    (:file "img" :depends-on ("_package_img"))
    (:file "_package_img" :depends-on ("_package"))
    (:file "light" :depends-on ("_package_light"))
    (:file "_package_light" :depends-on ("_package"))
    (:file "traffic" :depends-on ("_package_traffic"))
    (:file "_package_traffic" :depends-on ("_package"))
  ))