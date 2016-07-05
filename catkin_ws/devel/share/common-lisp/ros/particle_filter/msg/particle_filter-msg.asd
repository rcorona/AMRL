
(cl:in-package :asdf)

(defsystem "particle_filter-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Pose" :depends-on ("_package_Pose"))
    (:file "_package_Pose" :depends-on ("_package"))
    (:file "Particle" :depends-on ("_package_Particle"))
    (:file "_package_Particle" :depends-on ("_package"))
    (:file "Particle_vector" :depends-on ("_package_Particle_vector"))
    (:file "_package_Particle_vector" :depends-on ("_package"))
  ))