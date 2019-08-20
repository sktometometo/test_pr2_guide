#!/bin/env roseus

(load "package://pr2eus/pr2-interface.l")

(setq *ri* (instance pr2-interface :init))
(ros::roseus "go-to-spot")

(defun go-to-spot-cb (msg)
  (print "cb called")
  (let ((name (send msg :data))
        (goal-pose nil)
        (frame-id nil)
        (spots nil))
    (print "subscribing /spots_marker_array....")
    (setq spots (one-shot-subscribe "/kinetic/spots_marker_array" visualization_msgs::MarkerArray))
    (print "finished.")
    (dolist (x (send spots :markers))
      (if (equal (send x :text) name)
        (progn
          (setq goal-pose (send x :pose))
          (setq frame-id (send (send x :header) :frame_id))
          (return)
        )
      )
    )
    (if (equal goal-pose nil)
      (progn
        (print "can not find spot")
        (return)
      )
      (progn
        (print "spot found. move")
        (send (send goal-pose :position) :z 0)
        (setq goal-pose (ros::tf-pose->coords goal-pose))
        (send *ri* :move-to goal-pose :frame-id frame-id)
      )
    )
  )
)

(defun main ()
  (ros::subscribe "/go_to_spot" std_msgs::string #'go-to-spot-cb)
  (ros::spin)
  )

(main)
