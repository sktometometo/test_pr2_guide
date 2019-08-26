#!/bin/env roseus

(load "package://pr2eus/pr2-interface.l")

(defclass go-to-spot-server
    :slots ((goal-pose nil)
            (frame_id nil)
            (target-spot nil))
    )

(defmethod go-to-spot-server
    (:init ()
        (setq *ri* (instance pr2-interface :init))
        (ros::roseus "go-to-spot")
        (ros::advertise-service "/go_to_spot_server/start_go_to_spot"  roseus::GoToSpotServiceStart  #'send self :service-cb-start)
        (ros::advertise-service "/go_to_spot_server/cancel_go_to_spot" roseus::GoToSpotServiceCancel #'send self :service-cb-stop)
        (ros::advertise "/move_base/cancel" actionlib_msgs::GoalID 1)
        )

    (:service-cb-start (req)
        (let * ((spots-request (instance ros::SpotServer_GetSpots nil))
                (spots nil)
                (res (send req :response))
                )
            (setq target-spot (send req :target_spot))
            (setq spots (send (send (ros::service-call "/spotserver/get_spots" spots-request) :ret) :data))

            (dolist (x spots)
                (if (equal (send x :text) spotname)
                    (progn
                        (setq goal-pose (send x :pose))
                        (setq frame-id (send (send x :header) :frame_id))
                        (return))
                    nil)
                )
            (if (equal goal-pose nil)
                (progn
                    (send res :ret nil)
                    res
                    )
                (progn
                    (send (send goal-pose :position) :z 0)
                    (setq goal-pose (ros::tf-pose->coords goal-pose))
                    (send *ri* :move-to goal-pose :frame-id frame-id)
                    (send res :ret t)
                    res
                    )
                )
            )
        )

    (:service-cb-stop (req)
        (let* ((msg (instance actionlib_msgs::GoalID :init))
               (res (send req :response))
               )
            (ros::publish "/move_base/cancel" msg)
            (send res :ret t)
            res
            )
        )
    )

(defun main ()
  (setq gtss (instance go-to-spot-server :init))
  (ros::spin)
  )

(main)
