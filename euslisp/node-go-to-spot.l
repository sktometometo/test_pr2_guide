#!/bin/env roseus

(load "package://pr2eus/pr2-interface.l")

(setq *ri* (instance pr2-interface :init))
(ros::roseus "go-to-spot")

(defun go-to-spot-service (req)
    (let* ((goal-pose nil)
           (frame-id nil)
           (target-spot (send req :target_spot))
           (spots-request (instance ros::SpotServer_GetSpots :init))
           (spots nil))
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
                nil
                )
            (progn
                (send (send goal-pose :position) :z 0)
                (setq goal-pose (ros::tf-pose->coords goal-pose))
                (send *ri* :move-to goal-pose :frame-id frame-id)
                t
                )
            )
        )
    )

(defun main ()
  (ros::advertise-service "/go_to_spot_server/go_to_spot" roseus::GoToSpotService #'go-to-spot-service)
  (ros::spin)
  )

(main)
