#!/bin/env roseus

(load "package://pr2eus/pr2-interface.l")

(defclass go-to-spot-server
    :slots ((goal-pose nil)
            (frame_id nil)
            (target-spot nil)
            (nodename "go-to-spot")
            (topicname-movebase-cancel "/move_base/cancel")
            (servicename-go-to-spot-start "/go_to_spot/start")
            (servicename-go-to-spot-cancel "/go_to_spot/start")
            (servicename-spot-manager-get "/spot_manager/get")
            )
    )

(defmethod go-to-spot-server
    (:init ()
        (setq *ri* (instance pr2-interface :init))
        (ros::roseus nodename)
        (ros::advertise-service servicename-go-to-spot-start  pr2_guide::GoToSpotStart  #'send self :service-cb-start)
        (ros::advertise-service servicename-go-to-spot-cancel pr2_guide::GoToSpotCancel #'send self :service-cb-stop)
        (ros::advertise topicname-movebase-cancel actionlib_msgs::GoalID 1)
        )

    (:service-cb-start (req)
        (let * ((get-request (instance pr2_guide::SpotManagerGetRequest nil))
                (spots nil)
                (res (send req :response))
                )
            (setq target-spot (send req :target_spot))
            (setq spots (send (ros::service-call servicename-spot-manager-get get-request) :spots))

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
            (ros::publish topicname-movebase-cancel msg)
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
