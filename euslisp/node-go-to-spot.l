#!/bin/env roseus

(load "package://pr2eus/pr2-interface.l")

(defclass go-to-spot-server
    :slots ((goal-pose nil)
            (frame-id nil)
            (target-spot nil)
            (nodename "go-to-spot")
            topicname-movebase-cancel
            servicename-go-to-spot-start
            servicename-go-to-spot-cancel
            servicename-spot-manager-get
            )
    )

(defmethod go-to-spot-server
    (:init ()
        ;;
        (ros::roseus nodename)
        (setq *ri* (instance pr2-interface :init))
        ;;
        (setq topicname-movebase-cancel             (ros::get-param "/pr2_guide/topicname/move_base/cancel"            "/move_base/cancel"))
        (
        (setq servicename-go-to-spot-start          (ros::get-param "/pr2_guide/servicename/go_to_spot/start"          "/go_to_spot/start"))
        (setq servicename-go-to-spot-cancel         (ros::get-param "/pr2_guide/servicename/go_to_spot/cancel"         "/go_to_spot/cancel"))
        (setq servicename-spot-manager-get          (ros::get-param "/pr2_guide/servicename/spot_manager/get"          "/spot_manager/get"))
        (setq servicename-status-manager-set-status (ros::get-param "/pr2_guide/servicename/status_manager/set_status" "/status_manager/set_status"))
        ;;
        (ros::advertise-service servicename-go-to-spot-start  pr2_guide::GoToSpotStart  #'send self :service-cb-start)
        (ros::advertise-service servicename-go-to-spot-cancel pr2_guide::GoToSpotCancel #'send self :service-cb-stop)
        (ros::advertise topicname-movebase-cancel actionlib_msgs::GoalID 1)
        )

    (:set-status (status)
        (let* ((req (instance pr2_guide::StatusManagerSetStatusRequest :init status))
               ret
               )
            (setq ret (send (ros::service-call servicename-status-manager-set-status req) :ret))
            )
        )

    (:

    (:service-cb-start (req)
        (let* ((get-request (instance pr2_guide::SpotManagerGetRequest :init))
               (spots nil)
               (res (send req :response))
               )
            (setq target-spot (send req :target_spot))
            (setq spots (send (ros::service-call servicename-spot-manager-get get-request) :spots))

            (dolist (x spots)
                (if (equal (send x :text) spotname)
                    (progn
                        (setq goal-pose (send x :pose))
                        (setq frame-id (send (send x :header) :frame-id))
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
                    (send *ri* :move-to goal-pose :frame-id frame-id :no-wait t)
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
