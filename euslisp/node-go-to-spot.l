#!/usr/bin/env roseus

(load "package://pr2eus/pr2-interface.l")

(ros::load-ros-manifest "move_base_msgs")
(ros::load-ros-manifest "pr2_guide")

(ros::roseus "go-to-spot")

(defclass go-to-spot-server
    :slots (
            (frame-id nil)
            (target-spot nil)
            topicname-movebase-cancel
            servicename-go-to-spot-start
            servicename-go-to-spot-cancel
            servicename-spot-manager-get
            )
    )

(defmethod go-to-spot-server
    (:init ()
        ;;
        (setq *ri* (instance pr2-interface :init))
        ;;
        (setq topicname-movebase-cancel             (ros::get-param "/pr2_guide/topicname/move_base/cancel"            "/move_base/cancel"))
        (setq topicname-movebase-result             (ros::get-param "/pr2_guide/topicname/move_base/result"            "/move_base/result"))
        (setq servicename-go-to-spot-start          (ros::get-param "/pr2_guide/servicename/go_to_spot/start"          "/go_to_spot/start"))
        (setq servicename-go-to-spot-cancel         (ros::get-param "/pr2_guide/servicename/go_to_spot/cancel"         "/go_to_spot/cancel"))
        (setq servicename-spot-manager-get          (ros::get-param "/pr2_guide/servicename/spot_manager/get"          "/spot_manager/get"))
        (setq servicename-status-manager-set-status (ros::get-param "/pr2_guide/servicename/status_manager/set_status" "/status_manager/set_status"))
        ;;
        (ros::advertise-service servicename-go-to-spot-start  pr2_guide::GoToSpotStart  #'send self :service-cb-start)
        (ros::advertise-service servicename-go-to-spot-cancel pr2_guide::GoToSpotCancel #'send self :service-cb-stop)
        (ros::advertise topicname-movebase-cancel actionlib_msgs::GoalID 1)
        (ros::subscribe topicname-movebase-result move_base_msgs::MoveBaseActionResult #'send self :subscriber-cb-checking-goal)
        ;;
        (ros::ros-info "initialization finished")
        )

    (:subscriber-cb-checking-goal (msg)
        (ros::ros-info "checking goal subscriber callback is called")
        (send self :set-status "waiting-interaction")
        (ros::ros-info "result text is ~A" (send (send msg :status) :text))
        )

    (:set-status (status)
        (ros::ros-info "set-status function is called")
        (let* ((req (instance pr2_guide::StatusManagerSetStatusRequest :init))
               ret
               )
            (send req :status status)
            (setq ret (send (ros::service-call servicename-status-manager-set-status req) :ret))
            )
        )

    (:service-cb-start (req)
        (ros::ros-info "start service is called")
        (let* ((get-request (instance pr2_guide::SpotManagerGetRequest :init))
               (spots nil)
               (res (send req :response))
               (goal-pose nil)
               )
            (setq target-spot (send req :target_spot))
            (setq spots (send (ros::service-call servicename-spot-manager-get get-request) :spots))
            (ros::ros-info "target spot is ~A" target-spot)
            (if (eq spots nil)
                (progn
                    (ros::ros-info "target spot has not been found.")
                    (return-from :service-cb-start)
                    )
                nil)

            (dolist (x spots)
                (if (equal (send x :name) target-spot)
                    (progn
                        (setq goal-pose (send x :pose))
                        (setq frame-id (send (send x :header) :frame_id))
                        (return))
                    nil)
                )
            (if (equal goal-pose nil)
                (progn
                    (ros::ros-info "go-to-spot-start service has failed.")
                    (send res :ret nil)
                    res
                    )
                (progn
                    (send (send goal-pose :position) :z 0)
                    (setq goal-pose (ros::tf-pose->coords goal-pose))
                    (send *ri* :move-to goal-pose :frame-id frame-id :no-wait t)
                    (ros::ros-info "move_base action has been successfully called with coordinate ~A." goal-pose)
                    (send res :ret t)
                    res
                    )
                )
            )
        )

    (:service-cb-stop (req)
        (ros::ros-info "go-to-spot-stop service is called")
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
