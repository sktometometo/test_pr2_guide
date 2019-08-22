#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")

(load "package://pr2eus/pr2-interface.l")

(setq *ri* (instance pr2-interface :init))
(setq *pr2* (instance pr2-robot :init))

(defun str2symbol (str)
    (intern (string-upcase str) "KEYWORD"))

(defun go-to-spot (spotname)
    (let ((goal-pose nil)
          (frame-id nil)
          (spots nil))
        (setq spots (one-shot))

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

(defclass pr2-guide-status-based-action
    :slots ((status "default")
            (arg-str-list nil)
            (target-spotname nil)
            (actionhandler-roopcount)
            (dict-status-and-action (make-hash-table)) 
            )
    )

(defmethod pr2-guide-status-based-action
    (:init ()
        ;; setup hash table
        (sethash :default-case             (send self :get-val `dict-status-and-action) #`send self :actionhandler-default-case)
        (sethash :waiting                  (send self :get-val `dict-status-and-action) #`send self :actionhandler-waiting)
        (sethash :waiting-interaction      (send self :get-val `dict-status-and-action) #`send self :actionhandler-waiting-interaction)
        (sethash :guiding                  (send self :get-val `dict-status-and-action) #`send self :actionhandler-guiding)
        (sethash :guiding-halt-waiting     (send self :get-val `dict-status-and-action) #`send self :actionhandler-guiding-halt-waiting)
        (sethash :guiding-halt-interaction (send self :get-val `dict-status-and-action) #`send self :actionhandler-guiding-halt-interaction)

        ;; for ROS
        (ros::roseus "/pr2_guide/status_based_action_server" )
        (ros::advertise-service "/pr2_guide/status_based_action_server/set_action_status" roseus::StatusBasedActionServerSetStatus #`:send self :servicehandler-set-action-status)
        (ros::advertise-service "/pr2_guide/status_based_action_server/get_action_status" roseus::StatusBasedActionServerGetStatus #`:send self :servicehandler-get-action-status)
        )

    (:status ((status nil))
        (if (eq status nil)
             (send self :get-val `status)
             (send self :set-val `status status)))

    (:servicehandler-set-action-status (req)
        (let* ((ret (send req :response)) (status (send req :status)) (arg-str-list (send req :args)))
            (if (send self :set-action-status status arg-str-list)
                (progn
                  (send ret :ret t)
                  ret)
                (progn
                  (send ret :ret nil)
                  ret)
                )
            )
        )

    (:servicehandler-get-action-status (req)
        (let* ((ret (send req :response)))
            (send ret :status (send self :status))
            ret
            )
        )

    (:set-action-status (status &optional (arg-str-list nil))
        (if (< 0 (count (send (send self :get-val `dict-status-and-action) :list-keys) (str2symbol status)))
            (progn
                (send self :set-val `status status)
                (send self :set-val `arg-str-list arg-str-list)
                t)
            nil)
        )

    (:get-action-status ()
        (send self :get-val `status)
        )

    (:actionhandler-default-case ()
        (send self :set-action-status "waiting")
        )

    (:actionhandler-waiting ()
        )

    (:actionhandler-waiting-interaction ()
        )

    (:actionhandler-guiding ()
        (cond 
            ((= (send self :get-action-status) "waiting-interaction")
                (send self :set-val `target-spotname (elt (send self :get-val `arg-str-list) 0))
                (go-to-spot (send self :get-val `target-spotname))
                )
            ((= (send self :get-action-status) "guiding-halt-interaction")
                (go-to-spot (elt (send self :get-val `arg-str-list) 0))
                )
            (t
                )
            )
        )

    (:abort-action ()
        )

    )

(defun main()
   (ros::spin)
   )

(main)
