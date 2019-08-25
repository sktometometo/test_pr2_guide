#!/usr/bin/env roseus

(ros::load-ros-manifest "roseus")

(load "package://pr2eus/pr2-interface.l")

(setq *ri* (instance pr2-interface :init))
(setq *pr2* (instance pr2-robot :init))

(defun str2symbol (str)
    (intern (string-upcase str) "KEYWORD"))


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
        (if (send self :call-interactionmodule-waiting)
            (send self :set-action-status "waiting-interaction")
            nil)
        )

    (:actionhandler-waiting-interaction ()
        (let* ((ret (send self :call-interactionmodule-waiting-interaction)))
            (cond ((ret "waiting")
                    (send self :set-action-status "waiting")
                    )
                  ((ret "waiting-interaction")
                    nil
                    )
                  ((ret "guiding")
                    (send self :set-action-status "guiding")
                    (send self :target-spotname hoge)
                    )
                  )
        )

    (:actionhandler-guiding ()
        (let* ()
            ;; saying something
            (go-to-spot (send self :get-val `target-spotname))
            )
        )

    (:abort-action ()
        )

    )

(defun main()
   (ros::spin)
   )

(main)
