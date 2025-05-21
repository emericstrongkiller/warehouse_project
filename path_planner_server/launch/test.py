ser:~/ros2_ws$ ros2 launch path_planner_server pathplanner.launch.py use_sim_time:=True
[INFO] [launch]: All log files can be found below /home/user/.ros/log/2025-05-21-08-32-31-790682-1_xterm-10805
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [controller_server-1]: process started with pid [10806]
[INFO] [planner_server-2]: process started with pid [10808]
[INFO] [behavior_server-3]: process started with pid [10810]
[INFO] [bt_navigator-4]: process started with pid [10821]
[INFO] [rviz2-5]: process started with pid [10831]
[INFO] [lifecycle_manager-6]: process started with pid [10837]
[controller_server-1] [INFO] [1747816352.022585028] [controller_server]:
[controller_server-1]   controller_server lifecycle node launched.
[controller_server-1]   Waiting on external lifecycle transitions to activate
[controller_server-1]   See https://design.ros2.org/articles/node_lifecycle.html for moreinformation.
[controller_server-1] [INFO] [1747816352.100543028] [controller_server]: Creating controller server
[planner_server-2] [INFO] [1747816352.254700937] [planner_server]:
[planner_server-2]      planner_server lifecycle node launched.
[planner_server-2]      Waiting on external lifecycle transitions to activate
[planner_server-2]      See https://design.ros2.org/articles/node_lifecycle.html for moreinformation.
[planner_server-2] [INFO] [1747816352.263313339] [planner_server]: Creating
[rviz2-5] QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-user'
[controller_server-1] [INFO] [1747816352.269990039] [local_costmap.local_costmap]:
[controller_server-1]   local_costmap lifecycle node launched.
[controller_server-1]   Waiting on external lifecycle transitions to activate
[controller_server-1]   See https://design.ros2.org/articles/node_lifecycle.html for moreinformation.
[controller_server-1] [INFO] [1747816352.271340923] [local_costmap.local_costmap]: Creating Costmap
[planner_server-2] [INFO] [1747816352.335791803] [global_costmap.global_costmap]:
[planner_server-2]      global_costmap lifecycle node launched.
[planner_server-2]      Waiting on external lifecycle transitions to activate
[planner_server-2]      See https://design.ros2.org/articles/node_lifecycle.html for moreinformation.
[planner_server-2] [INFO] [1747816352.337434250] [global_costmap.global_costmap]: Creating Costmap
[behavior_server-3] [INFO] [1747816352.605799479] [recoveries_server]:
[behavior_server-3]     recoveries_server lifecycle node launched.
[behavior_server-3]     Waiting on external lifecycle transitions to activate
[behavior_server-3]     See https://design.ros2.org/articles/node_lifecycle.html for moreinformation.
[bt_navigator-4] [INFO] [1747816352.659605279] [bt_navigator]:
[bt_navigator-4]        bt_navigator lifecycle node launched.
[bt_navigator-4]        Waiting on external lifecycle transitions to activate
[bt_navigator-4]        See https://design.ros2.org/articles/node_lifecycle.html for moreinformation.
[bt_navigator-4] [INFO] [1747816352.660152078] [bt_navigator]: Creating
[lifecycle_manager-6] [INFO] [1747816352.697010507] [lifecycle_manager_pathplanner]: Creating
[lifecycle_manager-6] [INFO] [1747816352.717721040] [lifecycle_manager_pathplanner]: Creating and initializing lifecycle service clients
[rviz2-5] [INFO] [1747816352.835803806] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-5] [INFO] [1747816352.836099971] [rviz2]: OpenGl version: 4.5 (GLSL 4.5)
[rviz2-5] [INFO] [1747816352.863680994] [rviz2]: Stereo is NOT SUPPORTED
[rviz2-5] [INFO] [1747816353.217421059] [rviz2]: Trying to create a map of size 153 x 129using 1 swatches
[rviz2-5] [ERROR] [1747816353.230841580] [rviz2]: Vertex Program:rviz/glsl120/indexed_8bit_image.vert Fragment Program:rviz/glsl120/indexed_8bit_image.frag GLSL link result :
[rviz2-5] active samplers with a different type refer to the same texture image unit
[lifecycle_manager-6] [INFO] [1747816353.589424071] [lifecycle_manager_pathplanner]: Starting managed nodes bringup...
[lifecycle_manager-6] [INFO] [1747816353.589501300] [lifecycle_manager_pathplanner]: Configuring planner_server
[planner_server-2] [INFO] [1747816353.589893825] [planner_server]: Configuring
[planner_server-2] [INFO] [1747816353.589947743] [global_costmap.global_costmap]: Configuring
[planner_server-2] [INFO] [1747816353.604122390] [global_costmap.global_costmap]: Using plugin "static_layer"
[planner_server-2] [INFO] [1747816353.609854118] [global_costmap.global_costmap]: Subscribing to the map topic (/map) with transient local durability
[planner_server-2] [INFO] [1747816353.612314436] [global_costmap.global_costmap]: Initialized plugin "static_layer"
[planner_server-2] [INFO] [1747816353.612345994] [global_costmap.global_costmap]: Using plugin "obstacle_layer"
[planner_server-2] [INFO] [1747816353.616954778] [global_costmap.global_costmap]: Subscribed to Topics: scan
[planner_server-2] [INFO] [1747816353.632547777] [global_costmap.global_costmap]: Initialized plugin "obstacle_layer"
[planner_server-2] [INFO] [1747816353.632609287] [global_costmap.global_costmap]: Using plugin "inflation_layer"
[planner_server-2] [INFO] [1747816353.637101131] [global_costmap.global_costmap]: Initialized plugin "inflation_layer"
[planner_server-2] [INFO] [1747816353.657021698] [global_costmap.global_costmap]: StaticLayer: Resizing costmap to 153 X 129 at 0.050000 m/pix
[planner_server-2] [INFO] [1747816353.658357168] [planner_server]: Created global plannerplugin GridBased of type nav2_navfn_planner/NavfnPlanner
[planner_server-2] [INFO] [1747816353.658408460] [planner_server]: Configuring plugin GridBased of type NavfnPlanner
[planner_server-2] [INFO] [1747816353.659737836] [planner_server]: Planner Server has GridBased  planners available.
[lifecycle_manager-6] [INFO] [1747816353.676374582] [lifecycle_manager_pathplanner]: Configuring controller_server
[controller_server-1] [INFO] [1747816353.677161454] [controller_server]: Configuring controller interface
[controller_server-1] [INFO] [1747816353.677460036] [controller_server]: getting goal checker plugins..
[controller_server-1] [INFO] [1747816353.677609138] [controller_server]: Controller frequency set to 10.0000Hz
[controller_server-1] [INFO] [1747816353.677655786] [local_costmap.local_costmap]: Configuring
[controller_server-1] [INFO] [1747816353.692263918] [local_costmap.local_costmap]: Using plugin "voxel_layer"
[controller_server-1] [INFO] [1747816353.701545588] [local_costmap.local_costmap]: Subscribed to Topics: scan
[controller_server-1] [INFO] [1747816353.727929534] [local_costmap.local_costmap]: Initialized plugin "voxel_layer"
[controller_server-1] [INFO] [1747816353.727971231] [local_costmap.local_costmap]: Using plugin "inflation_layer"
[controller_server-1] [INFO] [1747816353.731389665] [local_costmap.local_costmap]: Initialized plugin "inflation_layer"
[controller_server-1] [INFO] [1747816353.753390292] [controller_server]: Created progress_checker : progress_checker of type nav2_controller::SimpleProgressChecker
[controller_server-1] [INFO] [1747816353.755382373] [controller_server]: Created goal checker : general_goal_checker of type nav2_controller::SimpleGoalChecker
[controller_server-1] [INFO] [1747816353.756844909] [controller_server]: Controller Server has general_goal_checker  goal checkers available.
[controller_server-1] [INFO] [1747816353.760037701] [controller_server]: Created controller : FollowPath of type dwb_core::DWBLocalPlanner
[controller_server-1] [INFO] [1747816353.763151371] [controller_server]: Setting transform_tolerance to 0.200000
[controller_server-1] [INFO] [1747816353.789431741] [controller_server]: Using critic "RotateToGoal" (dwb_critics::RotateToGoalCritic)
[controller_server-1] [INFO] [1747816353.792183124] [controller_server]: Critic plugin initialized
[controller_server-1] [INFO] [1747816353.792506269] [controller_server]: Using critic "Oscillation" (dwb_critics::OscillationCritic)
[controller_server-1] [INFO] [1747816353.794905659] [controller_server]: Critic plugin initialized
[controller_server-1] [INFO] [1747816353.795533334] [controller_server]: Using critic "ObstacleFootprint" (dwb_critics::ObstacleFootprintCritic)
[controller_server-1] [INFO] [1747816353.796655527] [controller_server]: Critic plugin initialized
[controller_server-1] [INFO] [1747816353.797180816] [controller_server]: Using critic "GoalAlign" (dwb_critics::GoalAlignCritic)
[controller_server-1] [INFO] [1747816353.799061775] [controller_server]: Critic plugin initialized
[controller_server-1] [INFO] [1747816353.799575454] [controller_server]: Using critic "PathAlign" (dwb_critics::PathAlignCritic)
[controller_server-1] [INFO] [1747816353.801122520] [controller_server]: Critic plugin initialized
[controller_server-1] [INFO] [1747816353.801876067] [controller_server]: Using critic "PathDist" (dwb_critics::PathDistCritic)
[controller_server-1] [INFO] [1747816353.803125925] [controller_server]: Critic plugin initialized
[controller_server-1] [INFO] [1747816353.803981329] [controller_server]: Using critic "GoalDist" (dwb_critics::GoalDistCritic)
[controller_server-1] [INFO] [1747816353.805153733] [controller_server]: Critic plugin initialized
[controller_server-1] [INFO] [1747816353.805192159] [controller_server]: Controller Server has FollowPath  controllers available.
[lifecycle_manager-6] [INFO] [1747816353.819734308] [lifecycle_manager_pathplanner]: Configuring recoveries_server
[behavior_server-3] [INFO] [1747816353.820080512] [recoveries_server]: Configuring
[behavior_server-3] [INFO] [1747816353.837539078] [recoveries_server]: Creating behavior plugin spin of type nav2_behaviors/Spin
[behavior_server-3] [INFO] [1747816353.839558975] [recoveries_server]: Configuring spin
[behavior_server-3] [INFO] [1747816353.854474930] [recoveries_server]: Creating behavior plugin backup of type nav2_behaviors/BackUp
[behavior_server-3] [INFO] [1747816353.855771212] [recoveries_server]: Configuring backup
[behavior_server-3] [INFO] [1747816353.865198919] [recoveries_server]: Creating behavior plugin drive_on_heading of type nav2_behaviors/DriveOnHeading
[behavior_server-3] [INFO] [1747816353.866489478] [recoveries_server]: Configuring drive_on_heading
[behavior_server-3] [INFO] [1747816353.882251408] [recoveries_server]: Creating behavior plugin wait of type nav2_behaviors/Wait
[behavior_server-3] [INFO] [1747816353.883184377] [recoveries_server]: Configuring wait
[lifecycle_manager-6] [INFO] [1747816353.893221344] [lifecycle_manager_pathplanner]: Configuring bt_navigator
[bt_navigator-4] [INFO] [1747816353.893630705] [bt_navigator]: Configuring
[lifecycle_manager-6] [INFO] [1747816354.046506706] [lifecycle_manager_pathplanner]: Activating planner_server
[planner_server-2] [INFO] [1747816354.046816093] [planner_server]: Activating
[planner_server-2] [INFO] [1747816354.046868637] [global_costmap.global_costmap]: Activating
[planner_server-2] [INFO] [1747816354.046888867] [global_costmap.global_costmap]: Checking transform
[planner_server-2] [INFO] [1747816354.046969998] [global_costmap.global_costmap]: Timed out waiting for transform from robot_base_footprint to map to become available, tf error: Lookup would require extrapolation into the past.  Requested time 3932.400000 but the earliest data is at time 3933.016000, when looking up transform from frame [robot_base_footprint] to frame [map]
[rviz2-5] [INFO] [1747816354.191419265] [rviz2]: Message Filter dropping message: frame 'robot_front_laser_base_link' at time 3932.016 for reason 'discarding message because the queue is full'
[rviz2-5] [INFO] [1747816354.223148081] [rviz2]: Message Filter dropping message: frame 'robot_front_laser_base_link' at time 3932.066 for reason 'discarding message because the queue is full'
[planner_server-2] [INFO] [1747816354.547016013] [global_costmap.global_costmap]: Timed out waiting for transform from robot_base_footprint to map to become available, tf error: Lookup would require extrapolation into the past.  Requested time 3932.800000 but the earliest data is at time 3933.016000, when looking up transform from frame [robot_base_footprint] to frame [map]
[planner_server-2] [INFO] [1747816355.047064092] [global_costmap.global_costmap]: start
[rviz2-5] [INFO] [1747816355.062132283] [rviz2]: Trying to create a map of size 153 x 129using 1 swatches
[planner_server-2] [INFO] [1747816355.098229989] [planner_server]: Activating plugin GridBased of type NavfnPlanner
[planner_server-2] [INFO] [1747816355.099600873] [planner_server]: Creating bond (planner_server) to lifecycle manager.
[lifecycle_manager-6] [INFO] [1747816355.209662443] [lifecycle_manager_pathplanner]: Server planner_server connected with bond.
[lifecycle_manager-6] [INFO] [1747816355.209719036] [lifecycle_manager_pathplanner]: Activating controller_server
[controller_server-1] [INFO] [1747816355.210020715] [controller_server]: Activating
[controller_server-1] [INFO] [1747816355.210083106] [local_costmap.local_costmap]: Activating
[controller_server-1] [INFO] [1747816355.210100384] [local_costmap.local_costmap]: Checking transform
[controller_server-1] [INFO] [1747816355.210178205] [local_costmap.local_costmap]: start
[controller_server-1] [INFO] [1747816355.411239030] [controller_server]: Creating bond (controller_server) to lifecycle manager.
[rviz2-5] [INFO] [1747816355.460882536] [rviz2]: Trying to create a map of size 20 x 20 using 1 swatches
[lifecycle_manager-6] [INFO] [1747816355.520206716] [lifecycle_manager_pathplanner]: Server controller_server connected with bond.
[lifecycle_manager-6] [INFO] [1747816355.520266723] [lifecycle_manager_pathplanner]: Activating recoveries_server
[behavior_server-3] [INFO] [1747816355.521175316] [recoveries_server]: Activating
[behavior_server-3] [INFO] [1747816355.521218220] [recoveries_server]: Activating spin
[behavior_server-3] [INFO] [1747816355.521238927] [recoveries_server]: Activating backup
[behavior_server-3] [INFO] [1747816355.521256642] [recoveries_server]: Activating drive_on_heading
[behavior_server-3] [INFO] [1747816355.521273496] [recoveries_server]: Activating wait
[behavior_server-3] [INFO] [1747816355.521295167] [recoveries_server]: Creating bond (recoveries_server) to lifecycle manager.
[lifecycle_manager-6] [INFO] [1747816355.636244873] [lifecycle_manager_pathplanner]: Server recoveries_server connected with bond.
[lifecycle_manager-6] [INFO] [1747816355.636307067] [lifecycle_manager_pathplanner]: Activating bt_navigator
[bt_navigator-4] [INFO] [1747816355.638680620] [bt_navigator]: Activating
[bt_navigator-4] [INFO] [1747816355.734325975] [bt_navigator]: Creating bond (bt_navigator) to lifecycle manager.
[lifecycle_manager-6] [INFO] [1747816355.839720908] [lifecycle_manager_pathplanner]: Server bt_navigator connected with bond.
[lifecycle_manager-6] [INFO] [1747816355.839788846] [lifecycle_manager_pathplanner]: Managed nodes are active
[lifecycle_manager-6] [INFO] [1747816355.839810823] [lifecycle_manager_pathplanner]: Creating bond timer...
^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
[lifecycle_manager-6] [INFO] [1747816358.647190669] [rclcpp]: signal_handler(signum=2)
[lifecycle_manager-6] [INFO] [1747816358.647290060] [lifecycle_manager_pathplanner]: Running Nav2 LifecycleManager rcl preshutdown (lifecycle_manager_pathplanner)
[lifecycle_manager-6] [INFO] [1747816358.647342957] [lifecycle_manager_pathplanner]: Terminating bond timer...
[rviz2-5] [INFO] [1747816358.647201592] [rclcpp]: signal_handler(signum=2)
[behavior_server-3] [INFO] [1747816358.647204351] [rclcpp]: signal_handler(signum=2)
[planner_server-2] [INFO] [1747816358.647250359] [rclcpp]: signal_handler(signum=2)
[planner_server-2] [INFO] [1747816358.647413614] [global_costmap.global_costmap]: RunningNav2 LifecycleNode rcl preshutdown (global_costmap)
[planner_server-2] [INFO] [1747816358.647485372] [global_costmap.global_costmap]: Deactivating
[controller_server-1] [INFO] [1747816358.647261626] [rclcpp]: signal_handler(signum=2)
[controller_server-1] [INFO] [1747816358.647376101] [local_costmap.local_costmap]: Running Nav2 LifecycleNode rcl preshutdown (local_costmap)
[controller_server-1] [INFO] [1747816358.647455155] [local_costmap.local_costmap]: Deactivating
[bt_navigator-4] [INFO] [1747816358.647292915] [rclcpp]: signal_handler(signum=2)
[bt_navigator-4] [INFO] [1747816358.647524305] [bt_navigator]: Running Nav2 LifecycleNodercl preshutdown (bt_navigator)
[bt_navigator-4] [INFO] [1747816358.647590587] [bt_navigator]: Deactivating
[bt_navigator-4] [INFO] [1747816358.647613786] [bt_navigator]: Destroying bond (bt_navigator) to lifecycle manager.
[behavior_server-3] [INFO] [1747816358.647602168] [recoveries_server]: Running Nav2 LifecycleNode rcl preshutdown (recoveries_server)
[behavior_server-3] [INFO] [1747816358.647666528] [recoveries_server]: Deactivating
[behavior_server-3] [INFO] [1747816358.647696722] [recoveries_server]: Destroying bond (recoveries_server) to lifecycle manager.
[behavior_server-3] [INFO] [1747816358.657941535] [recoveries_server]: Cleaning up
[bt_navigator-4] [INFO] [1747816358.658433712] [bt_navigator]: Cleaning up
[behavior_server-3] [INFO] [1747816358.662362870] [recoveries_server]: Destroying bond (recoveries_server) to lifecycle manager.
[lifecycle_manager-6] [INFO] [1747816358.682703652] [lifecycle_manager_pathplanner]: Destroying lifecycle_manager_pathplanner
[behavior_server-3] [INFO] [1747816358.750920735] [recoveries_server]: Destroying
[controller_server-1] [INFO] [1747816358.810460663] [local_costmap.local_costmap]: Cleaning up
[controller_server-1] [INFO] [1747816358.815314916] [local_costmap.local_costmap]: Destroying bond (local_costmap) to lifecycle manager.
[controller_server-1] [INFO] [1747816358.815359738] [controller_server]: Running Nav2 LifecycleNode rcl preshutdown (controller_server)
[controller_server-1] [INFO] [1747816358.815389942] [controller_server]: Deactivating
[controller_server-1] [INFO] [1747816358.815457740] [controller_server]: Destroying bond (controller_server) to lifecycle manager.
[controller_server-1] [INFO] [1747816358.815556047] [controller_server]: Cleaning up
[INFO] [lifecycle_manager-6]: process has finished cleanly [pid 10837]
[bt_navigator-4] [INFO] [1747816358.875172955] [bt_navigator]: Completed Cleaning up
[bt_navigator-4] [INFO] [1747816358.875243355] [bt_navigator]: Destroying bond (bt_navigator) to lifecycle manager.
[bt_navigator-4] [INFO] [1747816358.890258892] [bt_navigator]: Destroying
[controller_server-1] [INFO] [1747816358.899788032] [controller_server]: Destroying bond (controller_server) to lifecycle manager.
[controller_server-1] [INFO] [1747816358.912792691] [local_costmap.local_costmap]: Destroying
[INFO] [behavior_server-3]: process has finished cleanly [pid 10810]
[ERROR] [rviz2-5]: process has died [pid 10831, exit code -11, cmd '/opt/ros/humble/lib/rviz2/rviz2 -d /home/user/ros2_ws/src/warehouse_project/path_planner_server/rviz/path_planner.rviz --ros-args -r __node:=rviz2 --params-file /tmp/launch_params_z_s3akt8'].
[controller_server-1] [INFO] [1747816358.956994600] [controller_server]: Destroying
[planner_server-2] [INFO] [1747816359.050235411] [global_costmap.global_costmap]: Cleaning up
[INFO] [bt_navigator-4]: process has finished cleanly [pid 10821]
[planner_server-2] [INFO] [1747816359.060629709] [global_costmap.global_costmap]: Destroying bond (global_costmap) to lifecycle manager.
[planner_server-2] [INFO] [1747816359.060685725] [planner_server]: Running Nav2 LifecycleNode rcl preshutdown (planner_server)
[planner_server-2] [INFO] [1747816359.060713505] [planner_server]: Deactivating
[planner_server-2] [INFO] [1747816359.060729082] [planner_server]: Deactivating plugin GridBased of type NavfnPlanner
[planner_server-2] [INFO] [1747816359.060739538] [planner_server]: Destroying bond (planner_server) to lifecycle manager.
[planner_server-2] [INFO] [1747816359.060823561] [planner_server]: Cleaning up
[planner_server-2] [INFO] [1747816359.078220909] [planner_server]: Cleaning up plugin GridBased of type NavfnPlanner
[planner_server-2] [INFO] [1747816359.079033543] [planner_server]: Destroying plugin GridBased of type NavfnPlanner
[planner_server-2] [INFO] [1747816359.088329048] [planner_server]: Destroying bond (planner_server) to lifecycle manager.
[planner_server-2] [INFO] [1747816359.093396927] [global_costmap.global_costmap]: Destroying
[INFO] [controller_server-1]: process has finished cleanly [pid 10806]
[planner_server-2] [INFO] [1747816359.113247185] [planner_server]: Destroying
[INFO] [planner_server-2]: process has finished cleanly [pid 10808]