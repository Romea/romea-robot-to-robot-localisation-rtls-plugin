# 1 Overview #

This package is a RTLS plugin for leader robot localisation. It schedules ranging between the robot's rtls transceivers (called initiators) and the leader's rtls transceivers (called responders). It provides these measurements to the localization filter, as well as the leader's twist measurement broadcased during ranging process. Finally, it computes and send to localization filter a rough estimation of leader pose in using trilateration algorithm base on ranging results.  

# 2 Node #

### 2.1 Subscribed Topics ###

- For each initiator declared in parameters :

  - initiators_names[i]/payload: (romea_rtls_transceiver_msgs::msg::Payload)

    This topic is provided by rtls transceiver driver node called initiators_names[i]/driver, it contains serialized twist measurement data broadcasted during ranging process by leader robot. 

  - initiators_names[i]/range: (romea_rtls_transceiver_msgs::msg::RangingResult

    This topic is provided by rtls transceiver driver node called initiators_names[i]/driver, it contains ranging result beetween selected initiator and the leader responder

### 2.2 Published Topics ###

- leader_pose (romea_localisation_msgs::msg::ObservationPose2DStamped)

  Leader pose estimated by trilateration algorithm using ranging results

- range (romea_localisation_msgs::msg::ObservationRangeStamped)

  Ranging results between robot rtls tranceivers (called initiators) and leader rtls transceivers (called responders)

- leader_twist (romea_localisation_msgs/msg/ObservationTwist2DStamped)

  Leader twist displacement broadcasted by leader during ranging process 

- For each initiator declared in parameters :

  - initiators_names[i]/request: (romea_rtls_transceiver_msgs::msg::RangingRequest)

      This topic is sended to initiator driver node in order to start ranging with selected leader responder, if ranging succeeded then range result will be send to the plugin node 

### 2.3 Parameters ###

- enable_sheduler (bool, default: true)

    Enable ranging scheduling. The scheduler must be actived in live and simulated experiments and disables during replay.
    
- ~initiators_ids(vector<int>)

    Identifier of each rtls transceivers (called initiators) embedded on the robot

- ~initiators_names(vector<string>)

    Name of each rtls transceivers (called initiators) embedded on the robot, more precisely these names are the ros namespace of their driver nodes

- For each initiator declared previously :

  - ~initiators_positions.initiators_name[i] (vector<double>)

    Position of the antenna of the rtls transceiver embedded on the robot 

- ~responders_ids(vector<int>)

    Identifier of each rtls transceivers (called responders) embedded on leader robot

- ~responders_names(vector<string>)

    Name of the rtls transceivers (called responders) embedded on leader robot

- For each responder declared previously :

  - ~responders_positions.responders_names[i] (vector<double>)

    Position of the antenna of the rtls transceiver embedded on leader robot

- ~minimal_range (double, default: 0.5)

    Minimal available range distance   

- ~maximal_range (double, default: 20.)

    Maximal available range distance

- ~range_std (double, default: 0.02)

    Standard deviation on range measurement

- ~poll_rate (int, default: 20)

    Rate at which ranging requests are sent to initiators 

- ~base_footprint_frame_id (double, default: base_footprint)

    Name of robot base footprint

