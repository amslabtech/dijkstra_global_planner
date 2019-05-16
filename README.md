# dijkstra_global_planner

[![Build Status](https://travis-ci.org/amslabtech/dijkstra_global_planner.svg?branch=master)](https://travis-ci.org/amslabtech/dijkstra_global_planner)
![issue_opened](https://img.shields.io/github/issues/amslabtech/dijkstra_global_planner.svg)
![issue_closed](https://img.shields.io/github/issues-closed/amslabtech/dijkstra_global_planner.svg)

## Enviornment
- Ubuntu 16.04 or 18.04
- ROS Kinetic or Melodic

## Dependencies
- amsl_navigation_msgs

## Published topics
- /global_path/path (std_msgs/Int32MultiArray)
- /global_path/path/viz (visualization_msgs/MarkerArray)

## Subscribed topics
- /node_edge_map/map (amsl_navigation_msgs/NodeEdgeMap)
- /node_edge_map/checkpoint (std_msgs/Int32MultiArray)
- /estimated_pose/edge (amsl_navigation_msgs/Edge)

## Services
- /global_path/replan (amsl_navigation_msgs/Replan)

## Parameters
- init_node0_id
  - node id of the begin of initial edge (default: 0)
- init_node1_id
  - node id of the end of initial edge (default: 1)
- init_progress
  - initial progress on initial edge (default: 0.0)
