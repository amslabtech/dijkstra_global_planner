# dijkstra_global_planner

[![Build Status](https://travis-ci.org/amslabtech/dijkstra_global_planner.svg?branch=master)](https://travis-ci.org/amslabtech/dijkstra_global_planner)

![issue_opened](https://img.shields.io/github/issues/amslabtech/dijkstra_global_planner.svg)
![issue_closed](https://img.shields.io/github/issues-closed/amslabtech/dijkstra_global_planner.svg)

## Requirements
- amsl_navigation_msgs

## Published topics
- /global_path (std_msgs/Int32MultiArray)

## Subscribed topics
- /node_edge_map/map (amsl_navigation_msgs/NodeEdgeMap)
- /node_edge_map/checkpoint (std_msgs/Int32MultiArray)
- /estimated_pose/edge (amsl_navigation_msgs/Edge)

## Services
- /global_path/replan (amsl_navigation_msgs/Replan)
