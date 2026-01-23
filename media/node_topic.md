flowchart LR
    T_SCAN["/scan"] --> CARTO(["cartographer_node"]) & RVIZ(["rviz2"])
    T_ODOM["/odom"] --> CARTO
    CARTO --> T_SUBMAP["/submap_list"] & T_TF["/tf"]
    OCC_GRID(["cartographer_occupancy_grid_node"]) --> T_MAP["/map"]
    T_MAP --> RVIZ
    T_SUBMAP --> RVIZ
    T_TF --> RVIZ
    T_TF_STATIC["/tf_static"] --> RVIZ
