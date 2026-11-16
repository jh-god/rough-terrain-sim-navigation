Panels:
- Class: rviz_common/Displays
  Name: Displays
- Class: nav2_rviz_plugins/Navigation 2
  Name: Navigation 2
Visualization Manager:
  Class: ''
  Global Options:
    Fixed Frame: map
    Frame Rate: 20
  Displays:
  - Class: rviz_default_plugins/Marker
    Name: Aerial terrain background
    Enabled: true
    Topic:
      Value: /navigation/aerial_background
      Durability Policy: Transient Local
      Reliability Policy: Reliable
    Namespaces: {}
  - Class: rviz_default_plugins/Grid
    Name: Grid
    Enabled: false
    Cell Size: 1
  - Class: rviz_default_plugins/Map
    Name: Empty map
    Enabled: false
    Topic:
      Value: /map
      Durability Policy: Transient Local
  - Class: rviz_default_plugins/Map
    Name: Global costmap
    Enabled: true
    Alpha: 0.55
    Draw Behind: false
    Color Scheme: costmap
    Topic:
      Value: /global_costmap/costmap
      Durability Policy: Transient Local
      Reliability Policy: Reliable
  - Class: rviz_default_plugins/Map
    Name: Local costmap
    Enabled: true
    Alpha: 0.85
    Color Scheme: costmap
    Topic:
      Value: /local_costmap/costmap
      Durability Policy: Transient Local
      Reliability Policy: Reliable
    Draw Behind: false
  - Class: rviz_default_plugins/MarkerArray
    Name: Waypoints
    Enabled: true
    Topic:
      Value: /waypoints
      Durability Policy: Volatile
      Reliability Policy: Reliable
    Namespaces: {}
  - Class: rviz_default_plugins/Path
    Name: Global path
    Enabled: true
    Topic:
      Value: /plan
  - Class: rviz_default_plugins/RobotModel
    Name: Robot
    Enabled: true
    Description Source: Topic
    Description Topic:
      Value: /robot_description
      Durability Policy: Transient Local
  - Class: rviz_default_plugins/PointCloud2
    Name: Ground
    Enabled: false
    Topic:
      Value: /navigation/ground
      Reliability Policy: Best Effort
    Size (m): 0.04
  - Class: rviz_default_plugins/PointCloud2
    Name: Nonground
    Enabled: true
    Topic:
      Value: /navigation/nonground
      Reliability Policy: Best Effort
    Size (m): 0.06
  Tools:
  - Class: rviz_default_plugins/Interact
  - Class: rviz_default_plugins/MoveCamera
  - Class: nav2_rviz_plugins/GoalTool
  Views:
    Current:
      Class: rviz_default_plugins/TopDownOrtho
      Angle: 0.0
      Scale: 14.0
      X: 0.0
      Y: 0.0
Window Geometry:
  Width: 1400
  Height: 900
