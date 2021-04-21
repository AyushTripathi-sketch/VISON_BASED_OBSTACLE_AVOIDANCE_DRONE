# VISON_BASED_OBSTACLE_AVOIDANCE_DRONE
### PROBLEM STATEMENT:
    The task is to design an autonomous drone that navigates in a complex static environment by avoiding
    any collision with the on-field obstacles and reaching the target destination after its correct detection.

    ->Design an algorithm that autonomously navigates a drone from point A to point B avoiding collisions
    with the obstacles present in the path.

    ->The local coordinates of Point A would be known before-hand and the drone has to detect and
    navigate up to Point B ( that would be an Aruco Marker).

    ->The task is considered to be completed if the drone lands on Point B Aruco Marker) without any
    crash. The ROS package must publish the ID of the Aruco marker and string “Landed” after landing
    on to topic “/Aruco/message”
    a. When not detected: “Marker ID: none, looking for marker”
    b. When detected and landed: “ Marker ID : 0, Landed”

    ->Some parts of the Aruco marker must be visible to the RGB camera upon landing.

    ->Multiple Aruco Markers (false) may or may not be provided. The drone has to correctly identify the
    Aruco Marker based on the ID provided before landing. Correct Aruco ID will be ‘0’ in all the world.

    ->The drone model will be provided with a forward-facing depth camera and downward-facing RGB
    camera only (Any other sensors cannot be used).

    ->The flight should be strictly restricted to a height of 5m only.

### SOFTWARE SPECIFICATIONS:
    Ubuntu 18.04
    ROS Melodic
    Gazebo 9
    Ardupilot Firmware
    Python or C++
    Drone kit
    
    Installation instruction are given in the file 'Installation.md'
    Start instructions are given in the file 'Starting_Steps.md'
