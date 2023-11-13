pose.hh is a header file implementing a 2d position Pose2 by x,y and an angle. It also implements
add, minus and assignment operators to update the angle and normalize it to be in [−π, π].

The mapper header file includes the pose2 and necessary packages for making nodes and listening
to transforms. In the file, using create_wall_timer a timer is set to call map_update_call_back
function every second. This function updates and publishes map_msg_ using the publisher node spec-
ified in the file by map_topic. A laser_subscriber node is declared to subscribe to laser_topic
which calls laser_callback when receives laser messages. This function updates map_ which will be
used to update map_msg_.
In addition, a tf listener and buffer is defined to listen to the transform buffers containing transform
trees. This will be used in the mapper.cpp file.
map_ is implemented by a 1d array representing a 2d map which will be used to update an Occupancy-
Grid map, map_msg_. Their origin is set in a way that map_frame origin is in the center of map grid.

map_update_callback in mapper.cpp is called every second which first calls update_map_msg. The latter must
update map_msg_ using map_ array. Then, the callback function publishes this message to map_topic.
These updates will be made possible by updating map_ array. This is done by laser_topic calling
laser_callback when a laser message is received from the LRF subscriber node.
