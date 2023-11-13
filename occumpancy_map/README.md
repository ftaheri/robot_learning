# pose.hh
pose.hh is a header file implementing a 2d position Pose2 by x,y and an angle. It also implements
add, minus and assignment operators to update the angle and normalize it to be in [−π, π].

# mapper.hh
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

# mapper.cpp
map_update_callback in mapper.cpp is called every second which first calls update_map_msg. The latter must
update map_msg_ using map_ array. Then, the callback function publishes this message to map_topic.
These updates will be made possible by updating map_ array. This is done by laser_topic calling
laser_callback when a laser message is received from the LRF subscriber node.


# Updating and publishing map_msg_
## implementing at() function:
I used return &map_[i* width_x_ + j] to return a reference to the cell at
i,j. That is the formula that calculates the cell location of the flattened version
based on the 2d one.

## implementing update_map_msg:
In the mapper.cpp file, based on the evd_occ_,evd_free_, I pushed
OCC,FREE,UNKN to map_msg_, as specified in the mapper.hh comments.
After uncommenting map_publisher_, the code publishes map_msg_. Since
map_ is yet to be updated, all the cells of map_msg_ are set to unknown. Now
we can see a green (grey) map of unknown cells.

## Updating map_
In this section, I will describe how I implemented laser_callback function:
Updating map using back-projection and transform between the laser frame and the
map frame:
Firstly, I looked up transform from odom to base link in the tf buffer. I also added one second delay,
since there was a time conflict between the latest transform and when the lookup was called. Secondly,
I updated map2sensor_ (declared in the mapper header file using Pose2 class) coordinates using the
transform and updated its theta using its quaternion. I converted transform.transform.rotation
to tf2::Quaternion and converted that to Euler’s yaw angle using the formula
quat.getAngle() * quat.getAxis().z(). quat.getAngle() returns the rotation angle of the quater-
nion and quat.getAxis().z() returns the rotation angle around z axis. Now, we can use this trans-
form to find the coordinates in the laser frame.
The code iterates through all the map_ cells representing a grid map. It first converts x,y of each
cell to their coordinates in the map frame assuming that map indices are in the same direction as the
map frame. This is done by changing coordinates from cell numbers to metric points and adding the
frame offset to them.
```
double cell_x = x * map_resolution_ + map_msg_.info.origin.position.x;
double cell_y = y * map_resolution_ + map_msg_.info.origin.position.y;
```
Then, I calculated x,y based on the laser frame using the transform from previous steps:
```
double laser_x = map2sensor_.x + cell_x * std::cos(map2sensor_.theta) -
cell_y * std::sin(map2sensor_.theta);
double laser_y = map2sensor_.y + cell_x * std::sin(map2sensor_.theta) +
cell_y * std::cos(map2sensor_.theta);
```

Since the map grid’s indices relative to the map frame are flipped, the cell’s y is calculated by x’s
negative value while its x is calculated by y’s value.
```
std::swap(laser_x,laser_y);
laser_y = -laser_y;
```
Now, having x,y in the laser frame, we can easily calculate the cell’s distance to the laser.
$$range = \sqrt{laser_x^2 + laser_y^2}$$

Now comparing this range to `scan.ranges[ray_index]`, we can see if the cell is between an object
and the laser, the obstacle itself, or behind an obstacle.
I calculated the ran index by the code bellow.
```
double angle = std::atan2f(laser_y, laser_x) - map2sensor_.theta;
int ray_index = std::round((angle - scan.angle_min) / scan.angle_increment);
```
Then, if `range-scan.ranges[ray_index]` is less than -map_resolution, then the cell is free, so code
increments the cell using `*at(y,x)`. If `range-scan.ranges[ray_index]` is more than `-map_resolution`
and less than `map_resolution`, then code decrements the cell. Otherwise, the cell is behind an object,
3so the cell is set to 0. I chose `map_resolution` for the difference between cell and laser range, so it
could capture the uncertainty and assumes a cell to be occupied if laser range is in the cell. I also could
have used the maximum distance of the cell’s point to the laser range point by using $\sqrt{2} ∗ resolution$.
However, it is not that different in general.
I also added a threshold of 100, so that the `map_` is not increased or decreased until overflows.
Verifying the code works for a stationary sensor:
To verify that the code works, the first image is what the map shows using a stationary sensor and
the second one is what the same sensor shows after adding another object by gazebo. <br>
![image](https://github.com/ftaheri/robot_learning/assets/44457498/4fa8d86e-a67e-4b97-8e34-71bcdcd9197a)

Verifying the code works for a non-stationary sensor:
To verify that the mapper also works for a non-stationary sensor (using the transform lookup), the
first image shows the map before undocking the robot and the second image shows it after undocking. <br>
![image](https://github.com/ftaheri/robot_learning/assets/44457498/47a2cd65-7491-4fa5-9ae6-acfcec2ee32f)
