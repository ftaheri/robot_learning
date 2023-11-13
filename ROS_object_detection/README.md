## Runnin image_subscriber.py as an isolated node on test images

For testing purposes, it was better to run the execution file ”obj_detector” from obj_detector pack-
age alone, instead of seeing the result on the launch file.
Here is an example output:<br>

<p float="left">
  <img src="https://github.com/ftaheri/robot_learning/assets/44457498/b272efa4-d45c-4bf8-8a9f-5a733b766168" width="65%" />
  <img src="https://github.com/ftaheri/robot_learning/assets/44457498/273f495a-23c0-477a-ac12-1968dc9f40d5" width="30%" /> 
</p>
As can bee seen, both objects are detected, labeled and scored. What’s more, the bounding box
information shows that the origin of the locations is top-left corner of the picture. We also should
beware that the data are in pixels which could be a problem if we don’t know the resolution.
I also tried other images to see how good the service works.

<p float="left">
  <img src="https://github.com/ftaheri/robot_learning/assets/44457498/11ee0e34-21f6-4bdd-ac75-832df2556324" width="30%" />
  <img src="https://github.com/ftaheri/robot_learning/assets/44457498/be0945d1-cd5e-4408-aa20-a39ccb297bc9" width="30%" /> 
  <img src="https://github.com/ftaheri/robot_learning/assets/44457498/a782d3ea-0eb6-47f6-adef-5d1993dbb639" width="30%" />
</p>


The model works pretty decent for cluttered images. From the second image we can see that the
classes expectedly don’t cover every type of object. However, the detection still works. I’ll explain the
third step in the following section.

# landmark_mapper.hh
I set up the publisher to publish to the ”waypoints” topic which I found on RVIZ ”add by topic”
window. For this, I had to include the <visualization_msgs/msg/marker_array.hpp> and also
modify packages.xml file to add this dependency and CMakeLists.txt to find this package and link it
to the execution file.
I also changed the robot.rviz setup file to add the waypoints marker, so I wouldn’t need to add it every
time I launched the file.
As I mentioned before I also created a vector of PointStamped as the landmark array which will be
used in the landmark mapper.cpp file.

# image callback function
To calculate the robot movement, we first look up the transform from base link to odom and then,
convert it to Eigen T. T is a matrix which is used to calculate points in the base_link frame to
odom frame; more specifically, $T ∗ current_baselink = odom$. last_pose_ is the previous T; therefore,
$last_pose_ ∗ previous_baselink = odom$.
 $$last_pose_^{-1}*T*current_baselink = last_pose_^{-1} ∗ odom = previous_baselink$$
Therefore, $last_pose_^{-1}*T$ transfers a point from current-baselink to previous_baselink which is the movement matrix.
Now, by listening to the translation and rotation of movement, we can copy and fix the input data,
and release the data_cv_ lock. Finally, we can update the last_pose_.

# update_callback() function
I iterated through the current array of landmarks. For each landmark, I created a message Point
object with coordination set to the landmark’s. I pushed this message to the marker sphere points
and pushed the marker to the marker array. Finally, I published the marker array.

# run() function
This function is the most important function in this file. It holds the logic of the code. As discussed
in the background section, this function’s thread waits until the movement is large enough, and then
sends a request to the object detection service and waits for a response which is basically the objects’
bounding boxes detected from the scene.
After clearing the landmarks array, we have to iterate over all the detections, and for each detec-
tion, extract the points in the detected area. We calculate the average of these points and transform
it to odom. This transform was set when the images were receied and the transform was looked up.
How to extract these points is the main challenge of this code.

## upper and lower bounds
Firstly, I needed to set the upper and lower bound coming from the bounding boxes. The problem
was that the bounding boxes were points in pixel starting from top-left corner, while point cloud
points were in meters and are based on the robot’s frame. (1) To fix the units, since I did not know the
resolution, I printed the bounding box min and maxes and compared them roughly to the non-NULL
points from the point cloud. What I witnessed was that each pixel is roughly 0.5 cm. Therefore, I
divided the thresholds by 200.0. (2) To fix the frames, I used this formula:
``` 
float y_lowerbound = (current_color_image.width/2.0 - detection.x_max)/200.0
, y_upperbound = (current_color_image.width/2.0 - detection.x_min)/200.0
, z_lowerbound = (current_color_image.height - detection.y_max)/200.0
, z_upperbound = (current_color_image.height - detection.y_min)/200.0;
```
These thresholds will be used to filter out point clouds that are not in these bounds. Please note that,
x-axis and y-axis from the images correspond to y-axis and z-axis from the point clouds, respectively.
** I took three approaches to get a better result **

## (approach 1) iterating and filter out with a deviation from the thresholds
As the instructions, using the if statement bellow, I filtered out other point clouds, and calcluated the
average.
```
if (*iter_y >= y_lowerbound-delta && *iter_y <= y_upperbound+delta &&
*iter_z >= z_lowerbound -delta && *iter_z <= z_upperbound +delta )
``` 
This method did not work well especially in the x-axis (depth) that points are not filtered by bounding
boxes.

## (approach 2) statistically filter out outliers then do (1)
Since points were not filtered by x-axis, I used an statistical outlier from pcl library. The results were
better, but sometimes the useful cluster of inputs were filtered out, causing the marker to become
empty which gave me an idea for the third approach.

## (approach 3) filter out on z and y axis based on thresholds then do (2) then (1)
Using the Passthrough filter, which basically does the same job as the first approach, I filtered out
points outside of the bounding boxes. However, I used the size of the bounding box as the deviation
from the detected bounding box, so smaller or farther bounding boxes have more concentrated point
clouds. Now we have our pionts more concentrated in and around the bounding box area
After that, I used the statistical outlier removal to remove outliers around the bounding box which
now works significantly better for x-axis (depth of objects).
Then I iterated through the filtered point clouds and used the same principle as the first approach.
Also noteworthy is that this is the best one which will be used for evaluation. The parameters (e.g.,
statistical outlier filter number of points and deviation) are adjusted experimentally.

# Evaluation
I included three screenshots for every different scenarios; image detection, RVIZ map, and the scene.
Please, note that the image detection is more reliable since there was a delay in keyboard instruction
and execution.
Generally, I found out that cluttered scenes and near the wall scenes are the most challenging ones
since it is difficult for the outlier removal to distinguish outliers from the useful points.

## one object - clear scene
As can be seen, the algorithm does a good job detecting the landmark.

<p float="left">
  <img src="https://github.com/ftaheri/robot_learning/assets/44457498/ce8f5744-2dc3-47ad-a5e9-7682f63ce196" width="30%" />
  <img src="https://github.com/ftaheri/robot_learning/assets/44457498/158dbfde-99de-421f-a2c8-769151d9042a" width="30%" /> 
  <img src="https://github.com/ftaheri/robot_learning/assets/44457498/adbfdc67-0a32-4214-b2dc-24eb0c534ce9" width="30%" />

</p>

## near-wall objects
We can see that the landmark falls near the wall as the filters cannot remove points with outlier
depth. However, the coordinates on the other two dimensions are quite accurate.
<p float="left">
  <img src="https://github.com/ftaheri/robot_learning/assets/44457498/7799c1d5-3579-4dfa-b106-03588d66035b" width="30%" />
  <img src="https://github.com/ftaheri/robot_learning/assets/44457498/33b2b714-41e2-43f0-9e7a-8c97dfd32281" width="30%" /> 
  <img src="https://github.com/ftaheri/robot_learning/assets/44457498/080d70e1-6cd8-4ab0-a8bf-2bc8b4acc22c" width="30%" /> 
</p>

## two objects (cluster)
** First experiment: ** It can be seen that since the point clouds for these two bounding boxes are really
close, the farther object depth is filtered out as outliers. <br>

<p float="left">
  <img src="https://github.com/ftaheri/robot_learning/assets/44457498/aace3c21-c392-49ac-a675-df391a0420ba" width="30%" />
  <img src="https://github.com/ftaheri/robot_learning/assets/44457498/90d8d9a4-337c-4498-a595-8739ed1425fa" width="30%" /> 
  <img src="https://github.com/ftaheri/robot_learning/assets/44457498/c6e57426-9c08-4830-9707-96e7a6732469" width="30%" /> 
</p>
** Second experiment: ** Although the bounding boxes are also really close, but the algorithm works
better in this scenario. (The bounding box image is more reliable than the other two because of the
delay) <br>
<p float="left">
  <img src="https://github.com/ftaheri/robot_learning/assets/44457498/6010a41d-dde0-4af9-959e-f4893496c5fa width="30%" />
  <img src="https://github.com/ftaheri/robot_learning/assets/44457498/1422e7b3-594e-47b3-b8dc-7b410da19944 width="30%" /> 
  <img src="https://github.com/ftaheri/robot_learning/assets/44457498/06216443-04a8-4fd0-92dc-ba7ee5f90185 width="30%" /> 
</p>
** Third experiment: ** In this experiment we have somewhat a cluster, and one object is near the
wall. The landmark detection for this object didn’t work well. <br>
<p float="left">
  <img src="https://github.com/ftaheri/robot_learning/assets/44457498/d5b65835-f5fc-44e7-bed4-f77c20c75d7c width="30%" />
  <img src="https://github.com/ftaheri/robot_learning/assets/44457498/a57c5550-7b45-4a44-ab69-564ae968cdd2 width="30%" /> 
  <img src="https://github.com/ftaheri/robot_learning/assets/44457498/84711685-efef-4571-9b78-8dc831ae294" width="30%" /> 
</p>
![image](https://github.com/ftaheri/robot_learning/assets/44457498/aac424bd-700b-4ca5-8321-422a3b5107e8)

