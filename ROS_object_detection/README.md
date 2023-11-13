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
