#include "cas726/mapper.hh"
#include <iostream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h> 
#include <algorithm>

void cas726::Mapper::laser_callback(const sensor_msgs::msg::LaserScan &scan) {
  RCLCPP_INFO(this->get_logger(), "Got laser message with %ld ranges", scan.ranges.size());

  //TODO: implement laser callback
  // 0. lookup transform on TF
  geometry_msgs::msg::TransformStamped transform;
  try {
        transform = tf_buffer_->lookupTransform( "base_link","odom",rclcpp::Time(0),tf2::durationFromSec(1.0)); 
  }
  catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "catched error: %s" , ex.what());
    return;
  }
  // 1. calculate transform as a Pose2 object
  map2sensor_.x = transform.transform.translation.x;
  map2sensor_.y = transform.transform.translation.y;
  tf2::Quaternion quat;
  tf2::fromMsg(transform.transform.rotation, quat);
  map2sensor_.theta =quat.getAngle() * quat.getAxis().z();


  // 2. iterate through 
  //    a. all map cells and project them to the laser frame
  // or b. all rays and trace through map
   
  for(int y =0 ;y<height_y_ ; y++){
    for(int x=0 ; x<width_x_ ; x++){
        // x,y in map_frame by meters
      double cell_x =   x * map_resolution_ + map_msg_.info.origin.position.x;
      double cell_y = y * map_resolution_ + map_msg_.info.origin.position.y;
        // x,y in laser frame
      double laser_x = map2sensor_.x + cell_x * std::cos(map2sensor_.theta) - cell_y * std::sin(map2sensor_.theta);
      //laser_y = -laser_y;
      double laser_y = map2sensor_.y + cell_x * std::sin(map2sensor_.theta) + cell_y * std::cos(map2sensor_.theta);
        //since map picture is flipped
      std::swap(laser_x,laser_y);
      laser_y = -laser_y;

        //distance from laser frame 
      double range = std::sqrt(laser_x * laser_x + laser_y * laser_y);
//      RCLCPP_INFO(this->get_logger(),"laser_x , laser_y , range: %f %f %f " , laser_x , laser_y , range);

	// 3. update map cells 
      if (range >= scan.range_min && range <= scan.range_max)
      {
        double angle = std::atan2(laser_y, laser_x) - map2sensor_.theta;
        int ray_index = std::round((angle - scan.angle_min) / scan.angle_increment);
        //RCLCPP_INFO(this->get_logger(),"ray_index , angle: %d %f  " , ray_index , angle);
        if (ray_index >= 0 && ray_index < scan.ranges.size())
        {
          double cell_range = range - scan.ranges[ray_index];
          if (cell_range < -map_resolution_ && map_[y*width_x_ +x] <=100)
          {
            // The cell is within the occlusion threshold
            *at(y,x)+=1.0;
          }
          else if (cell_range <map_resolution_ && map_[y*width_x_ + x] >= -100)
          {
            // The cell is within the free threshold
            *at(y,x)-=1.0;
          }
          else {
            *at(y,x) = 0;
          }
          
        }
      }
    }
  }
  RCLCPP_INFO(this->get_logger(),"map_ updated! ");



}

//create a message and publish to map update topic      
void cas726::Mapper::map_update_callback() {
  RCLCPP_INFO(this->get_logger(), "Updating map_msg_");

  //update header
  map_msg_.info.map_load_time = now();
  map_msg_.header.stamp = now();
  this->update_map_msg();
  //TODO uncomment below
  map_publisher_->publish(map_msg_);
}


//update map to message
void cas726::Mapper::update_map_msg() {
  int8_t FREE=0;
  int8_t OCC=100;
  int8_t UNKN=-1;
  map_msg_.data.clear();
  //TODO here iterate through map cells and push them into message 
  for (int i=0 ; i<width_x_ * height_y_ ; i++){
    if (map_[i]<evd_occ_) map_msg_.data.push_back(OCC);
    else if (map_[i]>evd_free_) map_msg_.data.push_back(FREE);
    else map_msg_.data.push_back(UNKN);
  }
    RCLCPP_INFO(this->get_logger(), "Updating map_msg_ with size: %ld using map_",map_msg_.data.size());

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);

  auto node = std::make_shared<cas726::Mapper>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
