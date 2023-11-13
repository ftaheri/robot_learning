#include "cas726/landmark_mapper.hh"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include <rclcpp/executor.hpp>
 
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cmath>

using namespace std::chrono_literals;

//image callback
void cas726::LandmarkMapper::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &image, 
		const sensor_msgs::msg::PointCloud2::ConstSharedPtr &depth_cloud) {
  
  //TODO
  //1. get current odom pose (base link in odom frame) as an Eigen::Affine3d
  geometry_msgs::msg::TransformStamped transform;
  Eigen::Affine3d T;
  try {
        transform = tf_buffer_->lookupTransform( "base_link","odom",rclcpp::Time(0),tf2::durationFromSec(1.0)); 
        T = tf2::transformToEigen(transform.transform); 
 
  }
  catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(this->get_logger(), "catched error: %s" , ex.what());
    return;
  }

  //2. compute relative transform
  Eigen::Affine3d movement = last_pose_.inverse() * T; 
  
  //3. threshold for enough motion
  double transl = movement.translation().norm();
  Eigen::AngleAxisd rot(movement.rotation());
  double angl = rot.angle();

  //4. if we moved enough, lock the mutex and load up data
  if(transl > 1 || fabsf(angl) > M_PI/4) {
    {
      std::cerr<<"Movement since last pose is "<<transl<<" "<<angl<<std::endl;
      std::cerr<<"Acquiring lock... ";
      std::lock_guard<std::mutex> lg(data_mutex_); //acquire lock
      std::cerr<<"done. Copying images\n";
      color_image_ = *image;
      depth_cloud_ = *depth_cloud;
    }

    //5. signal worker thread to wake up
    data_cv_.notify_all();
    //6. set new last upodate pose
    last_pose_ = T;
  }
}

/** This is what the worker thread runs:
 *  1. Loop until asked to quit
 *  2. When woken up, assemble service request and check for object detections
 *  3. Take detected objects and use them as landmarks
 */
void cas726::LandmarkMapper::run() {

  //image data copies
  sensor_msgs::msg::Image current_color_image;
  sensor_msgs::msg::PointCloud2 current_depth_cloud;
  while(true) {
    {
      //acquire mutex access
      std::unique_lock<std::mutex> mutex_locker(data_mutex_); 
      // std::cerr<<"worker thread sleeping\n"; 
      ////////////the thread waits here and releases the lock so other threads could use it   until the condition variable is called
      data_cv_.wait(mutex_locker);     //release lock and wait to be woken up
      /////////////when the landmark node is quited data_cv_ is notified
      // std::cerr<<"worker thread woke up\n"; 
      //copy data
      current_color_image = color_image_;
      current_depth_cloud = depth_cloud_;
    } //release lock
    ///////since it is outside of the block/scope the destructor is called -> the lock is released
    if(!rclcpp::ok()||quit_) break; // check if we are asked to quit
    
    // std::cerr<<"worker thread assembling request\n"; 
    //load up color image in service request
    auto request = std::make_shared<cas726_interfaces::srv::DetectObjects::Request>();
    //////changed
    request->color = current_color_image;

    //wait for service to exist
    ////////////while client has not connected to the service server
    while (!detect_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting...");
    }
   
    //lambda magic to unblock once response is available 
    ////////////sharedfuturewithrequest is the result of the service call
    auto callback =  [this](rclcpp::Client<cas726_interfaces::srv::DetectObjects>::SharedFutureWithRequest future) { 
    // std::cerr<<"Result callback\n";
          auto response = future.get();
    std::cerr<<"Response has "<<response.second->detections.size()<<" objects\n";
    //////////it sets response and releases the lock
    this->res_cv_.notify_all(); 
	  };



  //////////////client sends the request to the server, the result is send to callback function asynchronously. result is a SharedFutureWithRequest object
  /////////////result is a future that is shared to different owners after sending a request asynchronously
  //////////// callback receives this future. extract the reponse from it.When response is available, it notifies everyone that the lock is released
    auto result = detect_client_->async_send_request(request, std::move(callback));
    //////// The worker thread continues
    // std::cerr<<"Request sent\n";
    {
      // Wait for the result.
      std::unique_lock<std::mutex> response_locker(res_mutex_);
      std::cerr<<"Waiting for result\n";
      //////////////released when the response is available
      res_cv_.wait(response_locker);
    }



//    ////////////we have current depth cloud, current color image, and result.get().second->detections which is the boundingBox array
//     //TODO: 
//     //1.clear current landmarks
    landmarks_.clear();
    //2.iterate over detections
    double avg_x=0 , avg_y=0, avg_z=0;
    int i=0;
    for (const auto& detection : result.get().second->detections) {
      i++;


    float y_lowerbound =  (current_color_image.width/2.0 - detection.x_max)/200.0
      , y_upperbound = (current_color_image.width/2.0 - detection.x_min)/200.0
      , z_lowerbound = (current_color_image.height - detection.y_max)/200.0
      , z_upperbound = (current_color_image.height - detection.y_min)/200.0;





/////////////////remove outliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(current_depth_cloud, *pcl_cloud);
    
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (pcl_cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0,z_upperbound+2.0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pass.filter (*cloud_filtered);

    
    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (y_lowerbound-0.5*(y_upperbound-y_lowerbound),y_upperbound+ 0.5*(y_upperbound-y_lowerbound));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
    pass.filter (*cloud_filtered);

   
    

    // statistical outlier removal filter
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_filtered);
    sor.setMeanK(50);  
    sor.setStddevMulThresh(0.2);  
    sor.filter(*cloud_filtered);

    sensor_msgs::msg::PointCloud2 filtered_cloud;
    pcl::toROSMsg(*cloud_filtered, filtered_cloud);




    // std::cout<<filtered_cloud.width<<"\n";














    //3.For each object create a point cloud iterator (sensor_msgs::PointCloud2Iterator)
      

      sensor_msgs::PointCloud2Iterator<float> iter_x(filtered_cloud, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(filtered_cloud, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(filtered_cloud, "z");
             
    //4.iterate through point cloud and take out points that are within the bounding box
     
      long long int count =0;
      
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
      {
        if(std::isnan(*iter_y) || std::isinf(*iter_y) || std::isnan(*iter_z) || std::isinf(*iter_z) || std::isnan(*iter_x) || std::isinf(*iter_x)){
          continue;
        }
     
          if (*iter_y >= y_lowerbound-0.5 && *iter_y <= y_upperbound+0.5 &&
              *iter_z >= z_lowerbound -1 && *iter_z <= z_upperbound +1 )      
          {
              
              avg_x += *iter_x;
              avg_y += *iter_y;
              avg_z += *iter_z;
              count++;
              
          }
      }
      if (count>0)
        {
            avg_x /= count;
            avg_y /= count;
            avg_z /= count;

            geometry_msgs::msg::PointStamped point;
            point.point.x = avg_x;
            point.point.y = avg_y;
            point.point.z = avg_z;
            
            point.header.frame_id = depth_cloud_.header.frame_id;
            point.header.stamp = depth_cloud_.header.stamp;
            //5.transform point to map frame and save it in the landmark array
            try {
              point = tf_buffer_->transform(point, "odom");
            } catch (tf2::TransformException& ex) {
              RCLCPP_ERROR(get_logger(), "Failed to transform point to map frame: %s", ex.what());
              continue;
            }
            landmarks_.emplace_back(point);
             
        }
    }

    std::cerr<<"landmarks size: "<<landmarks_.size()<<std::endl;

  }
}

//callback that publishes visualization markers
void cas726::LandmarkMapper::update_callback() {
  //TODO: publish the current array of landmarks as a visualization marker to display in rviz

  visualization_msgs::msg::MarkerArray marker_array_msg;;

  // create a new marker message
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "odom";
  marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.ns = "landmarks";
  marker.id = 0;
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  // iterate over the landmarks vector and add each landmark as a new marker
  for (auto& landmark : landmarks_) {
    geometry_msgs::msg::Point point;
    point.x = landmark.point.x;
    point.y = landmark.point.y;
    point.z = landmark.point.z;
    marker.points.push_back(point);
    marker_array_msg.markers.push_back(marker);
    marker.id++;
  }
  // publish the marker array
  marker_publisher_->publish(marker_array_msg);
  
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc,argv);

  auto node = std::make_shared<cas726::LandmarkMapper>();

  //start up a new thread that spins the node
  ///////starts a thread that continuously spins the node's event loop

  ////////starts the thread until stop_async_spinner signals
  std::promise<void> stop_async_spinner;
  std::thread async_spinner_thread(
    [stop_token = stop_async_spinner.get_future(), node]() {
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(node);
      //////executor is the thread that executes the node (landmark mapper)
      executor.spin_until_future_complete(stop_token);

      //at this point we are done and should signal the node
      node->quit();
    });

  //execute program logic
  node->run();
  ///////signals the stop_async_spinner to stop the thread
  stop_async_spinner.set_value();
  /////////// wait for the async_spinner_thread to stop
  async_spinner_thread.join();

  return 0;
}
