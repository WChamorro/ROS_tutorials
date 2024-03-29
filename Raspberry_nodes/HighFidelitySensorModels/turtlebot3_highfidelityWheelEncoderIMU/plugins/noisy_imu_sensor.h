 /* Copyright [2015] [Alessandro Settimi]
  * 
  * email: ale.settimi@gmail.com
  * 
  * Licensed under the Apache License, Version 2.0 (the "License");
  * you may not use this file except in compliance with the License.
  * You may obtain a copy of the License at
  * 
  * http://www.apache.org/licenses/LICENSE-2.0
  * 
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.*/ 
 
 
 #ifndef GAZEBO_ROS_IMU_SENSOR_H
 #define GAZEBO_ROS_IMU_SENSOR_H
 
 #include <gazebo/common/Plugin.hh>
 #include <gazebo/common/UpdateInfo.hh>
 #include <ignition/math/Vector3.hh>
 #include <ignition/math/Pose3.hh>
 #include <ros/ros.h>
 #include <sensor_msgs/Imu.h>
 #include <string>
 //#include "std_msgs/Float32.h"
 //#include "std_msgs/Int32.h"

 #include "std_msgs/Float32MultiArray.h"
 #include "std_msgs/Int32MultiArray.h"
 
 namespace gazebo
 {
   namespace sensors
   {
     class ImuSensor;
   }
   class GazeboRosImuSensor : public SensorPlugin
   {
   public:
     GazeboRosImuSensor();
     virtual ~GazeboRosImuSensor();
     virtual void Load(sensors::SensorPtr sensor_, sdf::ElementPtr sdf_);
     virtual void CallbackNoise(const std_msgs::Float32MultiArray::ConstPtr& _msg);
     virtual void CallbackNoiseOnOff(const std_msgs::Int32MultiArray::ConstPtr& _msg);
 
   protected:
     virtual void UpdateChild(const gazebo::common::UpdateInfo &/*_info*/);
 
   private:
     bool LoadParameters();
     double GaussianKernel(double mu, double sigma);
     
     ros::NodeHandle* node;
     ros::Publisher imu_data_publisher;
     ros::Subscriber imu_noise_subscriber;
     ros::Subscriber imu_noise_on_off_noise_subscriber;
     sensor_msgs::Imu imu_msg;
     std_msgs::Float32MultiArray imu_gaussian_noise;
     std_msgs::Int32MultiArray  imu_noise_on_off;

     common::Time last_time;
     gazebo::event::ConnectionPtr connection;
     sensors::ImuSensor* sensor;
     sdf::ElementPtr sdf;
     ignition::math::Quaterniond orientation;
     ignition::math::Vector3d accelerometer_data;
     ignition::math::Vector3d gyroscope_data;
 
     //loaded parameters
     std::string robot_namespace;
     std::string topic_name;
     std::string body_name;
     double update_rate;
     double gaussian_noise;
     int noise_on_off = 1;
     ignition::math::Pose3d offset;
   };
 }
 
 #endif //GAZEBO_ROS_IMU_SENSOR_H
