#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/LinkStates.h>

#include <eigen3/Eigen/Dense>

bool singularity = false;
double w_left,w_right;
ros::Time T;
bool got_w=false;

Eigen::Vector3d q_log (double &w, double &x, double &y, double &z){


	Eigen::Vector3d qv;
		Eigen::Vector3d w_;
		qv << x , y ,z ;
		if(w==0 || w<1e-5){
			double phi =0;
			singularity = true;
			return qv * phi;
		}
		double phi = 2 * atan2(qv.norm(),w);
		qv.normalize();
		return qv*phi;

	/*Eigen::Vector3d qv;
	Eigen::Vector3d w_;
	qv << x , y ,z ;
	double c = (1 - qv.squaredNorm()/(3*w*w))*(2/w);

	return w_ = qv*c;*/


	// Calculate the logarithm of the relative quaternion
	    /*double angle = 2.0 * acos(w); // Angle of rotation
	    double scale = sqrt(1 - w * w);

	    Eigen::Vector3d log_q_relative;
	    if (scale < 1e-6) {
	        log_q_relative << 0, 0, 0; // If scale is close to zero, set log to zero vector
	    } else {
	        log_q_relative << x / scale * angle,
	                          y / scale * angle,
	                          z / scale * angle;
	    }


	    return log_q_relative;*/


}

void gazebo_callback(const gazebo_msgs::LinkStates::ConstPtr msg){
	w_left=msg->twist[2].angular.y; //left
	w_right=msg->twist[3].angular.y; //right
	got_w = true;

}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_listener");

  ros::NodeHandle node;

 

  ros::Publisher wheel_pub = node.advertise<geometry_msgs::TwistStamped>("wheel_vel", 10);
  ros::Subscriber gazebon = node.subscribe("/gazebo/link_states",10,gazebo_callback);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(28.0);
  Eigen::Quaterniond qr(1,0,0,0);
  Eigen::Quaterniond qr_ant(1,0,0,0);
  Eigen::Quaterniond ql(1,0,0,0);
  Eigen::Quaterniond ql_ant(1,0,0,0);
  double dt = 0;
  double t_ant = -1;
  geometry_msgs::TwistStamped wr_ant ;
  geometry_msgs::TwistStamped wl_ant ;

  while (node.ok()){
   /*geometry_msgs::TransformStamped left_tf,right_tf;

    try{
      left_tf = tfBuffer.lookupTransform("base_footprint","wheel_left_link",
                               ros::Time(0));
      ql.w()=left_tf.transform.rotation.w;
      ql.x()=left_tf.transform.rotation.x;
      ql.y()=left_tf.transform.rotation.y;
      ql.z()=left_tf.transform.rotation.z;
                               
      right_tf = tfBuffer.lookupTransform("base_footprint","wheel_right_link",
                               ros::Time(0));
      qr.w()=right_tf.transform.rotation.w;
      qr.x()=right_tf.transform.rotation.x;
      qr.y()=right_tf.transform.rotation.y;
      qr.z()=right_tf.transform.rotation.z;
             
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }


    if(t_ant<0){

    	geometry_msgs::TwistStamped wheel;
    	wheel.header.stamp = ros::Time::now();
    	wheel.twist.angular.x = 0;
    	wheel.twist.angular.y = 0;
    	wheel.twist.angular.z = 0;
    	wheel_pub.publish(wheel);

    	    ql_ant = ql;
    	    qr_ant = qr;
    	    t_ant = wheel.header.stamp.toSec();
    }else{
    	ros::Time t = ros::Time::now();
    	dt =  t.toSec() - t_ant;
    if(dt>0.01){


    manif::SO3d R1(qr);
    manif::SO3d R2(qr_ant);
    manif::SO3Tangentd wr = R1.minus(R2);
    //wr = wr / dt;

    manif::SO3d L1(ql);
    manif::SO3d L2(ql_ant);
    manif::SO3Tangentd wl = L1.minus(L2);
        //wl = wl / dt;


    geometry_msgs::TwistStamped wheel;
        	wheel.header.stamp = t;
        	wheel.twist.angular.x = 0;
        	wheel.twist.angular.y = wr.coeffs().z()/dt;
        	wheel.twist.angular.z = wl.coeffs().z()/dt;
        	wheel_pub.publish(wheel);


    qr_ant = qr;
    ql_ant = ql;
    t_ant = t.toSec();*/



 


    //}
    //}

	  if(got_w){
		    geometry_msgs::TwistStamped wheel;
		        	wheel.header.stamp = ros::Time::now();
		        	wheel.twist.angular.x = 0;
		        	wheel.twist.angular.y = w_left;
		        	wheel.twist.angular.z = w_right;
		        	wheel_pub.publish(wheel);
		        	got_w=false;
	  }


    //rate.sleep();
    ros::spinOnce();
  }
  return 0;
};
