
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>



class ObjectInfo {
private:
    std::string objPath;
    unsigned int id;
    geometry_msgs::Pose pose;
    double scale;
    double r;
    double g;
    double b;
public:
    inline void setObjPath(std::string p) {objPath=p;}
    inline std::string getObjPath() {return objPath;}

    inline void setObjID(unsigned int i) {id=i;}
    inline unsigned int getObjID() {return id;}
    inline double getx() {return pose.position.x;}
    inline double gety() {return pose.position.y;}
    inline double getz() {return pose.position.z;}
    inline double getqx() {return pose.orientation.x;}
    inline double getqy() {return pose.orientation.y;}
    inline double getqz() {return pose.orientation.z;}
    inline double getqw() {return pose.orientation.w;}
    inline void setx(double v) {pose.position.x=v;}
    inline void sety(double v) {pose.position.y=v;}
    inline void setz(double v) {pose.position.z=v;}
    inline void setqx(double v) {pose.orientation.x=v;}
    inline void setqy(double v) {pose.orientation.y=v;}
    inline void setqz(double v) {pose.orientation.z=v;}
    inline void setqw(double v) {pose.orientation.w=v;}
    inline double getr() {return r;}
    inline double getg() {return g;}
    inline double getb() {return b;}
    inline double getscale() {return scale;}
    inline void setr(double v) {r=v;}
    inline void setg(double v) {g=v;}
    inline void setb(double v) {b=v;}
    inline void setscale(double v) {scale=v;}
};
std::vector<ObjectInfo> objects;



void SetWorld()
{
    ObjectInfo obj;

    obj.setObjPath("package://rviz_marker_demo/meshes/obstacles/fixtures/Wooden_Plate.dae");
    obj.setObjID(2);
    obj.setx(0.41);
    obj.sety(0);
    obj.setz(0.015);
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(1);
    obj.setr((double)rand() / (double)RAND_MAX);
    obj.setg((double)rand() / (double)RAND_MAX);
    obj.setb((double)rand() / (double)RAND_MAX);
    obj.setscale(1.0);
    objects.push_back(obj);

    obj.setObjPath("package://rviz_marker_demo/meshes/obstacles/plane/Window.stl");
    obj.setObjID(3);
    obj.setx(0.5);
    obj.sety(-0.42);
    obj.setz(0);
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(1);
    obj.setr((double)rand() / (double)RAND_MAX);
    obj.setg((double)rand() / (double)RAND_MAX);
    obj.setb((double)rand() / (double)RAND_MAX);
    obj.setscale(1.0);
    objects.push_back(obj);

    obj.setObjPath("package://rviz_marker_demo/meshes/obstacles/plane/Chassis.stl");
    obj.setObjID(4);
    obj.setx(0.5);
    obj.sety(-0.32);
    obj.setz(0.05);
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(1);
    obj.setr((double)rand() / (double)RAND_MAX);
    obj.setg((double)rand() / (double)RAND_MAX);
    obj.setb((double)rand() / (double)RAND_MAX);
    obj.setscale(1.0);
    objects.push_back(obj);

    obj.setObjPath("package://rviz_marker_demo/meshes/obstacles/plane/BottomWing.stl");
    obj.setObjID(5);
    obj.setx(0.3);
    obj.sety(-0.32);
    obj.setz(0);
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(1);
    obj.setr((double)rand() / (double)RAND_MAX);
    obj.setg((double)rand() / (double)RAND_MAX);
    obj.setb((double)rand() / (double)RAND_MAX);
    obj.setscale(1.0);
    objects.push_back(obj);

    obj.setObjPath("package://rviz_marker_demo/meshes/obstacles/plane/FrontWheel.stl");
    obj.setObjID(6);
    obj.setx(0.35);
    obj.sety(-0.4);
    obj.setz(0);
    obj.setqx(sin(M_PI /4));
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(cos(M_PI /4));
    obj.setr((double)rand() / (double)RAND_MAX);
    obj.setg((double)rand() / (double)RAND_MAX);
    obj.setb((double)rand() / (double)RAND_MAX);
    obj.setscale(1.0);
    objects.push_back(obj);

    obj.setObjPath("package://rviz_marker_demo/meshes/obstacles/plane/MotorGrill.stl");
    obj.setObjID(7);
    obj.setx(0.35);
    obj.sety(-0.46);
    obj.setz(0);
    obj.setqx(sin(M_PI /2));
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(cos(M_PI /2));
    obj.setr((double)rand() / (double)RAND_MAX);
    obj.setg((double)rand() / (double)RAND_MAX);
    obj.setb((double)rand() / (double)RAND_MAX);
    obj.setscale(1.0);
    objects.push_back(obj);

    obj.setObjPath("package://rviz_marker_demo/meshes/obstacles/plane/Propeller.stl");
    obj.setObjID(8);
    obj.setx(0.2);
    obj.sety(-0.43);
    obj.setz(0);
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(1);
    obj.setr((double)rand() / (double)RAND_MAX);
    obj.setg((double)rand() / (double)RAND_MAX);
    obj.setb((double)rand() / (double)RAND_MAX);
    obj.setscale(1.0);
    objects.push_back(obj);

    obj.setObjPath("package://rviz_marker_demo/meshes/obstacles/plane/RearWheelLeft.stl");
    obj.setObjID(9);
    obj.setx(0.12);
    obj.sety(-0.3);
    obj.setz(0);
    obj.setqx(0);
    obj.setqy(sin(-M_PI /4));
    obj.setqz(0);
    obj.setqw(cos(-M_PI /4));
    obj.setr((double)rand() / (double)RAND_MAX);
    obj.setg((double)rand() / (double)RAND_MAX);
    obj.setb((double)rand() / (double)RAND_MAX);
    obj.setscale(1.0);
    objects.push_back(obj);

    obj.setObjPath("package://rviz_marker_demo/meshes/obstacles/plane/RearWheelRight.stl");
    obj.setObjID(10);
    obj.setx(0.12);
    obj.sety(-0.35);
    obj.setz(0);
    obj.setqx(0);
    obj.setqy(sin(-M_PI /4));
    obj.setqz(0);
    obj.setqw(cos(-M_PI /4));
    obj.setr((double)rand() / (double)RAND_MAX);
    obj.setg((double)rand() / (double)RAND_MAX);
    obj.setb((double)rand() / (double)RAND_MAX);
    obj.setscale(1.0);
    objects.push_back(obj);

    obj.setObjPath("package://rviz_marker_demo/meshes/obstacles/plane/RearWing.stl");
    obj.setObjID(11);
    obj.setx(0.2);
    obj.sety(0.3);
    obj.setz(0.01);
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(1);
    obj.setr((double)rand() / (double)RAND_MAX);
    obj.setg((double)rand() / (double)RAND_MAX);
    obj.setb((double)rand() / (double)RAND_MAX);
    obj.setscale(1.0);
    objects.push_back(obj);

    obj.setObjPath("package://rviz_marker_demo/meshes/obstacles/plane/TopWing.stl");
    obj.setObjID(12);
    obj.setx(0.2);
    obj.sety(0.45);
    obj.setz(0);
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(M_PI /4));
    obj.setqw(cos(M_PI /4));
    obj.setr((double)rand() / (double)RAND_MAX);
    obj.setg((double)rand() / (double)RAND_MAX);
    obj.setb((double)rand() / (double)RAND_MAX);
    obj.setscale(1.0);
    objects.push_back(obj);

    obj.setObjPath("package://rviz_marker_demo/meshes/obstacles/plane/UnderBody.stl");
    obj.setObjID(13);
    obj.setx(0.4);
    obj.sety(0.4);
    obj.setz(0.01);
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(0);
    obj.setqw(1);
    obj.setr((double)rand() / (double)RAND_MAX);
    obj.setg((double)rand() / (double)RAND_MAX);
    obj.setb((double)rand() / (double)RAND_MAX);
    obj.setscale(1.0);
    objects.push_back(obj);

    obj.setObjPath("package://rviz_marker_demo/meshes/obstacles/plane/UpperBody.stl");
    obj.setObjID(14);
    obj.setx(0.5);
    obj.sety(0.4);
    obj.setz(0.01);
    obj.setqx(0);
    obj.setqy(0);
    obj.setqz(sin(M_PI /4));
    obj.setqw(cos(M_PI /4));
    obj.setr((double)rand() / (double)RAND_MAX);
    obj.setg((double)rand() / (double)RAND_MAX);
    obj.setb((double)rand() / (double)RAND_MAX);
    obj.setscale(1.0);
    objects.push_back(obj);
}



int main( int argc, char** argv )
{
    ros::init(argc, argv, "rviz_marker_demo");
    ros::NodeHandle n;
    ros::Rate r(1);   
    
    ROS_INFO("**** rviz_marker_demo ****");

    SetWorld();

    ros::Publisher vis_pub = n.advertise<visualization_msgs::MarkerArray>( "visualization_marker_array", 1 );

    std::vector <float> pos;
    int i;
    while (ros::ok())
    {
        i = 0;
        visualization_msgs::Marker marker;
        visualization_msgs::MarkerArray markers;

        
        ROS_DEBUG("LOADING OBJECTS");
        
        for(i=0; i<objects.size(); i++) {
            marker.header.frame_id = "assembly_frame";
            marker.header.stamp = ros::Time();
            marker.ns = "assembly_objects";
            marker.id = objects[i].getObjID();
            marker.mesh_resource = objects[i].getObjPath();

            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            marker.action = visualization_msgs::Marker::ADD;

            
            marker.pose.position.x = objects[i].getx();
            marker.pose.position.y = objects[i].gety();
            marker.pose.position.z = objects[i].getz();
            marker.pose.orientation.x = objects[i].getqx();
            marker.pose.orientation.y = objects[i].getqy();
            marker.pose.orientation.z = objects[i].getqz();
            marker.pose.orientation.w = objects[i].getqw();
            marker.scale.x = objects[i].getscale();
            marker.scale.y = objects[i].getscale();
            marker.scale.z = objects[i].getscale();
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = objects[i].getr();
            marker.color.g = objects[i].getg();
            marker.color.b = objects[i].getb();
            
            
            ROS_DEBUG("OBJECT: [%d]", marker.id);
            ROS_DEBUG("X value is: [%f]", marker.pose.position.x);
            ROS_DEBUG("Y value is: [%f]", marker.pose.position.y);
            ROS_DEBUG("Z value is: [%f]", marker.pose.position.z);

            ROS_DEBUG("ORI X value is: [%f]", marker.pose.orientation.x);
            ROS_DEBUG("ORI Y value is: [%f]", marker.pose.orientation.y);
            ROS_DEBUG("ORI Z value is: [%f]", marker.pose.orientation.z);
            ROS_DEBUG("ORI W value is: [%f]", marker.pose.orientation.w);

            markers.markers.push_back(marker);
        }
        vis_pub.publish( markers );

        r.sleep();
    }
}
