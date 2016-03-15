#include<iostream>
#include<iomanip>
#include<fstream>


#include<ros/ros.h>
#include<accompany_uva_msg/TrackedHumans.h>
#include<geometry_msgs/PointStamped.h>
#include<geometry_msgs/Vector3Stamped.h>
#include<geometry_msgs/Pose.h>
class tracked_humans_publisher
{
public:

	// Constructor
	tracked_humans_publisher()
	{


		pub_ = n_.advertise<accompany_uva_msg::TrackedHumans>("/accompany/TrackedHumans",1);

	}

	// Destructor
	~tracked_humans_publisher()
	{
	}



void set_location(geometry_msgs::Point& p1)
{

  // TODO hack with constant location

  p1.x=0;
  p1.y=-1;
  p1.z=0.0;


}
  void publish()
{
  accompany_uva_msg::TrackedHuman h1;
  geometry_msgs::Point l1;
  geometry_msgs::Vector3Stamped s1;
  int id1=0;
  std::string ident1="Caro";
  ros::Time tfs1;
  ros::Time tls1;

  h1.id=id1;
  h1.identity=ident1;
  h1.firstSeen=ros::Time::now();
  h1.lastSeen=ros::Time::now();


  this->set_location(l1);
  h1.location.point=l1;
  //h1.speed=s1;

  h1.location.header.frame_id="/map";
  h1.speed.header.frame_id="/map";
  h1.location.header.stamp = ros::Time::now();
  h1.speed.header.stamp = ros::Time::now();

  TrackedHumans_.trackedHumans.push_back(h1);

  accompany_uva_msg::TrackedHuman h2;
  geometry_msgs::Point l2;
  geometry_msgs::Vector3Stamped s2;
  int id2=0;
  std::string ident2="Car";
  ros::Time tfs2;
  ros::Time tls2;

  h2.id=id2;
  h2.identity=ident2;
  h2.firstSeen=ros::Time::now();
  h2.lastSeen=ros::Time::now();


  this->set_location(l2);
  h2.location.point=l2;
  //h1.speed=s1;

  h2.location.header.frame_id="/map";
  h2.speed.header.frame_id="/map";
  h2.location.header.stamp = ros::Time::now();
  h2.speed.header.stamp = ros::Time::now();

  TrackedHumans_.trackedHumans.push_back(h2);





  pub_.publish(TrackedHumans_);





}

	ros::NodeHandle n_;


protected:
	ros::Publisher pub_;
  accompany_uva_msg::TrackedHumans TrackedHumans_;




};

int main (int argc, char** argv)
{

	ros::init (argc, argv, "trackedHumansPublisher");

  tracked_humans_publisher thp;


	ros::Rate loop_rate(1);
	while (ros::ok())
	{
    thp.publish();
		ros::spinOnce ();
		loop_rate.sleep();
    }
}
