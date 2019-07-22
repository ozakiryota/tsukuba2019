#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

class ResetGPSOrigin{
	private:
		/*node handle*/
		ros::NodeHandle nh;
		ros::NodeHandle nhPrivate;
		/*subscriber*/
		ros::Subscriber sub_odom;
		/*publisher*/
		ros::Publisher pub_odom;
		tf::TransformBroadcaster tf_broadcaster;
		/*objects*/
		nav_msgs::Odometry odom_pub;
		tf::Quaternion q_ini_position;
		tf::Quaternion q_ini_orientation;
		double map_origin[2];	//utm(x, y)
		/*flags*/
		bool inipose_is_available = false;
		/*fram_id*/
		std::string parent_frame_id_name;
		std::string child_frame_id_name;
	public:
		ResetGPSOrigin();
		void CallbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void Publication(void);
};

ResetGPSOrigin::ResetGPSOrigin()
	: nhPrivate("~")
{
	sub_odom = nh.subscribe("/odom", 1, &ResetGPSOrigin::CallbackOdom, this);
	pub_odom = nh.advertise<nav_msgs::Odometry>("/odom/reset_origin", 1);

	nhPrivate.param("parent_frame_id", parent_frame_id_name, std::string("/odom"));
	nhPrivate.param("child_frame_id", child_frame_id_name, std::string("/gps_odom/reset_origin"));
	nhPrivate.param("map_origin_x", map_origin[0], 416892.838);
	nhPrivate.param("map_origin_y", map_origin[1], 3993537.078);
	std::cout <<" map_origin[0] = " << map_origin[0] << std::endl;
	std::cout <<" map_origin[1] = " << map_origin[1] << std::endl;
}

void ResetGPSOrigin::CallbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	// tf::Quaternion q_raw_position;
	// tf::Quaternion q_raw_orientation;
	// tf::Quaternion q_relative_position;
	// tf::Quaternion q_relative_orientation;
    //
	// q_raw_position = tf::Quaternion(
	// 	msg->pose.pose.position.x,
	// 	msg->pose.pose.position.y,
	// 	msg->pose.pose.position.z,
	// 	0.0
	// );
	// quaternionMsgToTF(msg->pose.pose.orientation, q_raw_orientation);
	// if(!inipose_is_available){
	// 	q_ini_position = q_raw_position;
	// 	q_ini_orientation = q_raw_orientation;
	// 	inipose_is_available = true;
	// }
	// else{
	// 	#<{(|compute relative|)}>#
	// 	q_relative_position = tf::Quaternion(
	// 		q_raw_position.x() - q_ini_position.x(),
	// 		q_raw_position.y() - q_ini_position.y(),
	// 		q_raw_position.z() - q_ini_position.z(),
	// 		0.0
	// 	);
	// 	q_relative_position = q_ini_orientation.inverse()*q_relative_position*q_ini_orientation;
	// 	q_relative_orientation = q_ini_orientation.inverse()*q_raw_orientation;
    //
	// 	#<{(|input|)}>#
	// 	odom_pub.header.frame_id = parent_frame_id_name;
	// 	odom_pub.child_frame_id = child_frame_id_name;
	// 	odom_pub.header.stamp = msg->header.stamp;
	// 	odom_pub.pose.pose.position.x = q_relative_position.x();
	// 	odom_pub.pose.pose.position.y = q_relative_position.y();
	// 	// odom_pub.pose.pose.position.z = q_relative_position.z();
	// 	odom_pub.pose.pose.position.z = 0;
	// 	quaternionTFToMsg(q_relative_orientation, odom_pub.pose.pose.orientation);
	// 	odom_pub.twist = msg->twist;
    //
	// 	#<{(|fit to map|)}>#
	// 	odom_pub.pose.pose.position.x -= map_origin[0];
	// 	odom_pub.pose.pose.position.y -= map_origin[1];
    //
	// 	#<{(|publish|)}>#
	// 	Publication();
	// }
	
	/*input*/
	odom_pub = *msg;
	odom_pub.header.frame_id = parent_frame_id_name;
	odom_pub.child_frame_id = child_frame_id_name;

	/*fit to map*/
	odom_pub.pose.pose.position.x -= map_origin[0];
	odom_pub.pose.pose.position.y -= map_origin[1];

	/*publish*/
	Publication();
}

void ResetGPSOrigin::Publication(void)
{
	/*publish*/
	pub_odom.publish(odom_pub);
	/*tf broadcast*/
    geometry_msgs::TransformStamped transform;
	transform.header = odom_pub.header;
	transform.child_frame_id = odom_pub.child_frame_id;
	transform.transform.translation.x = odom_pub.pose.pose.position.x;
	transform.transform.translation.y = odom_pub.pose.pose.position.y;
	transform.transform.translation.z = odom_pub.pose.pose.position.z;
	transform.transform.rotation = odom_pub.pose.pose.orientation;
	tf_broadcaster.sendTransform(transform);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "reset_gps_odom");

	ResetGPSOrigin reset_gps_odom;

	ros::spin();
}
