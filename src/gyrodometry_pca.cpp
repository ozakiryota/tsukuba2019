#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include "amsl_navigation_msgs/NodeEdgeMap.h"
#include <Eigen/Core>
#include <Eigen/Eigenvalues>

class GyrodometryPCA{
	private:
		ros::NodeHandle nh;
		/*subscribe*/
		ros::Subscriber sub_inipose;
		ros::Subscriber sub_odom;
		ros::Subscriber sub_imu;
		ros::Subscriber sub_bias;
		ros::Subscriber sub_map;
		/*publish*/
		ros::Publisher pub_odom_raw;
		ros::Publisher pub_odom_pca;
		tf::TransformBroadcaster tf_broadcaster;
		/*map*/
		amsl_navigation_msgs::NodeEdgeMap map;
		/*odom*/
		nav_msgs::Odometry odom2d_now;
		nav_msgs::Odometry odom2d_last;
		nav_msgs::Odometry odom3d_now;
		nav_msgs::Odometry odom3d_last;
		std::vector<nav_msgs::Odometry> odom_record;
		nav_msgs::Odometry odom_pca;
		/*imu*/
		sensor_msgs::Imu bias;
		sensor_msgs::Imu imu_last;
		/*time*/
		ros::Time time_imu_now;
		ros::Time time_imu_last;
		/*flags*/
		bool first_callback_odom = true;
		bool first_callback_imu = true;
		bool inipose_is_available = false;
		bool bias_is_available = false;
	public:
		GyrodometryPCA();
		void InitializeOdom(nav_msgs::Odometry& odom);
		void CallbackInipose(const geometry_msgs::QuaternionConstPtr& msg);
		void CallbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void Odom2Dto3D(void);
		void ComputePCA(void);
		void CallbackIMU(const sensor_msgs::ImuConstPtr& msg);
		void CallbackBias(const sensor_msgs::ImuConstPtr& msg);
		void CallbackMap(const amsl_navigation_msgs::NodeEdgeMapConstPtr& msg);
		void Publication(void);
};

GyrodometryPCA::GyrodometryPCA()
{
	sub_inipose = nh.subscribe("/initial_pose", 1, &GyrodometryPCA::CallbackInipose, this);
	sub_odom = nh.subscribe("/odom", 1, &GyrodometryPCA::CallbackOdom, this);
	sub_imu = nh.subscribe("/imu/data", 1, &GyrodometryPCA::CallbackIMU, this);
	sub_bias = nh.subscribe("/imu/bias", 1, &GyrodometryPCA::CallbackBias, this);
	sub_map = nh.subscribe("/node_edge_map", 1, &GyrodometryPCA::CallbackMap, this);
	pub_odom_raw = nh.advertise<nav_msgs::Odometry>("/gyrodometry", 1);
	pub_odom_pca = nh.advertise<nav_msgs::Odometry>("/gyrodometry/pca", 1);
	InitializeOdom(odom3d_now);
	InitializeOdom(odom3d_last);
	InitializeOdom(odom_pca);
}

void GyrodometryPCA::InitializeOdom(nav_msgs::Odometry& odom)
{
	odom.header.frame_id = "/odom";
	/* odom.child_frame_id = "/gyrodometry"; */
	odom.pose.pose.position.x = 0.0;
	odom.pose.pose.position.y = 0.0;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation.x = 0.0;
	odom.pose.pose.orientation.y = 0.0;
	odom.pose.pose.orientation.z = 0.0;
	odom.pose.pose.orientation.w = 1.0;
}

void GyrodometryPCA::CallbackMap(const amsl_navigation_msgs::NodeEdgeMapConstPtr& msg)
{
	map = *msg;
}

void GyrodometryPCA::CallbackInipose(const geometry_msgs::QuaternionConstPtr& msg)
{
	if(!inipose_is_available){
		odom3d_now.pose.pose.orientation = *msg;
		inipose_is_available = true;
	} 
}

void GyrodometryPCA::CallbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	odom2d_now = *msg;

	if(!first_callback_odom)	Odom2Dto3D();
	odom_record.push_back(odom3d_now);
	if(odom_record.size()>2)	ComputePCA();
	if(odom_record.size()>1000){	//test
		odom_record.clear();
		std::cout << "clear" << std::endl;
	}

	odom2d_last = odom2d_now;
	odom3d_last = odom3d_now;
	first_callback_odom = false;

	Publication();
}

void GyrodometryPCA::Odom2Dto3D(void)
{
	tf::Quaternion q_pose2d_last;
	tf::Quaternion q_pose3d_last;
	quaternionMsgToTF(odom2d_last.pose.pose.orientation, q_pose2d_last);
	quaternionMsgToTF(odom3d_last.pose.pose.orientation, q_pose3d_last);

	tf::Quaternion q_global_move2d(
		odom2d_now.pose.pose.position.x - odom2d_last.pose.pose.position.x,
		odom2d_now.pose.pose.position.y - odom2d_last.pose.pose.position.y,
		odom2d_now.pose.pose.position.z - odom2d_last.pose.pose.position.z,
		0.0
	);
	tf::Quaternion q_local_move2d = q_pose2d_last.inverse()*q_global_move2d*q_pose2d_last;
	tf::Quaternion q_global_move3d = q_pose3d_last*q_local_move2d*q_pose3d_last.inverse();

	odom3d_now.pose.pose.position.x = odom3d_last.pose.pose.position.x + q_global_move3d.x();
	odom3d_now.pose.pose.position.y = odom3d_last.pose.pose.position.y + q_global_move3d.y();
	odom3d_now.pose.pose.position.z = odom3d_last.pose.pose.position.z + q_global_move3d.z();
}

void GyrodometryPCA::ComputePCA(void)
{
	double ave_x = 0;
	double ave_y = 0;
	double cov_xx = 0;
	double cov_yy = 0;
	double cov_xy = 0;
	
	/* for(size_t i=0;i<odom_record.size();i++) */
	for(nav_msgs::Odometry odom : odom_record){
		ave_x += odom.pose.pose.position.x;
		ave_y += odom.pose.pose.position.y;
	}
	ave_x /= (double)odom_record.size();
	ave_y /= (double)odom_record.size();

	for(nav_msgs::Odometry odom : odom_record){
		cov_xx += (odom.pose.pose.position.x - ave_x)*(odom.pose.pose.position.x - ave_x);
		cov_yy += (odom.pose.pose.position.y - ave_y)*(odom.pose.pose.position.y - ave_y);
		cov_xy += (odom.pose.pose.position.x - ave_x)*(odom.pose.pose.position.y - ave_y);
	}
	cov_xx /= (double)odom_record.size();
	cov_yy /= (double)odom_record.size();
	cov_xy /= (double)odom_record.size();

	Eigen::Matrix2d mat_cov;
	mat_cov <<	cov_xx,	cov_xy,
		  		cov_xy,	cov_yy;
	Eigen::EigenSolver<Eigen::Matrix2d> es(mat_cov);
	Eigen::Vector2d eigenvalues = es.eigenvalues().real();
	Eigen::Matrix2d eigenvectors = es.eigenvectors().real();

	int col_pc1;
	if(eigenvalues(0)>eigenvalues(1))	col_pc1 = 0;
	else	col_pc1 = 1;

	Eigen::Vector2d vec_pc1 = eigenvectors.col(col_pc1);
	Eigen::Vector2d vec_pos;
	vec_pos <<	odom_record[odom_record.size()-1].pose.pose.position.x - odom_record[0].pose.pose.position.x,
				odom_record[odom_record.size()-1].pose.pose.position.y - odom_record[0].pose.pose.position.y;
	Eigen::Vector2d vec_pca = vec_pos.dot(vec_pc1)/vec_pc1.dot(vec_pc1)*vec_pc1;

	odom_pca.pose.pose.position.x = odom_record[0].pose.pose.position.x + vec_pca(0);
	odom_pca.pose.pose.position.y = odom_record[0].pose.pose.position.y + vec_pca(1);
	tf::Quaternion q_pose_pca = tf::createQuaternionFromRPY(0.0, 0.0, atan2(vec_pca(1), vec_pca(0)));
	quaternionTFToMsg(q_pose_pca, odom_pca.pose.pose.orientation);

	/* std::cout << "es.eigenvalues():" << std::endl << es.eigenvalues() << std::endl; */
	std::cout << "es.eigenvalues().real():" << std::endl << es.eigenvalues().real() << std::endl;
	/* std::cout << "es.eigenvectors():" << std::endl << es.eigenvectors() << std::endl; */
	std::cout << "es.eigenvectors().real():" << std::endl << es.eigenvectors().real() << std::endl;
	std::cout << "eigenvectors.col(col_pc1):" << std::endl << eigenvectors.col(col_pc1) << std::endl;
}

void GyrodometryPCA::CallbackIMU(const sensor_msgs::ImuConstPtr& msg)
{
	/*Get dt*/
	time_imu_now = ros::Time::now();
	double dt;
	try{
		dt = (time_imu_now - time_imu_last).toSec();
	}
	catch(std::runtime_error& ex) {
		ROS_ERROR("Exception: [%s]", ex.what());
	}
	time_imu_last = time_imu_now;

	/*PoseEstimation*/
	if(first_callback_imu)	dt = 0.0;
	else if(inipose_is_available){
		double delta_r = (msg->angular_velocity.x + imu_last.angular_velocity.x)*dt/2.0;
		double delta_p = (msg->angular_velocity.y + imu_last.angular_velocity.y)*dt/2.0;
		double delta_y = (msg->angular_velocity.z + imu_last.angular_velocity.z)*dt/2.0;
		if(bias_is_available){
			delta_r -= bias.angular_velocity.x*dt;
			delta_p -= bias.angular_velocity.y*dt;
			delta_y -= bias.angular_velocity.z*dt;
		}
		tf::Quaternion q_relative_rotation = tf::createQuaternionFromRPY(delta_r, delta_p, delta_y);
		tf::Quaternion q_pose3d_now;
		quaternionMsgToTF(odom3d_now.pose.pose.orientation, q_pose3d_now);
		q_pose3d_now = q_pose3d_now*q_relative_rotation;
		q_pose3d_now.normalize();
		quaternionTFToMsg(q_pose3d_now, odom3d_now.pose.pose.orientation);
	}

	imu_last = *msg;
	first_callback_imu = false;
}

void GyrodometryPCA::CallbackBias(const sensor_msgs::ImuConstPtr& msg)
{
	if(!bias_is_available){
		bias = *msg;
		bias_is_available = true;
	}
}

void GyrodometryPCA::Publication(void)
{
	/*publish*/
	odom3d_now.header.stamp = odom2d_now.header.stamp;
	odom_pca.header.stamp = odom2d_now.header.stamp;
	pub_odom_raw.publish(odom3d_now);
	pub_odom_pca.publish(odom_pca);
	/*tf broadcast*/
    geometry_msgs::TransformStamped transform;
	transform.header.stamp = odom2d_now.header.stamp;
	transform.header.frame_id = "/odom";
	transform.child_frame_id = "/gyrodometry";
	transform.transform.translation.x = odom3d_now.pose.pose.position.x;
	transform.transform.translation.y = odom3d_now.pose.pose.position.y;
	transform.transform.translation.z = odom3d_now.pose.pose.position.z;
	transform.transform.rotation = odom3d_now.pose.pose.orientation;
	tf_broadcaster.sendTransform(transform);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "gyrodometry_pca");

	GyrodometryPCA gyrodometry_pca;

	ros::spin();
}
