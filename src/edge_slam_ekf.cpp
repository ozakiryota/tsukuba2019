#include <ros/ros.h>
#include <Eigen/Core>
/* #include <Eigen/LU> */
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

class EdgeSLAMEKF{
	private:
		ros::NodeHandle nh;
		/*subscribe*/
		ros::Subscriber sub_reset_pose;
		ros::Subscriber sub_bias;
		ros::Subscriber sub_imu;
		ros::Subscriber sub_odom;
		ros::Subscriber sub_slam_odom;
		/*publish*/
		ros::Publisher pub_pose;
		/*const*/
		const int num_state = 3;
		const int size_robot_state = 6;	//X, Y, Z, R, P, Y (Global)
		/*objects*/
		Eigen::MatrixXd X;
		Eigen::MatrixXd P;
		sensor_msgs::Imu bias;
		Eigen::VectorXd Reset_origin;
		/*flags*/
		bool bias_is_available = false;
		bool first_callback_imu = true;
		bool first_callback_odom = true;
		bool first_callback_slam = true;
		/*counter*/
		int count_rpy_walls = 0;
		int count_slam = 0;
		/*time*/
		ros::Time time_imu_now;
		ros::Time time_imu_last;
		ros::Time time_odom_now;
		ros::Time time_odom_last;
	public:
		EdgeSLAMEKF();
		void CallbackResetPose(const geometry_msgs::PoseStampedConstPtr& msg);
		void CallbackBias(const sensor_msgs::ImuConstPtr& msg);
		void CallbackIMU(const sensor_msgs::ImuConstPtr& msg);
		void PredictionIMU(sensor_msgs::Imu imu, double dt);
		void CallbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void PredictionOdom(nav_msgs::Odometry odom, double dt);
		void CallbackSLAMOdom(const nav_msgs::OdometryConstPtr& msg);
		Eigen::Vector3d GetLocalDeltaXYZ(nav_msgs::Odometry origin, nav_msgs::Odometry target);
		Eigen::Vector3d GetLocalDeltaRPY(nav_msgs::Odometry origin, nav_msgs::Odometry target);
		void Publication();
		geometry_msgs::PoseStamped StateVectorToPoseStamped(void);
		float PiToPi(double angle);
};

EdgeSLAMEKF::EdgeSLAMEKF()
{
	sub_reset_pose = nh.subscribe("/reset_pose", 1, &EdgeSLAMEKF::CallbackResetPose, this);
	sub_bias = nh.subscribe("/imu_bias", 1, &EdgeSLAMEKF::CallbackBias, this);
	sub_imu = nh.subscribe("/imu/data", 1, &EdgeSLAMEKF::CallbackIMU, this);
	sub_odom = nh.subscribe("/tinypower/odom", 1, &EdgeSLAMEKF::CallbackOdom, this);
	sub_odom = nh.subscribe("/integrated_to_init", 1, &EdgeSLAMEKF::CallbackSLAMOdom, this);
	pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/pose_edge_slam_ekf", 1);
	X = Eigen::MatrixXd::Constant(size_robot_state, 1, 0.0);
	P = Eigen::MatrixXd::Identity(size_robot_state, size_robot_state);
	Reset_origin = Eigen::VectorXd::Zero(6);
}

void EdgeSLAMEKF::CallbackResetPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
	Eigen::Vector3d Reset_delta_xyz = {
		msg->pose.position.x - X(0, 0),
		msg->pose.position.y - X(1, 0),
		msg->pose.position.z - X(2, 0)
	};

	tf::Quaternion q_pose_origin = tf::createQuaternionFromRPY(X(3, 0), X(4, 0), X(5, 0));
	tf::Quaternion q_pose_target;
	quaternionMsgToTF(msg->pose.orientation, q_pose_target);

	Eigen::Vector3d Reset_delta_rpy;
	tf::Matrix3x3(q_pose_target*q_pose_origin.inverse()).getRPY(Reset_delta_rpy(0), Reset_delta_rpy(1), Reset_delta_rpy(2));
	Reset_delta_rpy = {0, 0, Reset_delta_rpy(2)};	//only yaw is used because of 2D

	Reset_origin.segment(0, 3) -= Reset_delta_xyz;
	Reset_origin.segment(3, 3) -= Reset_delta_rpy;
}

void EdgeSLAMEKF::CallbackBias(const sensor_msgs::ImuConstPtr& msg)
{
	if(!bias_is_available){
		bias = *msg;
		bias_is_available = true;
	}
}

void EdgeSLAMEKF::CallbackIMU(const sensor_msgs::ImuConstPtr& msg)
{
	std::cout << "Callback IMU" << std::endl;

	time_imu_now = ros::Time::now();
	double dt;
	try{
		dt = (time_imu_now - time_imu_last).toSec();
	}
	catch(std::runtime_error& ex) {
		ROS_ERROR("Exception: [%s]", ex.what());
	}
	time_imu_last = time_imu_now;
	if(first_callback_imu)	dt = 0.0;
	else	PredictionIMU(*msg, dt);
	
	Publication();

	first_callback_imu = false;
}

void EdgeSLAMEKF::PredictionIMU(sensor_msgs::Imu imu, double dt)
{
	std::cout << "PredictionIMU" << std::endl;
	double x = X(0, 0);
	double y = X(1, 0);
	double z = X(2, 0);
	double r_ = X(3, 0);
	double p_ = X(4, 0);
	double y_ = X(5, 0);

	double delta_r = imu.angular_velocity.x*dt;
	double delta_p = imu.angular_velocity.y*dt;
	double delta_y = imu.angular_velocity.z*dt;
	if(bias_is_available){
		delta_r -= bias.angular_velocity.x*dt;
		delta_p -= bias.angular_velocity.y*dt;
		delta_y -= bias.angular_velocity.z*dt;
	}
	Eigen::Vector3d Drpy = {delta_r, delta_p, delta_y};

	Eigen::Matrix3d Rot_rpy;	//normal rotation
	Rot_rpy <<	1,	sin(r_)*tan(p_),	cos(r_)*tan(p_),
				0,	cos(r_),			-sin(r_),
				0,	sin(r_)/cos(p_),	cos(r_)/cos(p_);

	Eigen::Matrix3d Rot_xyz_inv;	//inverse rotation
	Rot_xyz_inv <<	cos(delta_p)*cos(delta_y),	cos(delta_p)*sin(delta_y),	-sin(delta_p),
					sin(delta_r)*sin(delta_p)*cos(delta_y) - cos(delta_r)*sin(delta_y),	sin(delta_r)*sin(delta_p)*sin(delta_y) + cos(delta_r)*cos(delta_y),	sin(delta_r)*cos(delta_p),
					cos(delta_r)*sin(delta_p)*cos(delta_y) + sin(delta_r)*sin(delta_y),	cos(delta_r)*sin(delta_p)*sin(delta_y) - sin(delta_r)*cos(delta_y),	cos(delta_r)*cos(delta_p);

	/*F*/
	Eigen::MatrixXd F(X.rows(), 1);
	F.block(0, 0, 3, 1) = X.block(0, 0, 3, 1);
	F.block(3, 0, 3, 1) = X.block(3, 0, 3, 1) + Rot_rpy*Drpy;

	/*jF*/
	Eigen::MatrixXd jF(X.rows(), X.rows());
	/*jF-xyz*/
	jF.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
	jF.block(0, 3, 3, 3) = Eigen::Matrix3d::Zero();
	/*jF-rpy*/
	jF.block(3, 0, 3, 3) = Eigen::Matrix3d::Zero();
	jF(3, 3) = 1 + cos(r_)*tan(p_)*delta_p - sin(r_)*tan(p_)*delta_y;
	jF(3, 4) = sin(r_)/cos(p_)/cos(p_)*delta_p + cos(r_)/cos(p_)/cos(p_)*delta_y;
	jF(3, 5) = 0;
	jF(4, 3) = -sin(r_)*delta_p - cos(r_)*delta_y;
	jF(4, 4) = 1;
	jF(4, 5) = 0;
	jF(5, 3) = cos(r_)/cos(p_)*delta_p - sin(r_)/cos(p_)*delta_y;
	jF(5, 4) = sin(r_)*sin(p_)/cos(p_)/cos(p_)*delta_p + cos(r_)*sin(p_)/cos(p_)/cos(p_)*delta_y;
	jF(5, 5) = 1;
	
	/*Q*/
	const double sigma = 1.0e-1;
	Eigen::MatrixXd Q = sigma*Eigen::MatrixXd::Identity(X.rows(), X.rows());
	
	/*Update*/
	X = F;
	P = jF*P*jF.transpose() + Q;

	std::cout << "X =" << std::endl << X << std::endl;
	std::cout << "P =" << std::endl << P << std::endl;
	std::cout << "jF =" << std::endl << jF << std::endl;
}

void EdgeSLAMEKF::CallbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	std::cout << "Callback Odom" << std::endl;

	time_odom_now = ros::Time::now();
	double dt;
	try{
		dt = (time_odom_now - time_odom_last).toSec();
	}
	catch(std::runtime_error& ex) {
		ROS_ERROR("Exception: [%s]", ex.what());
	}
	time_odom_last = time_odom_now;
	if(first_callback_odom)	dt = 0.0;
	else	PredictionOdom(*msg, dt);
	
	Publication();

	first_callback_odom = false;
}

void EdgeSLAMEKF::PredictionOdom(nav_msgs::Odometry odom, double dt)
{
	std::cout << "Prediction Odom" << std::endl;

	double x = X(0, 0);
	double y = X(1, 0);
	double z = X(2, 0);
	double r_ = X(3, 0);
	double p_ = X(4, 0);
	double y_ = X(5, 0);
	Eigen::Vector3d Dxyz = {odom.twist.twist.linear.x*dt, 0, 0};

	Eigen::Matrix3d Rot_xyz;	//normal rotation
	Rot_xyz <<	cos(p_)*cos(y_),	sin(r_)*sin(p_)*cos(y_) - cos(r_)*sin(y_),	cos(r_)*sin(p_)*cos(y_) + sin(r_)*sin(y_),
				cos(p_)*sin(y_),	sin(r_)*sin(p_)*sin(y_) + cos(r_)*cos(y_),	cos(r_)*sin(p_)*sin(y_) - sin(r_)*cos(y_),
				-sin(p_),			sin(r_)*cos(p_),							cos(r_)*cos(p_);

	/*F*/
	Eigen::MatrixXd F(X.rows(), 1);
	F.block(0, 0, 3, 1) = X.block(0, 0, 3, 1) + Rot_xyz*Dxyz;
	F.block(3, 0, 3, 1) = X.block(3, 0, 3, 1);

	/*jF*/
	Eigen::MatrixXd jF(X.rows(), X.rows());
	/*jF-xyz*/
	jF.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
	jF(0, 3) = Dxyz(1)*(cos(r_)*sin(p_)*cos(y_) + sin(r_)*sin(y_)) + Dxyz(2)*(-sin(r_)*sin(p_)*cos(y_) + cos(r_)*sin(y_));
	jF(0, 4) = Dxyz(0)*(-sin(p_)*cos(y_)) + Dxyz(1)*(sin(r_)*cos(p_)*cos(y_)) + Dxyz(2)*(cos(r_)*cos(p_)*cos(y_));
	jF(0, 5) = Dxyz(0)*(-cos(p_)*sin(y_)) + Dxyz(1)*(-sin(r_)*sin(p_)*sin(y_) - cos(r_)*cos(y_)) + Dxyz(2)*(-cos(r_)*sin(p_)*sin(y_) + sin(r_)*cos(y_));
	jF(1, 3) = Dxyz(1)*(cos(r_)*sin(p_)*sin(y_) - sin(r_)*cos(y_)) + Dxyz(2)*(-sin(r_)*sin(p_)*sin(y_) - cos(r_)*cos(y_));
	jF(1, 4) = Dxyz(0)*(-sin(p_)*sin(y_)) + Dxyz(1)*(sin(r_)*cos(p_)*sin(y_)) + Dxyz(2)*(cos(r_)*cos(p_)*sin(y_));
	jF(1, 5) = Dxyz(0)*(cos(p_)*cos(y_)) + Dxyz(1)*(sin(r_)*sin(p_)*cos(y_) - cos(r_)*sin(y_)) + Dxyz(2)*(cos(r_)*sin(p_)*cos(y_) + sin(r_)*sin(y_));
	jF(2, 3) = Dxyz(1)*(cos(r_)*cos(p_)) + Dxyz(2)*(-sin(r_)*cos(p_)) ;
	jF(2, 4) = Dxyz(0)*(-cos(p_)) + Dxyz(1)*(-sin(r_)*sin(p_)) + Dxyz(2)*(-cos(r_)*sin(p_)) ;
	jF(2, 5) = 0;
	/*jF-rpy*/
	jF.block(3, 0, 3, 3) = Eigen::Matrix3d::Zero();
	jF.block(3, 3, 3, 3) = Eigen::Matrix3d::Identity();

	/*Q*/
	const double sigma = 1.0e-1;
	Eigen::MatrixXd Q = sigma*Eigen::MatrixXd::Identity(X.rows(), X.rows());
	
	/*Update*/
	X = F;
	P = jF*P*jF.transpose() + Q;

	std::cout << "Dxyz =" << std::endl << Dxyz << std::endl;
}

void EdgeSLAMEKF::CallbackSLAMOdom(const nav_msgs::OdometryConstPtr& msg)
{
}

void EdgeSLAMEKF::Publication(void)
{
	std::cout << "Publication" << std::endl;

	geometry_msgs::PoseStamped pose_pub = StateVectorToPoseStamped();
	pose_pub.header.frame_id = "/odom";
	// pose_pub.header.stamp = ros::Time::now();
	pose_pub.header.stamp = time_imu_now;
	pub_pose.publish(pose_pub);
}

geometry_msgs::PoseStamped EdgeSLAMEKF::StateVectorToPoseStamped(void)
{
	geometry_msgs::PoseStamped pose;
	pose.pose.position.x = X(0, 0);
	pose.pose.position.y = X(1, 0);
	pose.pose.position.z = X(2, 0);
	tf::Quaternion q_orientation = tf::createQuaternionFromRPY(X(3, 0), X(4, 0), X(5, 0));
	pose.pose.orientation.x = q_orientation.x();
	pose.pose.orientation.y = q_orientation.y();
	pose.pose.orientation.z = q_orientation.z();
	pose.pose.orientation.w = q_orientation.w();

	return pose;
}

float EdgeSLAMEKF::PiToPi(double angle)
{
	return fmod(angle + M_PI, 2*M_PI) - M_PI;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "edge_slam_ekf");
	std::cout << "Edge SLAM E.K.F." << std::endl;
	
	EdgeSLAMEKF edge_slam_ekf;
	ros::spin();
}
