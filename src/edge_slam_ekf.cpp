#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/LU>
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
		const int size_robot_state = 6;
		/*objects*/
		sensor_msgs::Imu bias;
		Eigen::VectorXd X;	//X, Y, Z, R, P, Y (Global)
		Eigen::MatrixXd P;
		Eigen::VectorXd Reset_origin;	//X, Y, Z, R, P, Y (Global)
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
		void ObservationSLAMOdom(nav_msgs::Odometry slam_odom);
		Eigen::Vector3d GetLocalDeltaXYZ(nav_msgs::Odometry origin, nav_msgs::Odometry target);
		Eigen::Vector3d GetLocalDeltaRPY(nav_msgs::Odometry origin, nav_msgs::Odometry target);
		void Publication();
		geometry_msgs::PoseStamped StateVectorToPoseStamped(void);
		float PiToPi(double angle);
};

EdgeSLAMEKF::EdgeSLAMEKF()
{
	sub_reset_pose = nh.subscribe("/reset_pose", 1, &EdgeSLAMEKF::CallbackResetPose, this);
	sub_bias = nh.subscribe("/imu/bias", 1, &EdgeSLAMEKF::CallbackBias, this);
	sub_imu = nh.subscribe("/imu/data", 1, &EdgeSLAMEKF::CallbackIMU, this);
	sub_odom = nh.subscribe("/tinypower/odom", 1, &EdgeSLAMEKF::CallbackOdom, this);
	sub_slam_odom = nh.subscribe("/integrated_to_init", 1, &EdgeSLAMEKF::CallbackSLAMOdom, this);
	pub_pose = nh.advertise<geometry_msgs::PoseStamped>("/pose_edge_slam_ekf", 1);
	X = Eigen::VectorXd::Zero(size_robot_state);
	P = Eigen::MatrixXd::Identity(size_robot_state, size_robot_state);
	Reset_origin = Eigen::VectorXd::Zero(size_robot_state);
}

void EdgeSLAMEKF::CallbackResetPose(const geometry_msgs::PoseStampedConstPtr& msg)
{
	Eigen::Vector3d Reset_delta_xyz = {
		msg->pose.position.x - X(0),
		msg->pose.position.y - X(1),
		msg->pose.position.z - X(2)
	};

	tf::Quaternion q_pose_origin = tf::createQuaternionFromRPY(X(3), X(4), X(5));
	tf::Quaternion q_pose_target;
	quaternionMsgToTF(msg->pose.orientation, q_pose_target);

	Eigen::Vector3d Reset_delta_rpy;
	tf::Matrix3x3(q_pose_target*q_pose_origin.inverse()).getRPY(Reset_delta_rpy(0), Reset_delta_rpy(1), Reset_delta_rpy(2));
	Reset_delta_rpy = {0, 0, Reset_delta_rpy(2)};	//only yaw is used because of 2D

	Reset_origin.segment(0, 3) += Reset_delta_xyz;
	Reset_origin.segment(3, 3) += Reset_delta_rpy;
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
	/* std::cout << "Callback IMU" << std::endl; */

	// time_imu_now = ros::Time::now();
	time_imu_now = msg->header.stamp;
	double dt;
	try{
		dt = (time_imu_now - time_imu_last).toSec();
	}
	catch(std::runtime_error& ex) {
		ROS_ERROR("Exception: [%s]", ex.what());
	}
	time_imu_last = time_imu_now;
	if(first_callback_imu)	dt = 0.0;
	else if(bias_is_available)	PredictionIMU(*msg, dt);
	
	Publication();

	first_callback_imu = false;
}

void EdgeSLAMEKF::PredictionIMU(sensor_msgs::Imu imu, double dt)
{
	/* std::cout << "PredictionIMU" << std::endl; */
	double x = X(0);
	double y = X(1);
	double z = X(2);
	double r_ = X(3);
	double p_ = X(4);
	double y_ = X(5);

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
	Eigen::VectorXd F(X.size());
	F.segment(0, 3) = X.segment(0, 3);
	F.segment(3, 3) = X.segment(3, 3) + Rot_rpy*Drpy;

	/*jF*/
	Eigen::MatrixXd jF(X.size(), X.size());
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
	Eigen::MatrixXd Q = sigma*Eigen::MatrixXd::Identity(X.size(), X.size());
	
	/*Update*/
	X = F;
	P = jF*P*jF.transpose() + Q;

	/* std::cout << "jF =" << std::endl << jF << std::endl; */
	/* std::cout << "X =" << std::endl << X << std::endl; */
	/* std::cout << "P =" << std::endl << P << std::endl; */
}

void EdgeSLAMEKF::CallbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	/* std::cout << "Callback Odom" << std::endl; */

	// time_odom_now = ros::Time::now();
	time_odom_now = msg->header.stamp;
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
	/* std::cout << "Prediction Odom" << std::endl; */

	double x = X(0);
	double y = X(1);
	double z = X(2);
	double r_ = X(3);
	double p_ = X(4);
	double y_ = X(5);
	Eigen::Vector3d Dxyz = {odom.twist.twist.linear.x*dt, 0, 0};

	Eigen::Matrix3d Rot_xyz;	//normal rotation
	Rot_xyz <<	cos(p_)*cos(y_),	sin(r_)*sin(p_)*cos(y_) - cos(r_)*sin(y_),	cos(r_)*sin(p_)*cos(y_) + sin(r_)*sin(y_),
				cos(p_)*sin(y_),	sin(r_)*sin(p_)*sin(y_) + cos(r_)*cos(y_),	cos(r_)*sin(p_)*sin(y_) - sin(r_)*cos(y_),
				-sin(p_),			sin(r_)*cos(p_),							cos(r_)*cos(p_);

	/*F*/
	Eigen::VectorXd F(X.size());
	F.segment(0, 3) = X.segment(0, 3) + Rot_xyz*Dxyz;
	F.segment(3, 3) = X.segment(3, 3);

	/*jF*/
	Eigen::MatrixXd jF(X.size(), X.size());
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
	Eigen::MatrixXd Q = sigma*Eigen::MatrixXd::Identity(X.size(), X.size());
	
	/*Update*/
	X = F;
	P = jF*P*jF.transpose() + Q;

	/* std::cout << "jF =" << std::endl << jF << std::endl; */
	/* std::cout << "X =" << std::endl << X << std::endl; */
	/* std::cout << "P =" << std::endl << P << std::endl; */
}

void EdgeSLAMEKF::CallbackSLAMOdom(const nav_msgs::OdometryConstPtr& msg)
{
	/* std::cout << "Callback SLAM Odom" << std::endl; */
	
	// ObservationSLAMOdom(*msg);
	Publication();
}

void EdgeSLAMEKF::ObservationSLAMOdom(nav_msgs::Odometry slam_odom)
{
	/* std::cout << "Observation SLAM Odom" << std::endl; */

	tf::Quaternion q_pose(
		slam_odom.pose.pose.orientation.z,
		slam_odom.pose.pose.orientation.x,
		slam_odom.pose.pose.orientation.y,
		slam_odom.pose.pose.orientation.w
	);
	double r_slam, p_slam, y_slam;
	tf::Matrix3x3(q_pose).getRPY(r_slam, p_slam, y_slam);

	/*Z*/
	Eigen::VectorXd Z(6);
	Z <<	(double)slam_odom.pose.pose.position.z,
			(double)slam_odom.pose.pose.position.x,
			(double)slam_odom.pose.pose.position.y,
			r_slam,
			p_slam,
			y_slam;
	/*H, jH*/
	Eigen::MatrixXd H = Eigen::MatrixXd::Identity(Z.size(), X.size());
	Eigen::MatrixXd jH = H;
	/*R*/
	const double sigma = 1.0e-1;
	Eigen::MatrixXd R = sigma*Eigen::MatrixXd::Identity(Z.size(), Z.size());
	/*(Y, S, K, I)*/
	Eigen::VectorXd Y(Z.size());
	Eigen::MatrixXd S(Z.size(), Z.size());
	Eigen::MatrixXd K(X.size(), Z.size());
	Eigen::MatrixXd I = Eigen::MatrixXd::Identity(X.size(), X.size());
	/*update*/
	Y = Z - H*X;
	for(size_t i=3;i<6;i++)	Y(i) = PiToPi(Y(i));
	S = jH*P*jH.transpose() + R;
	K = P*jH.transpose()*S.inverse();
	X = X + K*Y;
	for(size_t i=3;i<6;i++)	X(i) = PiToPi(X(i));
	P = (I - K*jH)*P;

	std::cout << "Y =" << std::endl << Y << std::endl;
	std::cout << "K*Y =" << std::endl << K*Y << std::endl;
	std::cout << "P =" << std::endl << P << std::endl;
}

void EdgeSLAMEKF::Publication(void)
{
	/* std::cout << "Publication" << std::endl; */

	geometry_msgs::PoseStamped pose_pub = StateVectorToPoseStamped();
	pose_pub.header.frame_id = "/odom";
	// pose_pub.header.stamp = ros::Time::now();
	pose_pub.header.stamp = time_imu_now;
	pub_pose.publish(pose_pub);
}

geometry_msgs::PoseStamped EdgeSLAMEKF::StateVectorToPoseStamped(void)
{
	Eigen::VectorXd X_ = X + Reset_origin;
	for(int i=3;i<6;i++)	X_(i) = PiToPi(X_(i));

	geometry_msgs::PoseStamped pose;
	pose.pose.position.x = X_(0);
	pose.pose.position.y = X_(1);
	pose.pose.position.z = X_(2);
	tf::Quaternion q_orientation = tf::createQuaternionFromRPY(X_(3), X_(4), X_(5));
	pose.pose.orientation.x = q_orientation.x();
	pose.pose.orientation.y = q_orientation.y();
	pose.pose.orientation.z = q_orientation.z();
	pose.pose.orientation.w = q_orientation.w();

	std::cout << "X_ = " << std::endl << X_ << std::endl;
	std::cout << "pose.pose.orientation.x = " << pose.pose.orientation.x << std::endl;
	std::cout << "pose.pose.orientation.y = " << pose.pose.orientation.y << std::endl;
	std::cout << "pose.pose.orientation.z = " << pose.pose.orientation.z << std::endl;
	std::cout << "pose.pose.orientation.w = " << pose.pose.orientation.w << std::endl;

	return pose;
}

float EdgeSLAMEKF::PiToPi(double angle)
{
	return atan2(sin(angle), cos(angle));
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "edge_slam_ekf");
	std::cout << "Edge SLAM E.K.F." << std::endl;
	
	EdgeSLAMEKF edge_slam_ekf;
	ros::spin();
}
