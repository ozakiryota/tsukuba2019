#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

class ImuBiasEstimation{
	private:
		ros::NodeHandle nh;
		/*subscribe*/
		ros::Subscriber sub_imu;
		/*publish*/
		ros::Publisher pub_bias;
		/*const*/
		const double timelimit = 30.0;	//[s]
		/*objects*/
		std::vector<sensor_msgs::Imu> record;
		sensor_msgs::Imu average;
		ros::Time time_started;
		/*flags*/
		bool imu_is_moving = false;
		bool initial_algnment_is_done = false;
	public:
		ImuBiasEstimation();
		void Callback(const sensor_msgs::ImuConstPtr& msg);
		void ComputeAverage(void);
		bool JudgeMoving(void);
		void Publication(void);
};

ImuBiasEstimation::ImuBiasEstimation()
{
	sub_imu = nh.subscribe("/imu/data", 1, &ImuBiasEstimation::Callback, this);
	pub_bias = nh.advertise<sensor_msgs::Imu>("/imu/bias", 1);
}

void ImuBiasEstimation::Callback(const sensor_msgs::ImuConstPtr& msg)
{
	if(!initial_algnment_is_done){
		double time;
		if(record.size()<10){
			time = 0.0;
			time_started = ros::Time::now();
		}
		else{
			try{
				time = (ros::Time::now() - time_started).toSec();
			}
			catch(std::runtime_error& ex) {
				ROS_ERROR("Exception: [%s]", ex.what());
			}

			imu_is_moving = JudgeMoving();
		}
		
		if(imu_is_moving || time>timelimit){
			initial_algnment_is_done = true;
			if(time>timelimit)	std::cout << "time > " << timelimit << "[s]" << std::endl;
			else	std::cout << "Moved at " << time << "[s]" << std::endl;
			std::cout << "bias = " << std::endl << average.angular_velocity.x << std::endl << average.angular_velocity.y << std::endl << average.angular_velocity.z << std::endl;
		}
		else{
			record.push_back(*msg);
			ComputeAverage();
		}
	}
	else	Publication();
}

void ImuBiasEstimation::ComputeAverage(void)
{
	average.angular_velocity.x = 0.0;
	average.angular_velocity.y = 0.0;
	average.angular_velocity.z = 0.0;
	average.linear_acceleration.x = 0.0;
	average.linear_acceleration.y = 0.0;
	average.linear_acceleration.z = 0.0;
	
	for(size_t i=0;i<record.size();i++){
		average.angular_velocity.x += record[i].angular_velocity.x/(double)record.size();
		average.angular_velocity.y += record[i].angular_velocity.y/(double)record.size();
		average.angular_velocity.z += record[i].angular_velocity.z/(double)record.size();
		average.linear_acceleration.x += record[i].linear_acceleration.x/(double)record.size();
		average.linear_acceleration.y += record[i].linear_acceleration.y/(double)record.size();
		average.linear_acceleration.z += record[i].linear_acceleration.z/(double)record.size();
	}
}

bool ImuBiasEstimation::JudgeMoving(void)
{
	const float threshold_w = 0.05;
	const float threshold_a = 0.5;

	if(fabs(record[record.size()-1].angular_velocity.x - average.angular_velocity.x)>threshold_w){
		std::cout << "Moved-wx: " << record[record.size()-1].angular_velocity.x - average.angular_velocity.x << std::endl;
		return true;
	}
	if(fabs(record[record.size()-1].angular_velocity.y - average.angular_velocity.y)>threshold_w){
		std::cout << "Moved-wy: " << record[record.size()-1].angular_velocity.y - average.angular_velocity.y << std::endl;
		return true;
	}
	if(fabs(record[record.size()-1].angular_velocity.z - average.angular_velocity.z)>threshold_w){
		std::cout << "Moved-wz: " << record[record.size()-1].angular_velocity.z - average.angular_velocity.z << std::endl;
		return true;
	}
	if(fabs(record[record.size()-1].linear_acceleration.x - average.linear_acceleration.x)>threshold_a){
		std::cout << "Moved-ax: " << record[record.size()-1].linear_acceleration.x - average.linear_acceleration.x << std::endl;
		return true;
	}
	if(fabs(record[record.size()-1].linear_acceleration.y - average.linear_acceleration.y)>threshold_a){
		std::cout << "Moved-ay: " << record[record.size()-1].linear_acceleration.y - average.linear_acceleration.y << std::endl;
		return true;
	}
	if(fabs(record[record.size()-1].linear_acceleration.z - average.linear_acceleration.z)>threshold_a){
		std::cout << "Moved-az: " << record[record.size()-1].linear_acceleration.z - average.linear_acceleration.z << std::endl;
		return true;
	}
	return false;
}

void ImuBiasEstimation::Publication(void)
{
	/*publish*/
	average.header = record[record.size()-1].header;
	pub_bias.publish(average);
}

int main(int argc, char**argv)
{
	ros::init(argc, argv, "imu_bias_estimation");

	ImuBiasEstimation imu_bias_estimation;

	ros::spin();
}
