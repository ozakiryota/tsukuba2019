#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nmea_msgs/Sentence.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

class CombineGPSPositionOrientation{
	private:
		/*node handle*/
		ros::NodeHandle nh;
		ros::NodeHandle nhPrivate;
		/*subscriber*/
		ros::Subscriber sub_odom;
		ros::Subscriber sub_sentence;
		/*publisher*/
		ros::Publisher pub_odom;
		tf::TransformBroadcaster tf_broadcaster;
		/*objects*/
		nav_msgs::Odometry odom_pub;
		/*flags*/
		bool inipose_is_available = false;
		/*fram_id*/
		std::string parent_frame_id_name;
		std::string child_frame_id_name;
	public:
		CombineGPSPositionOrientation();
		void CallbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void CallbackSentence(const nmea_msgs::SentenceConstPtr& msg);
		std::vector<std::string> SplitSentence(std::string sentence, std::string delimiter);
		double NorthBaseToUTM(double angle);
		void Publication(void);
		double DegToRad(double angle);
		double PiToPi(double angle);
};

CombineGPSPositionOrientation::CombineGPSPositionOrientation()
	: nhPrivate("~")
{
	sub_odom = nh.subscribe("/gps/odom", 1, &CombineGPSPositionOrientation::CallbackOdom, this);
	sub_sentence = nh.subscribe("nmea_sentence", 1, &CombineGPSPositionOrientation::CallbackSentence, this);
	pub_odom = nh.advertise<nav_msgs::Odometry>("/gps/odom/with_orientation", 1);

	nhPrivate.param("parent_frame_id", parent_frame_id_name, std::string("/odom"));
	nhPrivate.param("child_frame_id", child_frame_id_name, std::string("/gps/odom/with_orientation"));
}

void CombineGPSPositionOrientation::CallbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	odom_pub = *msg;
	odom_pub.header.frame_id = parent_frame_id_name;
	odom_pub.child_frame_id = child_frame_id_name;
}

void CombineGPSPositionOrientation::CallbackSentence(const nmea_msgs::SentenceConstPtr& msg)
{
	const std::string type = "$GPVTG";
	const int index = 1;
	if(msg->sentence.find(type.c_str()) != std::string::npos){
		std::vector<std::string> splited_sentence = SplitSentence(msg->sentence, std::string(","));
		double yaw = DegToRad( std::stod(splited_sentence[index]) );
		/* yaw += M_PI; //convet to utm */
		yaw = NorthBaseToUTM(yaw);
		yaw = PiToPi(yaw);
    	tf::Quaternion q = tf::createQuaternionFromRPY(0.0, 0.0, yaw);
    	quaternionTFToMsg(q, odom_pub.pose.pose.orientation);

		Publication();

		/* std::cout << msg->sentence << std::endl; */
		std::cout << "splited_sentence[index] = " << splited_sentence[index].c_str() << std::endl;
		std::cout << "yaw[deg] = " << yaw/M_PI*180.0 << std::endl;
	}
}

std::vector<std::string> CombineGPSPositionOrientation::SplitSentence(std::string sentence, std::string delimiter)
{
	std::vector<std::string> words;
	size_t position = 0;
	/* size_t next; */
	while(sentence.find(delimiter.c_str(), position) != std::string::npos){
		size_t next = sentence.find(delimiter.c_str(), position);
		std::string word = sentence.substr(position, next-position);
		position = next + 1;
		words.push_back(word);
		/* std::cout << "word.c_str() = " << word.c_str() << std::endl; */
	}
	std::string last_word = sentence.substr(position, sentence.length()-1-position);
	words.push_back(last_word);
	/* std::cout << "last_word.c_str() = " << last_word.c_str() << std::endl; */

	return words;
}

double CombineGPSPositionOrientation::NorthBaseToUTM(double angle)
{
	return -(angle - M_PI/2);
}

void CombineGPSPositionOrientation::Publication(void)
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

double CombineGPSPositionOrientation::DegToRad(double angle)
{
	return angle/180.0*M_PI;
}

double CombineGPSPositionOrientation::PiToPi(double angle)
{
	return atan2(sin(angle), cos(angle)); 
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "combine_gps_position_orientation");

	CombineGPSPositionOrientation combine_gps_position_orientation;

	ros::spin();
}
