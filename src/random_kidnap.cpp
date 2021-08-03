#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>

class RandomKidnap{
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Timer timer_;
        ros::Publisher pub_;

        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener tfListener_;

        std::string map_frame_;
        std::string base_frame_;

        double kidnap_length_;

        double timer_duration_, examine_duration_;
        int count_;

    public:
        RandomKidnap(): nh_(), pnh_("~"), tfBuffer_(), tfListener_(tfBuffer_), count_(0){
            ROS_INFO("start random kidnap node");
            pnh_.param<std::string>("map_frame", map_frame_, "map");
            pnh_.param<std::string>("base_frame", base_frame_, "base_link");
            pnh_.param<double>("timer_duration", timer_duration_, 0.1);
            pnh_.param<double>("examine_duration", examine_duration_, 10.0);
            pnh_.param<double>("kidnap_length", kidnap_length_, 50.0);

            timer_ = nh_.createTimer(ros::Duration(timer_duration_),&RandomKidnap::timerCallback,this);
            pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped> ("initialpose",1000);

        }
        
        void timerCallback(const ros::TimerEvent& e);
};

void
RandomKidnap::timerCallback(const ros::TimerEvent& e){
    geometry_msgs::TransformStamped trans;
    try{
        trans = tfBuffer_.lookupTransform(map_frame_, base_frame_, ros::Time(0));
    }
    catch (tf2::TransformException& ex){
        ROS_WARN("%s", ex.what());
        return;
    }

    count_++;

    if(count_ >= (examine_duration_ / timer_duration_) ){
        geometry_msgs::PoseWithCovarianceStamped kidnap_pose;

        double length = 2*((double)rand()/RAND_MAX - 0.5)*kidnap_length_;
		double direction = 2*((double)rand()/RAND_MAX - 0.5)*M_PI;

        kidnap_pose.header.frame_id = map_frame_;
        kidnap_pose.header.stamp = ros::Time(0);
        kidnap_pose.pose.pose.position.x = trans.transform.translation.x + length*std::cos(direction);
        kidnap_pose.pose.pose.position.y = trans.transform.translation.y + length*std::sin(direction);
        kidnap_pose.pose.pose.position.z = 0;

        tf2::Quaternion q;
		q.setRPY(0, 0, direction);
		tf2::convert(q, kidnap_pose.pose.pose.orientation);

        kidnap_pose.pose.covariance[0] = kidnap_pose.pose.covariance[7] = 0.25;
		kidnap_pose.pose.covariance[35] = 0.7;

        pub_.publish(kidnap_pose);
        ROS_INFO("publish kidnap_pose");
        count_ = 0;
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "random_kidnap");
    RandomKidnap rk;
    ros::spin();

    return 0;
}