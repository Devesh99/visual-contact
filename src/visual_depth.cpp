#include "ros/ros.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "nav_msgs/Odometry.h"
#include "visual_contact/Matrix.h"
#include "sensor_msgs/Image.h"

class VisualDepthNode{
private:
    ros::NodeHandle nh_;
    ros::Subscriber subs_odom_;
    ros::Subscriber subs_ttc_;

    ros::Publisher pub_depth_map_;
    ros::Publisher pub_depth_matrix_;

    float lin_x_;
    cv::Mat ttcDepthMap_;
    visual_contact::Matrix depth_matrix_;
    cv::Mat adjMap_, falseColorsMap_;

    double time_diff_;
    double time_prev_;

public:
    VisualDepthNode();
    ~VisualDepthNode();

    void depthMap(const visual_contact::MatrixConstPtr&);
    void getOdom(const nav_msgs::OdometryConstPtr&);
};


int main(int argc, char** argv){

    ros::init(argc, argv, "visual_depth");
    VisualDepthNode vD;
    ros::spin();

    return 0;
}


VisualDepthNode::VisualDepthNode() : nh_("~") {
    subs_odom_ = nh_.subscribe("/RosAria/pose",1, &VisualDepthNode::getOdom, this);
    subs_ttc_ = nh_.subscribe("/ttc_perspective/ttc_matrix",1,&VisualDepthNode::depthMap,this);

    pub_depth_map_ = nh_.advertise<sensor_msgs::Image>("depth_map",1);
    pub_depth_matrix_ = nh_.advertise<visual_contact::Matrix>("depth_matrix",1);

    lin_x_ = 0;
    time_prev_ = 0;
    time_diff_ = 0;
}


VisualDepthNode::~VisualDepthNode(){
    // empty destructor
}


void VisualDepthNode::getOdom(const nav_msgs::OdometryConstPtr &odom){
    lin_x_ = odom->twist.twist.linear.x;
}


void VisualDepthNode::depthMap(const visual_contact::MatrixConstPtr &data){
    ttcDepthMap_ = cv::Mat::zeros(data->height,data->width,CV_32F);
    depth_matrix_.array.clear();

    depth_matrix_.width = data->width;
    depth_matrix_.height = data->height;

    double curr_sec = data->header.stamp.sec;
    double curr_nsec = data->header.stamp.nsec/(1000000000.0);
    time_diff_ = curr_sec + curr_nsec - time_prev_;
    time_prev_ = curr_sec + curr_nsec;

    std::cout<<time_diff_<<std::endl;

    std::vector<float>::const_iterator it=data->array.begin();

    for (int ii=0;ii<ttcDepthMap_.rows;ii++){
        for (int jj=0;jj<ttcDepthMap_.cols;jj++){
            ttcDepthMap_.at<float>(ii,jj) = 100*((*it)*time_diff_)*lin_x_; // in cm
            depth_matrix_.array.push_back(ttcDepthMap_.at<float>(ii,jj));
            it++;
        }
    }

    ttcDepthMap_ = cv::abs(ttcDepthMap_);

    ttcDepthMap_.convertTo(adjMap_,CV_8UC1);
    cv::applyColorMap(adjMap_, falseColorsMap_, cv::COLORMAP_JET);

    cv_bridge::CvImage CvFalseColorsMap(std_msgs::Header(), "bgr8", falseColorsMap_);
    pub_depth_map_.publish(CvFalseColorsMap.toImageMsg());

    depth_matrix_.header.stamp = ros::Time::now();
    pub_depth_matrix_.publish(depth_matrix_);
}
