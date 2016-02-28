#include "ros/ros.h"
#include <opencv2/imgproc/imgproc.hpp>
#include "geometry_msgs/Twist.h" // control velocity
#include "visual_contact/Matrix.h" // ttc matrix data
#include "std_msgs/Float32.h" // output ttc metric

class VisualBraking{
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_vel_;
    ros::Publisher pub_ttcMean_;
    ros::Subscriber subs_matrix_;

    geometry_msgs::Twist twist_;
    std_msgs::Float32 ttcMean_;

    cv::Mat ttcMap_;
    cv::Mat ttcMapBlur_;

    int ttc_count_;
    int ttcCount_thresh_;
    int ttcMean_thresh_;
    int wSize_;
    double cmd_vel_;

public:
    VisualBraking();
    ~VisualBraking();

    void controlLaw(const visual_contact::MatrixConstPtr&);

};


int main(int argc, char** argv){
    ros::init(argc, argv, "visual_braking_perspective");

    VisualBraking vB;

    ros::spin();

    return 0;
}


VisualBraking::VisualBraking() : nh_("~") {
    pub_vel_ = nh_.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 1);
    pub_ttcMean_ = nh_.advertise<std_msgs::Float32>("ttc_mean", 1);
    subs_matrix_ = nh_.subscribe("/ttc_perspective/ttc_matrix",1,&VisualBraking::controlLaw,this);

    ttc_count_ = 0;

    // Read parameters if specified (such as in launch file), otherwise setting default values
    if (!(nh_.getParam("ttcCount_thresh",ttcCount_thresh_))){
        ttcCount_thresh_ = 5;
    }

    if (!(nh_.getParam("ttcMean_thresh",ttcMean_thresh_))){
        ttcMean_thresh_ = 100;
    }

    if (!(nh_.getParam("wSize",wSize_))){
        wSize_ = 200;
    }

    if (!(nh_.getParam("cmd_vel",cmd_vel_))){
        cmd_vel_ = 0.1;
    }

}


VisualBraking::~VisualBraking(){
    // empty destructor
}


void VisualBraking::controlLaw(const visual_contact::MatrixConstPtr& data){
    // ---------get map------

    ttcMap_ = cv::Mat::zeros(data->height,data->width,CV_32F);

    std::vector<float>::const_iterator it=data->array.begin();

    for (int ii=0;ii<ttcMap_.rows;ii++){
        for (int jj=0;jj<ttcMap_.cols;jj++){
            ttcMap_.at<float>(ii,jj) = *it;
            it++;
        }
    }

    std::cout<<"Here"<<std::endl;
    // ---------- process map ----------

    ttcMapBlur_ = ttcMap_(cv::Range((data->height-wSize_)/2, (data->height+wSize_)/2), cv::Range((data->width-wSize_)/2, (data->width+wSize_)/2));
    cv::medianBlur(ttcMapBlur_, ttcMapBlur_, 5);
    cv::medianBlur(ttcMapBlur_, ttcMapBlur_, 5);

    cv::Scalar ttcMean = cv::mean(ttcMapBlur_);
    ROS_INFO("Mean TTC:%f",ttcMean[0]);

    ttcMean_.data = ttcMean[0];

    if (ttcMean[0]<ttcMean_thresh_){
        ttc_count_++;
    }


    // ---------- control law ----------
    //

    if (ttc_count_<ttcCount_thresh_){
        // continue motion
        twist_.linear.x = cmd_vel_;
    }
    else{
        // brake
        twist_.linear.x = 0.0;
    }

    pub_ttcMean_.publish(ttcMean_);
    pub_vel_.publish(twist_);

}


