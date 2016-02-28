#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h> // conversion between ROS and openCV image data types
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/contrib/contrib.hpp> // color map
#include "visual_contact/Matrix.h" // custom mesage for matrix data
#include "dynamic_reconfigure/server.h"
#include "visual_contact/visual_contactConfig.h" // configuration for dynamically reconfigurable parameters


class VisualContactPerspective{
private:
    ros::NodeHandle nh_;

    ros::Subscriber image_sub_; // camera stream
    ros::Publisher image_pub_, image_pub2_; // optical flow, TTC maps
    ros::Publisher pub_ttcmatrix_; // matrix message

    visual_contact::Matrix ttc_matrix_;

    cv_bridge::CvImagePtr cv_ptr;

    cv::Mat flow, cflow, gray, prevgray, uflow;
    cv::Mat ttcMap;
    cv::Mat falseColorsMap;

    float u0, v0;

    // Optical flow algorithm paramters (dynamically reconfigurable)
    double GF_pyr_scale_;
    int GF_levels_;
    int GF_winsize_;
    int GF_iterations_;
    int GF_poly_n_;
    double GF_poly_s_;
    int GF_disp_scale_;
    int GF_disp_step_;
    int TTC_disp_scale_;

    dynamic_reconfigure::Server<visual_contact::visual_contactConfig> server;
    dynamic_reconfigure::Server<visual_contact::visual_contactConfig>::CallbackType f;

public:
    VisualContactPerspective(); // constructor
    ~VisualContactPerspective(); // destructor

    // camera subscriber callback
    void imageCb(const sensor_msgs::ImageConstPtr&);

    // dynamic reconfigure callback
    void drcallback(visual_contact::visual_contactConfig&, uint32_t);

    // optical flow + TTC computations
    void computeTTC();

    // optical flow vector over image
    void drawOptFlowMap(const cv::Scalar&);
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ttc_perspective");
    VisualContactPerspective vcc;
    ros::spin();
    return 0;
}

// constructor (node handle in private namespace through parameter constructor in initializer list)
VisualContactPerspective::VisualContactPerspective() : nh_("~"){
    // subscrive to input video feed
    image_sub_ = nh_.subscribe("/camera/image_raw", 1, &VisualContactPerspective::imageCb, this);

    // publish TTC, optical flow maps visualization
    // Note: decided against using image transport since using the raw stream anyway,
    // clutters topics and parameters list, and probably better
    // for optical flow estimation also)
    image_pub_ = nh_.advertise<sensor_msgs::Image>("optical_flow", 1);
    image_pub2_ = nh_.advertise<sensor_msgs::Image>("ttc_map", 1);

    // publish computed TTC matrix
    pub_ttcmatrix_ = nh_.advertise<visual_contact::Matrix>("ttc_matrix",1);

    // dynamic reconfigure callback
    f = boost::bind(&VisualContactPerspective::drcallback, this, _1, _2);
    server.setCallback(f);
}


VisualContactPerspective::~VisualContactPerspective()
{
    // empty destructor
}


void VisualContactPerspective::imageCb(const sensor_msgs::ImageConstPtr& msg){
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8); // converting to grayscale here itself (no need to conver later in processing loop)
        // using copy though could use shared pointer, since data is not modified in place
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // clearing before filling in next processing loop
    ttc_matrix_.array.clear();

    // initializing here since size determined after receiving image, although it should be done
    // just once, and not in a callback
    ttcMap = cv::Mat::zeros(cv_ptr->image.rows, cv_ptr->image.cols, CV_32F);

    // using center of image currently, change to include user input (for example, calibrated
    // cameras)
    u0 = floor(cv_ptr->image.cols/2);
    v0 = floor(cv_ptr->image.rows/2);

    computeTTC();

    // add header fields - time stamp?
    cv_bridge::CvImage CvCflow(std_msgs::Header(), "bgr8", cflow);
    image_pub_.publish(CvCflow.toImageMsg());

    cv_bridge::CvImage CvFalseColorsMap(std_msgs::Header(), "bgr8", falseColorsMap);
    image_pub2_.publish(CvFalseColorsMap.toImageMsg());

    // time stamp on matrix header message (used for example, in depth map node)
    ttc_matrix_.header.stamp = ros::Time::now();
    pub_ttcmatrix_.publish(ttc_matrix_);
}



void VisualContactPerspective::computeTTC(){
    // variables initialized in each processing loop, memory management taken care of
    // by itself?
    float f1, f2, f;

    cv_ptr->image.copyTo(gray);

    ttc_matrix_.height = gray.rows;
    ttc_matrix_.width = gray.cols;

    if( !prevgray.empty() )
    {
        cv::calcOpticalFlowFarneback(prevgray, gray, uflow, GF_pyr_scale_, GF_levels_, GF_winsize_, GF_iterations_, GF_poly_n_, GF_poly_s_, 0);

        cv::cvtColor(prevgray, cflow, cv::COLOR_GRAY2BGR);
        uflow.copyTo(flow);
        drawOptFlowMap(cv::Scalar(0, 255, 0));

        for (int ii=0; ii<uflow.rows; ii++){
            for (int jj=0; jj<uflow.cols; jj++){
                f1 = (jj-u0)*(jj-u0) + (ii-v0)*(ii-v0);
                f2 = (uflow.at<cv::Vec2f>(ii,jj)[0])*(uflow.at<cv::Vec2f>(ii,jj)[0]) + (uflow.at<cv::Vec2f>(ii,jj)[1])*(uflow.at<cv::Vec2f>(ii,jj)[1]);
                f = sqrt(f1/f2);

                if (!( isinf(f) | isnan(f) ) ){
                    ttcMap.at<float>(ii,jj) = f;
                }

                // matrix contains values as computed, should be processed later even for
                // absolute values
                ttc_matrix_.array.push_back(ttcMap.at<float>(ii,jj));

            }
        }

        // absolute values used
        ttcMap = cv::abs(ttcMap);

        cv::Mat adjMap;

        ttcMap.convertTo(adjMap, CV_8UC1, 255.0/TTC_disp_scale_);

        applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_JET);
    }
    // why swap, could just copy gray to prevgray, right?
    std::swap(prevgray, gray);
}


void VisualContactPerspective::drawOptFlowMap(const cv::Scalar& color){
    // optical flow maps sampled scaled for visualization
    for(int y = 0; y < cflow.rows; y += GF_disp_step_)
        for(int x = 0; x < cflow.cols; x += GF_disp_step_)
        {
            const cv::Point2f& fxy = flow.at<cv::Point2f>(y, x);
            cv::line(cflow, cv::Point(x,y), cv::Point(cvRound(x+ GF_disp_scale_*fxy.x), cvRound(y+ GF_disp_scale_*fxy.y)),
                     color);
            cv::circle(cflow, cv::Point(x,y), 2, color, -1);
        }
}


void VisualContactPerspective::drcallback(visual_contact::visual_contactConfig& config, uint32_t level){
    // update class member datab
    GF_pyr_scale_ = config.GF_pyr_scale;;
    GF_levels_ = config.GF_levels;
    GF_winsize_ = config.GF_winsize;
    GF_iterations_ = config.GF_iterations;
    GF_poly_n_ = config.GF_poly_n;
    GF_poly_s_ = config.GF_poly_s;
    GF_disp_scale_ = config.GF_disp_scale;
    GF_disp_step_ = config.GF_disp_step;
    TTC_disp_scale_ = config.TTC_disp_scale;
}

