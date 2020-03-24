
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <dynamic_reconfigure/server.h>
#include <myRace/normalizerConfig.h>

using namespace cv;
using namespace std;

namespace enc = sensor_msgs::image_encodings;

static const char WINDOW[] = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  image_transport::Publisher image_pub_red_;
  dynamic_reconfigure::Server<myRace::normalizerConfig> server;
  dynamic_reconfigure::Server<myRace::normalizerConfig>::CallbackType f;
  
public:
  int bgr_low ;
  int b_r_low ;  
  ImageConverter()
    : it_(nh_)
  {
    image_pub_ = it_.advertise("/normalized_image", 1);
    image_pub_red_ = it_.advertise("/normalized_image_red", 1);
    image_sub_ = it_.subscribe("/axis_videocap/image_raw", 1, &ImageConverter::imageCb, this);

    f = boost::bind(&ImageConverter::callback, this,_1, _2);
    server.setCallback(f);
   

    // cv::namedWindow(WINDOW);
  }

  ~ImageConverter()
  {
    // cv::destroyWindow(WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      // cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
    
    cv::Mat gray(cv_ptr->image.rows , cv_ptr->image.cols, CV_8UC1, cv::Scalar(0));
    cv::Mat gray_red(cv_ptr->image.rows , cv_ptr->image.cols, CV_8UC1, cv::Scalar(0));
    
    for(int y = 0; y < cv_ptr->image.rows; y++){
        for(int x = 0; x < cv_ptr->image.cols; x++){
            cv::Vec3b & bgrPixel = cv_ptr->image.at<cv::Vec3b>(y,x);
            double b = bgrPixel[0];
            double g = bgrPixel[1];
       
            double r = bgrPixel[2];
            // cout << r << endl;
            double v1 = 0;
            double v2 = 0;
            if(b+g+r>this->bgr_low and ((r>this->b_r_low) or (b>this->b_r_low))){
                v1 = r/(r+g+b);
                v2 = b/(r+g+b); 
            }
            else{
                v1 = 0;
                v2 = 0;
            }
            gray_red.at<uchar>(Point(x,y)) = round(v1*255);
            // cout << v1 << endl;
            if(v1>v2)
                gray.at<uchar>(Point(x,y)) = round(v1*255); 
            else
                gray.at<uchar>(Point(x,y)) = round(v2*255);
        }
        // ROS_INFO("Reconfigure Request: %d %d", 
           // this->bgr_low, this->b_r_low);
        
    }

    // cv::imshow(WINDOW, gray);
    // cv::waitKey(3);
    // cv_ptr->image = gray;
    // sensor_msgs::ImagePtr gray_msg; 
    // gray_msg = cv_bridge::CvImage(std_msgs::Header(),"8UC1",gray).toImageMsg();
    // img1 = cv_bridge::CvImage(std_msgs::Header(),"8UC1",gray);
    // image_pub_.publish(cv_ptr->toImageMsg);
    cv_bridge::CvImage gray_msg;
    gray_msg.header.stamp = ros::Time::now();
    gray_msg.header.frame_id = "gray_img";
    gray_msg.encoding = "8UC1";
    gray_msg.image = gray;
    sensor_msgs::Image img_msg;
    gray_msg.toImageMsg(img_msg);
    image_pub_.publish(img_msg);
    gray_msg.image = gray_red;
    gray_msg.toImageMsg(img_msg);
    image_pub_red_.publish(img_msg);
  }
  void callback(myRace::normalizerConfig &config, uint32_t level) {
    
     //       config.str_param.c_str(), 
      //      config.bool_param?"True":"False", 
       //     config.size);
    this->b_r_low = config.normalize_r_b;
    this->bgr_low = config.normalize_bgr;
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
