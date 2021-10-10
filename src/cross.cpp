#include <iostream>
using namespace std;
#include <opencv2/opencv.hpp>
using namespace cv;
#include "ros/ros.h"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include "CBR_2021/H_info.h"
#include "std_msgs/Bool.h"

#define AREA_THRESH 0.008
#define PI 3.14159265

#define vp vector<Point>
#define vpf vector<Point2f>

// Set true for debugging purposes, showing internals of algorithm
#define DEBUG false

// Sorts points based on y coordinate
struct comparison
{
  bool operator()(Point2f pt1, Point2f pt2) { return (pt1.y > pt2.y); }
} comparing;

class HDetector
{
private:
  float x_center;
  float y_center;
  float area_ratio;
  ros::NodeHandle n;
  void image_cb(const sensor_msgs::ImageConstPtr &img);
  void runnin_state_cb(std_msgs::Bool data);
  ros::Publisher h_pub;
  ros::Publisher img_debug_pub;
  ros::Subscriber h_sub_image;
  ros::Subscriber h_sub_runner;
  bool runnin = 0;

public:
  HDetector();
  bool detect(Mat frame);
  float getArea();
  void setArea(float area);
  int getCenterX();
  void setCenterX(float x);
  int getCenterY();
  void setCenterY(float y);
};

HDetector::HDetector()
{
  this->h_pub = this->n.advertise<CBR_2021::H_info>("/precision_landing/detection", 0);
  this->img_debug_pub = this->n.advertise<sensor_msgs::Image>("/precision_landing/debug/image_raw", 0);
  // this->h_sub_image = this->n.subscribe("/uav1/bluefox_optflow/image_raw", 5, &HDetector::image_cb, this);
  this->h_sub_image = this->n.subscribe("/uav1/bluefox_optflow/image_raw", 5, &HDetector::image_cb, this);
  this->h_sub_runner = this->n.subscribe("/precision_landing/set_running_state", 10, &HDetector::runnin_state_cb, this);
  cout << "Iniciou cross.cpp " << endl;
}

void HDetector::runnin_state_cb(std_msgs::Bool data)
{
  this->runnin = data.data;
}

void HDetector::image_cb(const sensor_msgs::ImageConstPtr &img)
{
  if (this->runnin) 
  {

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    CBR_2021::H_info msg;
    if (this->detect(cv_ptr->image))
    {
      msg.detected = true;
      msg.center_x = this->getCenterX();
      msg.center_y = this->getCenterY();
      msg.area_ratio = this->getArea();
      
      
      //cout << "Centro em x: " << msg.center_x << endl;
      //cout << "Centro em y: " << msg.center_y << endl;
      //cout << "Area ratio " << msg.area_ratio << endl;
      

      this->h_pub.publish(msg);
    }
    else{
      msg.detected = false;
      msg.center_x = -1;
      msg.center_y = -1;
      msg.area_ratio = 0;
  }
      
   }
}

float HDetector::getArea()
{
  return this->area_ratio;
}

void HDetector::setArea(float area)
{
  this->area_ratio = area;
}

int HDetector::getCenterX()
{
  return this->x_center;
}

void HDetector::setCenterX(float x)
{
  this->x_center = x;
}

int HDetector::getCenterY()
{
  return this->y_center;
}

void HDetector::setCenterY(float y)
{
  this->y_center = y;
}

// Takes an image 'frame' and detects whether it contains the letter H
bool HDetector::detect(Mat frame)
{
  bool detected = false;

  Mat hsvFrame, maskFrame, maskChannels[3], bgrFrame;
  //cvtColor(frame, hsvFrame, COLOR_BGR2HSV);
  // Blur and threshold remove noise from image
  // medianBlur(hsvFrame, hsvFrame, 11);
  // GaussianBlur(hsvFrame, hsvFrame, Size(9, 9), 1.0);
  bilateralFilter(frame, bgrFrame, 0, 100, 0);
  cvtColor(bgrFrame, hsvFrame, COLOR_BGR2HSV);
  // inRange(hsvFrame, Scalar(30, 150, 0), Scalar(70, 255, 255), hsvFrame);
  inRange(hsvFrame, Scalar(20, 25, 0), Scalar(35, 255, 255), maskFrame);

  if (!maskFrame.empty())
    imshow("mask", maskFrame);

  vector<vp> contours;
  findContours(maskFrame, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

  int max_w = 0,
      max_h = 0, max_x = 0, max_y = 0;
  Rect rect;

  for (vp cnt : contours)
  {
    rect = boundingRect(cnt);
    if (rect.width * rect.height > max_w * max_h)
    {
      max_w = rect.width;
      max_h = rect.height;
      max_x = rect.x;
      max_y = rect.y;
    }
    rectangle(frame, Point(rect.x, rect.y), Point(rect.x + rect.width, rect.y + rect.height), Scalar(0, 255, 0), 1);
  }

  rectangle(frame, Point(max_x, max_y),
            Point(max_x + max_w, max_y + max_h), Scalar(0, 0, 255), 2);
  circle(frame, Point((max_x + max_w / 2), (max_y + max_h / 2)),
         1, (0, 0, 255), 3);
  imshow("frame", frame);
  waitKey(3); // Wait for a keystroke in the window

  if (DEBUG)
  {
    imshow("Processed", maskFrame);
    waitKey(3); // Wait for a keystroke in the window
  }
  // cout << max_w * max_h << endl;
  if ((max_w * max_h > frame.rows * frame.cols * AREA_THRESH )&& (max_w < (max_h * 2)) && max_w > (max_h*0.4))
  {

    detected = true;
    this->setCenterX(max_x + max_w / 2);
    this->setCenterY(max_y + max_h / 2);
    this->setArea((max_w * max_h * 1.0) / (frame.rows * frame.cols) );
  }
  // detected = true;
  // this->setCenterX(max_x + max_w / 2);
  // this->setCenterY(max_y + max_h / 2);
  // this->setArea(max_w * max_h);

  return detected;
}

// For testing
int main(int argc, char **arvg)
{
  ROS_INFO("Running cross detection node!");
  ros::init(argc, arvg, "cross_node");
  HDetector *detector = new HDetector();
  ros::spin();
}
