#include <cv_bridge/cv_bridge.h>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <referee_pkg/msg/multi_object.hpp>
#include <referee_pkg/msg/object.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>
#include "sensor_msgs/msg/image.hpp"

using namespace std;
using namespace rclcpp;
using namespace cv;

class rectNode : public rclcpp::Node {
 public:
  rectNode(string name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "Initializing rectNode");

    Image_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        bind(&rectNode::callback_camera, this, std::placeholders::_1));

    Target_pub = this->create_publisher<referee_pkg::msg::MultiObject>(
        "/vision/target", 10);

    cv::namedWindow("Detection Result", cv::WINDOW_AUTOSIZE);

    RCLCPP_INFO(this->get_logger(), "rectNode initialized successfully");
  }

  ~rectNode() { cv::destroyWindow("Detection Result"); }

 private:
  void callback_camera(sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr Image_sub;
  rclcpp::Publisher<referee_pkg::msg::MultiObject>::SharedPtr Target_pub;
  vector<Point2f> Point_V;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rectNode>("rectNode");
  RCLCPP_INFO(node->get_logger(), "Starting rectNode");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

void rectNode::callback_camera(sensor_msgs::msg::Image::SharedPtr msg) {
  try {
    // 图像转换
    cv_bridge::CvImagePtr cv_ptr;

    if (msg->encoding == "rgb8" || msg->encoding == "R8G8B8") {
      cv::Mat image(msg->height, msg->width, CV_8UC3,
                    const_cast<unsigned char *>(msg->data.data()));
      cv::Mat bgr_image;
      cv::cvtColor(image, bgr_image, cv::COLOR_RGB2BGR);
      cv_ptr = std::make_shared<cv_bridge::CvImage>();
      cv_ptr->header = msg->header;
      cv_ptr->encoding = "bgr8";
      cv_ptr->image = bgr_image;
    } else {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }

    cv::Mat image = cv_ptr->image;

    if (image.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty image");
      return;
    }

    // 创建结果图像
    cv::Mat result_image = image.clone();

    // 转换到 HSV 空间
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
//cv::imshow("hsv",hsv);
    // 绿色检测 - 使用绿色范围
    cv::Mat mask;
    cv::inRange(hsv, cv::Scalar(35, 50, 50), cv::Scalar(100, 255, 255), mask);
//cv::imshow("testhsv",mask);
    // 适度的形态学操作
    cv::Mat kernel =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);


  //cv::imshow("testmask",mask);


    // 找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    Point_V.clear();
    int valid_rectangles = 0;

    for (size_t i = 0; i < contours.size(); i++) {
      double area = cv::contourArea(contours[i]);
      if (area < 500) continue;

      // 使用多边形逼近轮廓
      std::vector<cv::Point> approx;
      cv::approxPolyDP(contours[i], approx, 0.02 * cv::arcLength(contours[i], true), true);

      // 如果是长方形（四个顶点）
      if (approx.size() == 4 && cv::isContourConvex(approx)) {
        // 获取四个顶点
        vector<cv::Point2f> rect_points;
        for (int j = 0; j < 4; j++) {
          rect_points.push_back(approx[j]);
        }

        // 绘制长方形
        for (int j = 0; j < 4; j++) {
          cv::line(result_image, rect_points[j], rect_points[(j + 1) % 4], cv::Scalar(0, 255, 0), 2);
        }

        // 绘制四个角点（红色）
        for (int j = 0; j < 4; j++) {
          cv::circle(result_image, rect_points[j], 6, cv::Scalar(0, 0, 255), -1);
          Point_V.push_back(rect_points[j]);

          // 标注序号
          string point_text = to_string(j + 1);
          cv::putText(result_image, point_text, cv::Point(rect_points[j].x + 10, rect_points[j].y - 10),
                      cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
        }

        valid_rectangles++;
        RCLCPP_INFO(this->get_logger(), "Found rectangle: (%.1f, %.1f, %.1f, %.1f)", rect_points[0].x,
                    rect_points[0].y, rect_points[2].x, rect_points[2].y);
      }
    }

    // 显示结果图像
    cv::imshow("Detection Result", result_image);
    cv::waitKey(1);

    // 创建并发布消息
    referee_pkg::msg::MultiObject msg_object;
    msg_object.header = msg->header;
    msg_object.num_objects = Point_V.size() / 4;

    vector<string> types = {"rect"};

    for (int k = 0; k < msg_object.num_objects; k++) {
      referee_pkg::msg::Object obj;
      obj.target_type = (k < types.size()) ? types[k] : "unknown";

      for (int j = 0; j < 4; j++) {
        int index = 4 * k + j;
        if (index < Point_V.size()) {
          geometry_msgs::msg::Point corner;
          corner.x = Point_V[index].x;
          corner.y = Point_V[index].y;
          corner.z = 0.0;
          obj.corners.push_back(corner);
        }
      }

      msg_object.objects.push_back(obj);
    }

    Target_pub->publish(msg_object);
    RCLCPP_INFO(this->get_logger(), "Published %d rectangle targets", msg_object.num_objects);

  } catch (const cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
  }
}
