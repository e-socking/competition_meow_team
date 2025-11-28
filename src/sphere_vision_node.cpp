#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point.hpp>
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


class sphere_node : public rclcpp::Node  //文件名作为类名
{
public:
    
    sphere_node(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s节点已经启动.", name.c_str());
        image_get=this->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw",10,
                             bind(&sphere_node::camera_image, this, std::placeholders::_1));
        Target_pub = this->create_publisher<referee_pkg::msg::MultiObject>(
        "/vision/target", 10);
    }

private:
//对象声明：
   rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_get;
   //subscription是接受信息的类，而接口会改变这个类让它对应接口的类
//回调
vector<Point2f> calculateStableSpherePoints(const Point2f &center, float radius);
   void camera_image(sensor_msgs::msg::Image::SharedPtr msg);
   
  rclcpp::Publisher<referee_pkg::msg::MultiObject>::SharedPtr Target_pub;
   vector<Point2f> sphere_point;
   vector<Point2f> Point_V;
};


//-------------------------------------------------main--------------



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<sphere_node>("sphere_node");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);




    rclcpp::shutdown();
    return 0;
}



//--------------------------求点坐标函数


vector<Point2f> sphere_node::calculateStableSpherePoints(const Point2f &center,
                                                      float radius) {//输入Point2f类型的中心点，半径
  vector<Point2f> points;

  // 简单稳定的几何计算，避免漂移
  // 左、下、右、上
  points.push_back(Point2f(center.x - radius, center.y));  // 左点 (1)
  points.push_back(Point2f(center.x, center.y + radius));  // 下点 (2)
  points.push_back(Point2f(center.x + radius, center.y));  // 右点 (3)
  points.push_back(Point2f(center.x, center.y - radius));  // 上点 (4)

  return points;
}



void sphere_node::camera_image(sensor_msgs::msg::Image::SharedPtr msg){
try{             //有问题就报错。try内是主体内容

int valid_spheres = 0; 
  //-----------------------传入--------------------


  // 图像转换到cv——ptr
    cv_bridge::CvImagePtr cv_ptr;


   
if(msg->encoding=="rgb8"||msg->encoding=="R8G8B8"){ //判断色彩格式是否为rgb
  Mat bgr_image,image_bgr;
  //创建矩阵名为image，把msg信息存入。参数分别为：行，列，图像格式（rgb3通道），指向图像像素信息的指针
  Mat imagepre(msg->height, msg->width, CV_8UC3,
                    const_cast<unsigned char *>(msg->data.data()));
  
  cvtColor(imagepre,image_bgr,COLOR_RGB2BGR);

  cv_ptr = cv_bridge::CvImagePtr(  // 直接初始化 CvImagePtr
    new cv_bridge::CvImage(
        msg->header,  // 传入 header
        "bgr8",       // 传入编码格式
        image_bgr     // 传入转换后的有效图像（注意：之前你误写为 bgr_image，这里需同步修正）
    )
);
}else {//本句是以牺牲部分性能为代价，转换msg的任意格式为bgr格式
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }



//---------------------预处理---------------------------------


    Mat image=cv_ptr->image;//-----获取
     Mat hsv;
    if(image.empty()){
      RCLCPP_WARN(this->get_logger(), "Received empty image");
      return;
    }
   
    Mat result_image=image.clone();//clone是为了隔绝两个对象避免干扰
    cvtColor(image,hsv,COLOR_BGR2HSV);//注意：转成了hsv
    
    

// 红色检测 - 使用稳定的范围--------颜色处理
    cv::Mat mask1, mask2, mask;
    cv::inRange(hsv, Scalar(0, 120, 70), Scalar(10, 255, 255), mask1);
    cv::inRange(hsv, Scalar(170, 120, 70), Scalar(180, 255, 255),
                mask2);
    mask = mask1 | mask2;



     // 适度的形态学操作------------形状的瑕疵处理
    cv::Mat kernel =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));//指定椭圆用于处理圆形图像
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);//填充内部小空洞
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);//去除外部噪点



    //------------------------正式处理--------------------------


        Point_V.clear();//-----清除之前存下的圆的四个特征点的数据。对应下文的：calculateStableSpherePoints(center, radius);

    // -----找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    //------球上画点
    for(size_t i = 0;i<contours.size();i++){//size_t对应contours.size()的数据类型
        double area = contourArea(contours[i]);
        if(area<500) continue;//太小就跳过循环
          // 计算最小外接圆
      Point2f center;
      float radius = 0;
      minEnclosingCircle(contours[i], center, radius);//输出中心和半径

      // 计算曲度
      double perimeter = arcLength(contours[i], true);
      double circularity = 4 * CV_PI * area / (perimeter * perimeter);
      //获取关键点坐标
      if (circularity > 0.7 && radius > 15 && radius < 200) {
        vector<Point2f> sphere_points =//符合条件，调用此函数（已定义）
            calculateStableSpherePoints(center, radius);

             // -------------绘制球体上的信息
        cv::circle(result_image, center, static_cast<int>(radius),
                   cv::Scalar(0, 255, 0), 2);  // 绿色圆圈，static_cast<int>是强制转换，因为circle要求半径是整数
        cv::circle(result_image, center, 3, cv::Scalar(0, 0, 255),
                   -1);  // 红色圆心

              // -------------绘制球体边上的信息
        vector<string> point_names = {"左", "下", "右", "上"};//--定义绘制方式
        vector<Scalar> point_colors = {
            Scalar(255, 0, 0),    // 蓝色 - 左
            Scalar(0, 255, 0),    // 绿色 - 下
            Scalar(0, 255, 255),  // 黄色 - 右
            Scalar(255, 0, 255)   // 紫色 - 上
        };
         //--绘制
        for (int j = 0; j < 4; j++) {
          cv::circle(result_image, sphere_points[j], 6, point_colors[j], -1);
          cv::circle(result_image, sphere_points[j], 6, cv::Scalar(0, 0, 0), 2);
          //参数诠释：处理的图像，点坐标，半径，颜色，填充/轮廓宽度
          
          // 标注序号
          string point_text = to_string(j + 1);//把j+1后转变成字符串格式
          putText(
              result_image, point_text,
              Point(sphere_points[j].x + 10, sphere_points[j].y - 10),
              FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 3);
          putText(
              result_image, point_text,
              Point(sphere_points[j].x + 10, sphere_points[j].y - 10),
              FONT_HERSHEY_SIMPLEX, 0.6, point_colors[j], 2);
              //0.6是字体缩放比例
         
         
              //--------------------结果发送--------------------
              
              
              // 添加到发送列表
          Point_V.push_back(sphere_points[j]);

          RCLCPP_INFO(this->get_logger(),
                      "Sphere %d, Point %d (%s): (%.1f, %.1f)",
                      valid_spheres + 1, j + 1, point_names[j].c_str(),
                      sphere_points[j].x, sphere_points[j].y);
         
    }
     // --------显示半径信息
        string info_text = "R:" + to_string((int)radius);
        cv::putText(
            result_image, info_text, cv::Point(center.x - 15, center.y + 5),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
       valid_spheres++;
        RCLCPP_INFO(this->get_logger(),
                    "Found sphere: (%.1f, %.1f) R=%.1f C=%.3f", center.x,
                    center.y, radius, circularity);//计算曲度时cir赋值
        }
      }
      // -----显示结果图像
    cv::imshow("Detection Result", result_image);
    cv::waitKey(1);


    //--------------------------------------发送
    // 创建并发布消息
    referee_pkg::msg::MultiObject msg_object;//多物体类型，详见大赛文档
    msg_object.header = msg->header;
    msg_object.num_objects = Point_V.size() / 4;

    vector<string> types = {"sphere"};

    for (int k = 0; k < msg_object.num_objects; k++) {
      referee_pkg::msg::Object obj;
      obj.target_type = (k < types.size()) ? types[k] : "unknown";//基础的判断语句，若k正常，即只有一个，就赋值"sphere"给obj.target_type

      for (int j = 0; j < 4; j++) {
        int index = 4 * k + j;
        if (index < Point_V.size()) {
          geometry_msgs::msg::Point corner;
          corner.x = Point_V[index].x;
          corner.y = Point_V[index].y;
          corner.z = 0.0;
          obj.corners.push_back(corner);//在obj的对象数组obj.corners后面再加上一个
        }
      }

      msg_object.objects.push_back(obj);
    }

    Target_pub->publish(msg_object);
    RCLCPP_INFO(this->get_logger(), "Published %d sphere targets",
                msg_object.num_objects);

  }catch (const cv_bridge::Exception &e) {     //与try配合
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());//ros到opencv数据传送错误
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());//普通错误
  }
}