#include<rclcpp/rclcpp.hpp>
#include<opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/msg/image.hpp"
#include <std_msgs/msg/header.hpp>
#include <opencv2/core.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <referee_pkg/msg/multi_object.hpp>
#include <referee_pkg/msg/object.hpp>

#include<algorithm>//角点稳定处理
#include <string>

//#include "referee_pkg/msg/race_stage.hpp"
//#include "referee_pkg/srv/hit_arror.hpp"

using namespace cv;
using namespace rclcpp;


class armor_node : public rclcpp::Node
{
    public:
    armor_node(std::string name):Node(name){
    RCLCPP_INFO(get_logger(),"节点armor启动%s",name.c_str());
    get_ptr = create_subscription<sensor_msgs::msg::Image>("/camera/image_raw",10,
                                         std::bind(&armor_node::get_camera,this,std::placeholders::_1));
   //像这样函数的定义要写在构造函数里，class里一般只能声明
    Target_pub = this->create_publisher<referee_pkg::msg::MultiObject>(
        "/vision/target", 10 );
    this->declare_parameter<int>("h_min",0);
    this->declare_parameter<int>("s_min",0);
    this->declare_parameter<int>("v_min",0);
    this->declare_parameter<int>("h_max",255);
    this->declare_parameter<int>("s_max",30);
    this->declare_parameter<int>("v_max",50);
   
    
// stage_sub_ = this->create_subscription<referee_pkg::msg::RaceStage>(
//             "/referee/race_stage",
//             10,
//             std::bind(&RefereeNode::stage_callback, this, _1));

//  // 2. 声明服务端（初始未激活，阶段5时启动）
//         hit_armor_srv_ = nullptr;




  }
    
    private:

// rclcpp::Subscription<referee_pkg::msg::RaceStage>::SharedPtr stage_sub_;
//     rclcpp::Service<referee_pkg::srv::HitArmor>::SharedPtr hit_armor_srv_;



    std::string version;
    Subscription<sensor_msgs::msg::Image>::SharedPtr get_ptr;
    void get_camera(sensor_msgs::msg::Image::SharedPtr msg);
    std::vector<Point2f> Point_V;
    Publisher<referee_pkg::msg::MultiObject>::SharedPtr Target_pub;
    int h_min,s_min,v_min,h_max,s_max,v_max;
 
} ;



//点坐标调整函数
void mark_point(Point2f* armor_point,RotatedRect rect){//注意格式,计算点位置
//注意：此函数仅适用于正放目标 

//--------计算点位置
float height = rect.size.height;
float up = height*0.243;
float sub = height*0.757;

//---------点赋值
armor_point[0]=Point2f(armor_point[0].x,armor_point[0].y-up);//要加上Point2f来构造
armor_point[1]=Point2f(armor_point[1].x,armor_point[1].y-up);
armor_point[2]=Point2f(armor_point[2].x,armor_point[2].y+up);
armor_point[3]=Point2f(armor_point[3].x,armor_point[3].y+up);
//不知为何，-+up的符号和我预想中相反
}


//---------回调
void armor_node::get_camera(sensor_msgs::msg::Image::SharedPtr msg)
{   
try{
    this->get_parameter("h_min",h_min);
   this->get_parameter("s_min",s_min);
   this->get_parameter("v_min",v_min);
   this->get_parameter("h_max",h_max);
   this->get_parameter("s_max",s_max);
   this->get_parameter("v_max",v_max);
  //------窗口初始化：
   

        static bool window_init = false;
        if(!window_init){
            //cv::namedWindow("hsv Result", cv::WINDOW_NORMAL);
            cv::namedWindow("mask", cv::WINDOW_NORMAL);
            cv::namedWindow("result", cv::WINDOW_NORMAL);
            cv::namedWindow("dis", cv::WINDOW_NORMAL);
            window_init = true;
           }

            


//-------------------------预处理--------------

Mat hsv,result;
cv_bridge::CvImagePtr image_ptr;

//------bgr

if(msg->encoding=="R8G8B8"||msg->encoding=="rgb8"){
    Mat bgr;
    image_ptr = std::make_shared<cv_bridge::CvImage>();
    Mat image(msg->height, msg->width, CV_8UC3,
                    const_cast<unsigned char *>(msg->data.data()));//ros2信息传入
    cvtColor(image,bgr,COLOR_RGB2BGR);
    image_ptr->image = bgr;
    image_ptr->header= msg->header;
    image_ptr->encoding = "bgr8";
}else{//异常措施处理
          image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
}
if(image_ptr->image.empty()){RCLCPP_WARN(get_logger(), "空信息");//异常措施
}

//---------hsv

cvtColor(image_ptr->image,hsv,COLOR_BGR2HSV);
//imshow("hsv Result",hsv);//test

// ----------------黑色检测，遮照
Mat mask;
inRange(hsv, cv::Scalar(h_min,s_min,v_min), cv::Scalar(h_max,s_max,v_max), mask);

cv::Mat kernel =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

  //  imshow("mask",mask);//(test)
  
// -----------------找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);



//-----------------------------正式处理-------------------


result=image_ptr->image.clone();
  Point_V.clear();
 //----找点
for(size_t i=0;i<contours.size();i++){//遍历
  Point2f armor_point[4];
 RotatedRect rect;
  Point2f rect_point[4];
  Point2f sorted_corners[4];  // 排序后的稳定角点（左上→右上→右下→左下）
  double area = cv::contourArea(contours[i]);
  if(area<=500 ) {continue;}//噪点
 rect= minAreaRect(contours[i]);//计算外接矩形(可以是倾斜的)
  

 rect.points(rect_point);//把外接矩形的角点给到rect_point，无序


  // ------------------- 角点稳定化：按物理位置重新排序 （因为minAreaRect返回无序角点
    // 1. 找到左上（TL）：x+y 最小
    auto tl_it = std::min_element(rect_point, rect_point+4,
        [](const Point2f& a, const Point2f& b) { return (a.x + a.y) < (b.x + b.y); });
    sorted_corners[3] = *tl_it;  // sorted_corners[0] = 左上

    // 2. 找到右上（TR）：x-y 最大
    auto tr_it = std::max_element(rect_point, rect_point+4,
        [](const Point2f& a, const Point2f& b) { return (a.x - a.y) < (b.x - b.y); });
    sorted_corners[2] = *tr_it;  // sorted_corners[1] = 右上

    // 3. 找到右下（BR）：x+y 最大
    auto br_it = std::max_element(rect_point, rect_point+4,
        [](const Point2f& a, const Point2f& b) { return (a.x + a.y) < (b.x + b.y); });
    sorted_corners[1] = *br_it;  // sorted_corners[2] = 右下

    // 4. 找到左下（BL）：x-y 最小
    auto bl_it = std::min_element(rect_point, rect_point+4,
        [](const Point2f& a, const Point2f& b) { return (a.x - a.y) < (b.x - b.y); });
    sorted_corners[0] = *bl_it;  // sorted_corners[3] = 左下

 
 
   mark_point(sorted_corners,rect);//计算点位置,传数组不用&
 
 
 
  //----画矩形测试（--------测试）
 Mat display_img = image_ptr->image.clone();
 for(int j=0;j<4;j++){//画轮廓
 line(display_img,rect_point[j],rect_point[j+1],Scalar(255, 0, 0), 2);


 }
 imshow("dis",display_img);


 for(int j=0;j<4;j++){//遍历四个点
  //--------------图上标记----------
  //-----画点
circle(result, sorted_corners[j], 5, cv::Scalar(0,0,255), 4);
// ----标注序号
  std::string point_text = std::to_string(j+1);
  cv::putText(
  result, point_text,
  cv::Point(sorted_corners[j].x + 10, sorted_corners[j].y - 10),
  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 3);

  Point_V.push_back(sorted_corners[j]);

 }
}


//------------------存入和发布-----------------------------

 //-------------信息存入----------
    referee_pkg::msg::MultiObject msg_object;
    msg_object.header = msg->header;//obj的header存入
    msg_object.num_objects = Point_V.size() / 4;//obj数量存入

  std::vector<std::string> types = {"armor_red_1"};

    for (int k = 0; k < msg_object.num_objects; k++) {//此循环判定装甲板被识别
      referee_pkg::msg::Object obj;
      obj.target_type = (k < types.size()) ? types[k] : "unknown";//----物体类型赋值
    //-----------corner的角点坐标赋值
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
      //--------物体赋值
      msg_object.objects.push_back(obj);
    }

    //-----------------------发布---------------
    Target_pub->publish(msg_object);




//-------测试+输出：
imshow("result",result);

waitKey(1);

}catch (const cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
  }
}





int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    
    auto node = std::make_shared<armor_node>("armor_node");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);

    



    rclcpp::shutdown();
    return 0;
}







 


    // // 击打服务回调函数（处理客户端请求）
    // void hit_armor_callback(
    //     const std::shared_ptr<referee_pkg::srv::HitArmor::Request> req,
    //     std::shared_ptr<referee_pkg::srv::HitArmor::Response> res)
    // {
    //     // 1. 解析请求参数
    //     RCLCPP_INFO(this->get_logger(), "收到击打请求：");
    //     RCLCPP_INFO(this->get_logger(), "重力加速度g：%.2f", req->g);
    //     RCLCPP_INFO(this->get_logger(), "模型真值点数量：%ld", req->modelpoint.size());
    //     for (size_t i = 0; i < req->modelpoint.size(); i++)
    //     {
    //         RCLCPP_INFO(this->get_logger(), "点%d：(%.2f, %.2f, %.2f)", 
    //                     i+1, req->modelpoint[i].x, req->modelpoint[i].y, req->modelpoint[i].z);
    //     }

    //     // 2. 计算欧拉角（示例逻辑，需替换为实际算法）
    //     // 此处仅为示例，实际需根据模型点和重力加速度计算
    //     res->yaw = 0.15;    // 偏航角（弧度）
    //     res->pitch = 0.08;  // 俯仰角（弧度）
    //     res->roll = 0.02;   // 翻滚角（弧度）

    //     RCLCPP_INFO(this->get_logger(), "返回欧拉角：yaw=%.2f, pitch=%.2f, roll=%.2f",
    //                 res->yaw, res->pitch, res->roll);
    // }


