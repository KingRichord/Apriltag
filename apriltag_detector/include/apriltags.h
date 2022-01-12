#include <tf2_ros/transform_broadcaster.h>
#include <apriltag_mit/apriltag_mit.h>
#include <apriltag_msgs/Apriltag.h>
#include <apriltag_umich/apriltag_umich.h>



const double DEFAULT_TAG_SIZE =0.075;
const std::string DEFAULT_DISPLAY_TYPE = "CUBE";
const std::string DEFAULT_CAMERA_TOPIC_NAME = "/usb_cam/image_raw";
const std::string DEFAULT_CAMERA_INFO_NAME = "/usb_cam/camera_info";
sensor_msgs::CameraInfo camera_info_;
ros::Subscriber camera_info_sub;
image_transport::Subscriber image_sub;
ros::Publisher marker_pub;
ros::Publisher apriltag_publisher;

// shared_ptr<HexagonDetector> detector_;
// 相机图像话题
std::string camera_topic_name_;
// 相机info信息订阅
std::string camera_info_topic_name_;

//是否进行显示
bool viewer_;
//显示
bool debug_;
//默认的tag大小
double default_tag_size_;
//检测画面中显示图像的边框
double marker_thickness_;
//检测频率
double process_hz_;

//是否收到camera_info信息
bool has_camera_info_;
//自定义可视化显示的信息
std::string display_type_;

//可视化显示的信息列表
bool display_marker_overlay_;
bool display_marker_outline_;
bool display_marker_id_;
bool display_marker_edges_;
bool display_marker_axes_;

// apriltag
int TagFamily_;
// 用哪一种方式解码
bool USE_UMICH_;
// 线程数量
int nthreads_;
// 边界框的大小
int black_border_;
// 增强模式
double quad_decimate_;

apriltag_umich::TagFamilyPtr umich_tag_family_;
apriltag_umich::TagDetectorPtr umich_tag_detector_;
apriltag_mit::TagDetectorPtr mit_tag_detector_;

float Get_TagFamily(){return TagFamily_;}
int Get_black_border(){return black_border_;}