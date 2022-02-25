#ifndef APRILTAG_DETECTOR_MANAGER_H
#define APRILTAG_DETECTOR_MANAGER_H

#include <ros/ros.h>

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point32.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <std_msgs/Bool.h>

#include <apriltag_mit/apriltag_mit.h>
#include <apriltag_msgs/Apriltag.h>
#include <apriltag_umich/apriltag_umich.h>

#include <dbrobot_msg/QRTagDetections.h>
#include <tf2_ros/transform_broadcaster.h>

const double DEFAULT_TAG_SIZE =0.075;
const std::string DEFAULT_DISPLAY_TYPE = "CUBE";
const std::string DEFAULT_CAMERA_TOPIC_NAME = "/usb_cam/image_raw";
const std::string DEFAULT_CAMERA_INFO_NAME = "/usb_cam/camera_info";
enum class DetectorType { Mit, Umich };
enum class TagFamily { tf36h11, tf25h9, tf16h5 };
using ApriltagVec = std::vector<apriltag_msgs::Apriltag>;

class manager
{
public:
    manager(ros::NodeHandle &node, image_transport::ImageTransport &image):m_node(node),m_image(image),tag_enable(false)
    {
        //读取launch文件信息
        m_node.param("camera_topic", camera_topic_name_, DEFAULT_CAMERA_TOPIC_NAME);
        m_node.param("camera_info", camera_info_topic_name_, DEFAULT_CAMERA_INFO_NAME);

        m_node.param("default_tag_size", default_tag_size_, DEFAULT_TAG_SIZE);
        m_node.param("display_type", display_type_, DEFAULT_DISPLAY_TYPE);
        m_node.param("marker_thickness", marker_thickness_, 0.01);
        m_node.param("process_hz", process_hz_, 10.0);
        m_node.param("viewer", viewer_, false);
        m_node.param("debug", debug_, false);

        m_node.param("display_marker_overlay", display_marker_overlay_, true);
        m_node.param("display_marker_outline", display_marker_outline_, true);
        m_node.param("display_marker_id", display_marker_id_, true);
        m_node.param("display_marker_edges", display_marker_edges_, true);
        m_node.param("display_marker_axes", display_marker_axes_, true);
        m_node.param("display_marker_axes", display_marker_axes_, true);

        m_node.param("TagFamily", TagFamily_, 2);
        m_node.param("USE_UMICH", USE_UMICH_, false);
        m_node.param("num_threads", nthreads_, 1);
        m_node.param("black_border", black_border_, 1);
        m_node.param("quad_decimate", quad_decimate_, 1.0);

        // 注册回调信息
        tag_enable_sub = node.subscribe("tag_enable", 1, &manager::tag_enable_Callback, this);
        has_camera_info_ = false;
        ros::Rate loop_rate(process_hz_);
        while (m_node.ok()) {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    ~manager()
    {
        stop();
        tag_enable_sub.shutdown();
    }

    float Get_TagFamily(){return TagFamily_;}
    int Get_black_border(){return black_border_;}
    void GetMarkerTransformUsingOpenCV(apriltag_msgs::Apriltag& detection, Eigen::Matrix4d& transform, cv::Mat& rvec, cv::Mat& tvec) {
        // Check if fx,fy or cx,cy are not set
        if ((camera_info_.K[0] == 0.0) || (camera_info_.K[4] == 0.0) || (camera_info_.K[2] == 0.0) ||
            (camera_info_.K[5] == 0.0)) {
            ROS_WARN("Warning: Camera intrinsic matrix K is not set, can't recover 3D pose");
        }
        std::vector<cv::Point3f> object_pts;
        std::vector<cv::Point2f> image_pts;
        double tag_radius = default_tag_size_ / 2.;
        //这样设计是以二维码中点为中心
        object_pts.emplace_back(-tag_radius,  -tag_radius, 0.0);
        object_pts.emplace_back( tag_radius,  -tag_radius, 0.0);
        object_pts.emplace_back( tag_radius,   tag_radius, 0.0);
        object_pts.emplace_back(-tag_radius,   tag_radius, 0.0);
        image_pts.emplace_back((float)detection.corners[0].x,(float )detection.corners[0].y);
        image_pts.emplace_back((float)detection.corners[1].x,(float )detection.corners[1].y);
        image_pts.emplace_back((float)detection.corners[2].x,(float )detection.corners[2].y);
        image_pts.emplace_back((float)detection.corners[3].x,(float )detection.corners[3].y);
        // 这里反畸变操作的基本流程
        // 将图像中的点投影到归一化平面
        for (auto &pt : image_pts) {
            double fx = camera_info_.K[0];
            double fy = camera_info_.K[4];
            double cx = camera_info_.K[2];
            double cy = camera_info_.K[5];

            double k1 = camera_info_.D[0];
            double k2 = camera_info_.D[1];
            double p1 = camera_info_.D[2];
            double p2 = camera_info_.D[3];

            // Inverse camera projection matrix parameters
            double m_inv_K11, m_inv_K13, m_inv_K22, m_inv_K23;
            m_inv_K11 = 1.0 / fx;
            m_inv_K13 = -cx / fx;
            m_inv_K22 = 1.0 / fy;
            m_inv_K23 = -cy / fy;

            Eigen::Vector2d P2(pt.x, pt.y);
            Eigen::Vector3d P3;
            liftProjective(P2, P3, m_inv_K11, m_inv_K13, m_inv_K22, m_inv_K23, k1, k2, p1, p2);

            Eigen::Vector2d image_pt;
            pt.x = fx * (P3[0]) + cx;
            pt.y = fy * (P3[1]) + cy;
        }  //畸变修正结束
        cv::Mat_<double> cameraMatrix(3, 3);
        cameraMatrix << camera_info_.K[0], 0, camera_info_.K[2], 0, camera_info_.K[4], camera_info_.K[5], 0, 0, 1;
        cv::Mat_<double> distortion(4, 1);
        // distortion << camera_info_.D[0], camera_info_.D[1], camera_info_.D[2], camera_info_.D[3];
        distortion << 0, 0, 0, 0;
        // new_in_parm = cv::getOptimalNewCameraMatrix(intrinsics, distortion_coeff, cv::Size(640,400), 1);
        // cv::undistortPoints(image_pts, image_pts_new, new_in_parm, distortion,cv::noArray(), new_in_parm);
        int method = CV_ITERATIVE;
        bool use_extrinsic_guess = false;  // only used for ITERATIVE method

        auto good = cv::solvePnP(object_pts, image_pts, cameraMatrix, distortion, rvec, tvec, use_extrinsic_guess, method);
        if (!good) {
            ROS_WARN(" Pose solver failed");
        }
        // detection.observedPerimeter =
        //(float)getReprojectionError(object_pts, image_pts, cameraMatrix, distortion, rvec, tvec);
        detection.hamming = calcFiducialArea(image_pts);
        cv::Matx33d r;
        cv::Rodrigues(rvec, r);
        Eigen::Matrix3d rot;
        rot << r(0, 0), r(0, 1), r(0, 2), r(1, 0), r(1, 1), r(1, 2), r(2, 0), r(2, 1), r(2, 2);

        Eigen::Matrix4d T;
        T.topLeftCorner(3, 3) = rot;
        T.col(3).head(3) << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
        T.row(3) << 0, 0, 0, 1;
        // transformation camera -> world
        // Eigen::Vector3d t_wc = -T.block<3, 3>(0, 0).transpose() * T.block<3, 1>(0, 3);
        // std::cout << "cam pos in world:" << t_wc.transpose() << std::endl;
        transform = T;
    }
    void ArrowLine(cv::Mat& image, const cv::Point& pt1, const cv::Point& pt2, const cv::Scalar& color,
                   const int thickness = 1, const int line_type = 8, const int shift = 0, const double tip_length = 0.1) {
        // Normalize the size of the tip depending on the length of the arrow
        const double tip_size = norm(pt1 - pt2) * tip_length;

        cv::line(image, pt1, pt2, color, thickness, line_type, shift);
        const double angle = atan2(double(pt1.y - pt2.y), double(pt1.x - pt2.x));
        cv::Point p(cvRound(pt2.x + tip_size * cos(angle + CV_PI / 4.0)),
                    cvRound(pt2.y + tip_size * sin(angle + CV_PI / 4.0)));
        cv::line(image, p, pt2, color, thickness, line_type, shift);
        p.x = cvRound(pt2.x + tip_size * cos(angle - CV_PI / 4.0));
        p.y = cvRound(pt2.y + tip_size * sin(angle - CV_PI / 4.0));
        cv::line(image, p, pt2, color, thickness, line_type, shift);
    }
    void DrawMarkerAxes(const cv::Matx33f& intrinsic_matrix, const cv::Vec4f& distortion_coeffs, const cv::Mat& rvec,
                        const cv::Mat& tvec, const float length, const bool use_arrows, cv::Mat& image) {
        // cout << "enter drawMarkerAxes!------>" << endl;
        std::vector<cv::Point3f> axis_points;
        axis_points.emplace_back(0, 0, 0);
        axis_points.emplace_back(length, 0, 0);
        axis_points.emplace_back(0, length, 0);
        axis_points.emplace_back(0, 0, length);
        std::vector<cv::Point2f> image_points;
        cv::projectPoints(axis_points, rvec, tvec, intrinsic_matrix, distortion_coeffs, image_points);
        // Draw axis lines
        const int thickness = 1;
        if (use_arrows) {
            ArrowLine(image, image_points[0], image_points[1], cv::Scalar(0, 0, 255), thickness);
            ArrowLine(image, image_points[0], image_points[2], cv::Scalar(0, 255, 0), thickness);
            ArrowLine(image, image_points[0], image_points[3], cv::Scalar(255, 0, 0), thickness);
        } else {
            cv::line(image, image_points[0], image_points[1], cv::Scalar(0, 0, 255), thickness);
            cv::line(image, image_points[0], image_points[2], cv::Scalar(0, 255, 0), thickness);
            cv::line(image, image_points[0], image_points[3], cv::Scalar(255, 0, 0), thickness);
        }
    }
    void DrawMarkerOutline(const apriltag_msgs::Apriltag& detection, const cv::Scalar& outline_color, cv::Mat& image) {
        // Draw outline
        const int outline_thickness = 2;
        for (int i = 0; i < 4; i++) {
            cv::Point2f p0(detection.corners[i].x, detection.corners[i].y);
            cv::Point2f p1(detection.corners[(i + 1) % 4].x, detection.corners[(i + 1) % 4].y);
            cv::line(image, p0, p1, outline_color, outline_thickness);
        }
        // Indicate first corner with a small rectangle
        const int width = 6;
        const int rect_thickness = 1;
        cv::Point2f p0(detection.corners[0].x - width / 2, detection.corners[0].y - width / 2);
        cv::Point2f p1(detection.corners[0].x + width / 2, detection.corners[0].y + width / 2);
        cv::rectangle(image, p0, p1, outline_color, rect_thickness, 0);  // anti-aliased
    }
    void DrawMarkerEdges(const apriltag_msgs::Apriltag& detection, cv::Mat& image) {
        // Draw edges
        std::vector<cv::Scalar> colors;
        colors.emplace_back(0, 0, 255);    // red (BGR ordering)
        colors.emplace_back(0, 255, 0);    // green
        colors.emplace_back(255, 0, 0);    // blue
        colors.emplace_back(0, 204, 255);  // yellow
        const int edge_thickness = 2;
        for (int i = 0; i < 4; i++) {
            cv::Point2f p0(detection.corners[i].x, detection.corners[i].y);
            cv::Point2f p1(detection.corners[(i + 1) % 4].x, detection.corners[(i + 1) % 4].y);
            cv::line(image, p0, p1, colors[i], edge_thickness);
        }
    }
    void DrawMarkerID(const apriltag_msgs::Apriltag& detection, const cv::Scalar& text_color, cv::Mat& image) {
        cv::Point2f center(0, 0);
        for (int i = 0; i < 4; i++) {
            center += cv::Point2f(detection.corners[i].x, detection.corners[i].y);
        }
        center.x = center.x / 4.0;
        center.y = center.y / 4.0;
        std::stringstream s;
        s << "  " << detection.id;  // move label away from origin
        const double font_scale = 1;
        const int thickness = 2;
        cv::putText(image, s.str(), center, cv::FONT_HERSHEY_SIMPLEX, font_scale, text_color, thickness);
    }
    void tag_enable_Callback(const std_msgs::BoolConstPtr &msg)
    {
        if(msg->data == true){
            tag_enable = true;
            stop();
            start();
        }
        if(msg->data == false){
            tag_enable = false;
            stop();
        }
    }
    void InfoCallback(const sensor_msgs::CameraInfoConstPtr& camera_info) {
        // ROS_INFO("received camera info");
        camera_info_ = (*camera_info);
        has_camera_info_ = true;
    }
    void start()
    {
        ROS_WARN("finsh already！");
        //强枚举类型
        TagFamily tag_family{static_cast<TagFamily>(TagFamily_)};
        //两种模式
        if (USE_UMICH_) {
            umich_tag_detector_.reset(apriltag_detector_create());
            switch (tag_family) {
                case TagFamily::tf36h11:
                    umich_tag_family_.reset(tag36h11_create());
                    break;
                case TagFamily::tf25h9:
                    umich_tag_family_.reset(tag25h9_create());
                    break;
                case TagFamily::tf16h5:
                    umich_tag_family_.reset(tag16h5_create());
                    break;
                default:
                    throw std::invalid_argument("Invalid tag family");
            }
            umich_tag_family_->black_border = black_border_;
            umich_tag_detector_->quad_decimate = quad_decimate_;
            umich_tag_detector_->nthreads = nthreads_;
            umich_tag_detector_->quad_sigma =0.3;
            apriltag_detector_add_family(umich_tag_detector_.get(), umich_tag_family_.get());
        } else {
            switch (tag_family) {
                case TagFamily::tf36h11:
                    mit_tag_detector_ = boost::make_shared<apriltag_mit::TagDetector>(apriltag_mit::tag_codes_36h11);
                    break;
                case TagFamily::tf25h9:
                    mit_tag_detector_ = boost::make_shared<apriltag_mit::TagDetector>(apriltag_mit::tag_codes_25h9);
                    break;
                case TagFamily::tf16h5:
                    mit_tag_detector_ = boost::make_shared<apriltag_mit::TagDetector>(apriltag_mit::tag_codes_16h5);
                    break;
                default:
                    throw std::invalid_argument("Invalid tag family");
            }
            mit_tag_detector_->set_black_border(black_border_);
        }
        //订阅相机info
        camera_info_sub = m_node.subscribe(camera_info_topic_name_, 1, &manager::InfoCallback,this);
        //订阅图像相机
        image_sub = m_image.subscribe(camera_topic_name_, 1,  &manager::ImageCallback, this);
        marker_pub = m_node.advertise<visualization_msgs::MarkerArray>("marker_array", 1);
        apriltag_publisher = m_node.advertise<dbrobot_msg::QRTagDetections>("detections", 1);
    }
    void stop()
    {
        if (USE_UMICH_)
        {
            //umich方法中只要其中一个不为空就执行清空
            if(umich_tag_detector_ == nullptr && umich_tag_family_ == nullptr)
            {
                ROS_INFO("finsh already！");
            }
            else
            {
                //关闭所有的订阅发布
                camera_info_sub.shutdown();
                image_sub.shutdown();
                marker_pub.shutdown();
                apriltag_publisher.shutdown();

                umich_tag_family_.reset(nullptr);
                umich_tag_detector_.reset(nullptr);
            }
        }
        else
        {
            if(mit_tag_detector_ == nullptr)
            {
                ROS_INFO("finsh already！");
            } else
            {
                camera_info_sub.shutdown();
                image_sub.shutdown();
                marker_pub.shutdown();
                apriltag_publisher.shutdown();

                mit_tag_detector_.reset();
            }
        }
    }
private:
    // Euclidean distance between two points
    double dist(const cv::Point2f& p1, const cv::Point2f& p2) {
        double x1 = p1.x;
        double y1 = p1.y;
        double x2 = p2.x;
        double y2 = p2.y;
        double dx = x1 - x2;
        double dy = y1 - y2;
        return sqrt(dx * dx + dy * dy);
    }
    double calcFiducialArea(const std::vector<cv::Point2f>& pts) {
        const cv::Point2f& p0 = pts.at(0);
        const cv::Point2f& p1 = pts.at(1);
        const cv::Point2f& p2 = pts.at(2);
        const cv::Point2f& p3 = pts.at(3);

        double a1 = dist(p0, p1);
        double b1 = dist(p0, p3);
        double c1 = dist(p1, p3);

        double a2 = dist(p1, p2);
        double b2 = dist(p2, p3);
        double c2 = c1;

        double s1 = (a1 + b1 + c1) / 2.0;
        double s2 = (a2 + b2 + c2) / 2.0;

        a1 = sqrt(s1 * (s1 - a1) * (s1 - b1) * (s1 - c1));
        a2 = sqrt(s2 * (s2 - a2) * (s2 - b2) * (s2 - c2));
        return a1 + a2;
    }
    double getReprojectionError(const std::vector<cv::Point3f>& objectPoints, const std::vector<cv::Point2f>& imagePoints,
                                       const cv::Matx33d& cameraMatrix, const cv::Vec4f& distCoeffs, const cv::Vec3d& rvec,
                                       const cv::Vec3d& tvec) {
        std::vector<cv::Point2f> projectedPoints;
        cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, projectedPoints);
        // calculate RMS image error
        double totalError = 0.0;
        for (unsigned int i = 0; i < objectPoints.size(); i++) {
            double error = dist(imagePoints[i], projectedPoints[i]);
            totalError += error * error;
        }
        double rerror = totalError / (double)objectPoints.size();
        return rerror;
    }
    void distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u, double k1, double k2, double p1, double p2) {
        double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;
        mx2_u = p_u(0) * p_u(0);
        my2_u = p_u(1) * p_u(1);
        mxy_u = p_u(0) * p_u(1);
        rho2_u = mx2_u + my2_u;
        rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;
        d_u << p_u(0) * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u),
                p_u(1) * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);
    }
    void liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P, double m_inv_K11, double m_inv_K13, double m_inv_K22,
                        double m_inv_K23, double k1, double k2, double p1, double p2) {
        double mx_d, my_d, mx2_d, mxy_d, my2_d, mx_u, my_u;
        double rho2_d, rho4_d, radDist_d, Dx_d, Dy_d, inv_denom_d;
        // Lift points to normalised plane
        mx_d = m_inv_K11 * p(0) + m_inv_K13;
        my_d = m_inv_K22 * p(1) + m_inv_K23;
        if (0) {
            mx_u = mx_d;
            my_u = my_d;
        } else {
            if (0) {
                // Apply inverse distortion model
                // proposed by Heikkila
                mx2_d = mx_d * mx_d;
                my2_d = my_d * my_d;
                mxy_d = mx_d * my_d;
                rho2_d = mx2_d + my2_d;
                rho4_d = rho2_d * rho2_d;
                radDist_d = k1 * rho2_d + k2 * rho4_d;
                Dx_d = mx_d * radDist_d + p2 * (rho2_d + 2 * mx2_d) + 2 * p1 * mxy_d;
                Dy_d = my_d * radDist_d + p1 * (rho2_d + 2 * my2_d) + 2 * p2 * mxy_d;
                inv_denom_d = 1 / (1 + 4 * k1 * rho2_d + 6 * k2 * rho4_d + 8 * p1 * my_d + 8 * p2 * mx_d);

                mx_u = mx_d - inv_denom_d * Dx_d;
                my_u = my_d - inv_denom_d * Dy_d;
            } else {
                // Recursive distortion model
                int n = 8;
                Eigen::Vector2d d_u;
                distortion(Eigen::Vector2d(mx_d, my_d), d_u, k1, k2, p1, p2);
                // Approximate value
                mx_u = mx_d - d_u(0);
                my_u = my_d - d_u(1);

                for (int i = 1; i < n; ++i) {
                    distortion(Eigen::Vector2d(mx_u, my_u), d_u, k1, k2, p1, p2);
                    mx_u = mx_d - d_u(0);
                    my_u = my_d - d_u(1);
                }
            }
        }
        // Obtain a projective ray
        P << mx_u, my_u, 1.0;
    }
    void ImageCallback(const sensor_msgs::ImageConstPtr& msg) {
        if (!has_camera_info_) {
            ROS_WARN("No Camera Info Received Yet");
            return;
        }
        // Get the image
        cv_bridge::CvImagePtr subscribed_ptr;
        try {
            subscribed_ptr = cv_bridge::toCvCopy(msg, "mono8");
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat gray;
        if (subscribed_ptr->image.type() == CV_8UC1) {
            gray = subscribed_ptr->image;
        } else if (subscribed_ptr->image.type() == CV_8UC3) {
            cv::cvtColor(subscribed_ptr->image, gray, cv::COLOR_BGR2GRAY);
        } else {
            return;
        }
        //cv::adaptiveThreshold(gray,gray,255,cv::ADAPTIVE_THRESH_GAUSSIAN_C,cv::THRESH_BINARY,21,-11);
        threshold(gray, gray, 150, 255, cv::THRESH_OTSU);
        cv::Mat subscribed_gray = gray;
        ApriltagVec apriltags;
        if (USE_UMICH_) {
            apriltag_umich::ImageU8Ptr image_u8(
                    image_u8_create_from_gray(subscribed_gray.cols, subscribed_gray.rows, subscribed_gray.data));
            // Detection
            apriltag_umich::ZarrayPtr detections(apriltag_detector_detect(umich_tag_detector_.get(), image_u8.get()));
            // Handle empty detection
            const auto num_detections = zarray_size(detections.get());
            apriltags.reserve(num_detections);
            for (int i = 0; i < num_detections; ++i) {
                apriltag_detection_t* td;
                zarray_get(detections.get(), i, &td);
                apriltag_msgs::Apriltag apriltag;
                apriltag.id = td->id;
                apriltag.bits = 6;  // payload_
                apriltag.hamming = td->hamming;
                apriltag.family = static_cast<char>(TagFamily(Get_TagFamily()));
                apriltag.border = Get_black_border();  // black_border_
                apriltag.center.x = td->c[0];
                apriltag.center.y = td->c[1];
                for (size_t i = 0; i < 4; ++i) {
                    // Umich's order of corners is different from mit's
                    apriltag.corners[i].x = td->p[i][0];
                    apriltag.corners[i].y = td->p[i][1];
                }
                apriltags.push_back(apriltag);
            }
        } else {
            auto detections = mit_tag_detector_->ExtractTags(subscribed_gray);
            apriltags.reserve(detections.size());
            for (const apriltag_mit::TagDetection& td : detections) {
                apriltag_msgs::Apriltag apriltag;
                apriltag.id = td.id;
                apriltag.bits = 6;
                apriltag.border = Get_black_border();
                apriltag.family = static_cast<char>(TagFamily(Get_TagFamily()));
                apriltag.hamming = td.hamming_distance;
                apriltag.center.x = td.cxy.x;
                apriltag.center.y = td.cxy.y;
                for (size_t j = 0; j < 4; ++j) {
                    apriltag.corners[j].x = td.p[j].x;
                    apriltag.corners[j].y = td.p[j].y;
                }
                apriltags.push_back(apriltag);
            }
        }
        visualization_msgs::MarkerArray marker_transforms;
        dbrobot_msg::QRTagDetections apriltag_detections;
        apriltag_detections.header.frame_id = msg->header.frame_id;
        apriltag_detections.header.stamp = msg->header.stamp;
        cv_bridge::CvImagePtr subscribed_color_ptr;
        if (viewer_) {
            try {
                subscribed_color_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
        }
        for (apriltag_msgs::Apriltag &detection : apriltags) {
            Eigen::Matrix4d pose;
            cv::Mat rvec;
            cv::Mat tvec;
            GetMarkerTransformUsingOpenCV(detection, pose, rvec, tvec);
            // Get this info from earlier code, don't extract it again
            Eigen::Matrix3d R = pose.block<3, 3>(0, 0);
            Eigen::Quaternion<double> q(R);
            // Fill in MarkerArray msg
            visualization_msgs::Marker marker_transform;
            marker_transform.header.frame_id = msg->header.frame_id;
            marker_transform.header.stamp = msg->header.stamp;
            // Only publish marker for 0.5 seconds after it
            marker_transform.lifetime = ros::Duration(0.5);
            std::stringstream convert;
            convert << "tag" << detection.id;
            marker_transform.ns = convert.str().c_str();
            marker_transform.id = detection.id;
            if (display_type_ == "ARROW") {
                marker_transform.type = visualization_msgs::Marker::ARROW;
                marker_transform.scale.x = default_tag_size_;         // arrow length
                marker_transform.scale.y = default_tag_size_ / 10.0;  // diameter
                marker_transform.scale.z = default_tag_size_ / 10.0;  // diameter
            } else if (display_type_ == "CUBE") {
                marker_transform.type = visualization_msgs::Marker::CUBE;
                marker_transform.scale.x = default_tag_size_;
                marker_transform.scale.y = default_tag_size_;
                marker_transform.scale.z = marker_thickness_;
            }
            //因为图像坐标系　以向右边为Ｘ正方向　向下为Y的正方向 所以这里将Ｙ的方向置反　机器人朝向置反
            marker_transform.action = visualization_msgs::Marker::ADD;
            marker_transform.pose.position.x = pose(0, 3);
            marker_transform.pose.position.y = pose(1, 3);
            marker_transform.pose.position.z = pose(2, 3);
            marker_transform.pose.orientation.x = q.x();
            marker_transform.pose.orientation.y = q.y();
            marker_transform.pose.orientation.z = q.z();
            marker_transform.pose.orientation.w = q.w();

            marker_transform.color.r = 1.0;
            marker_transform.color.g = 0.0;
            marker_transform.color.b = 1.0;
            marker_transform.color.a = 1.0;
            marker_transforms.markers.push_back(marker_transform);

            // Fill in AprilTag detection.
            dbrobot_msg::AprilTagDetection apriltag_det;
            apriltag_det.header = marker_transform.header;
            apriltag_det.id = marker_transform.id;
            apriltag_det.pose = marker_transform.pose;
            for (int i = 0; i < 4; ++i) {
                geometry_msgs::Point32 msgs;
                msgs.x=detection.corners[i].x;
                msgs.y=detection.corners[i].y;
                msgs.z=0.;
                apriltag_det.corners2d[i]=msgs;
            }
            apriltag_det.tag_size = default_tag_size_;
            //apriltag_det.tag_size = detection.bits;
            //apriltag_det.tag_variance = detection.observedPerimeter;
            apriltag_detections.detections.push_back(apriltag_det);

            //发布TF数据
            geometry_msgs::TransformStamped ts;
            ts.transform.rotation = marker_transform.pose.orientation;
            ts.transform.translation.x = marker_transform.pose.position.x;
            ts.transform.translation.y = marker_transform.pose.position.y;
            ts.transform.translation.z = marker_transform.pose.position.z;

            ts.header.frame_id = msg->header.frame_id;
            ts.header.stamp = msg->header.stamp;
            ts.child_frame_id = "fiducial_" + std::to_string(apriltag_det.id);
            static tf2_ros::TransformBroadcaster broadcaster;
            broadcaster.sendTransform(ts);

            if (viewer_) {
                if (display_marker_outline_) {
                    cv::Scalar outline_color(0, 0, 255);  // blue (BGR ordering)
                    DrawMarkerOutline(detection, outline_color, subscribed_color_ptr->image);
                }
                if (display_marker_id_) {
                    cv::Scalar text_color(255, 255, 0);  // light-blue (BGR ordering)
                    DrawMarkerID(detection, text_color, subscribed_color_ptr->image);
                }
                if (display_marker_edges_) {
                    DrawMarkerEdges(detection, subscribed_color_ptr->image);
                }
                if (display_marker_axes_) {
                    cv::Matx33f intrinsics(camera_info_.K[0], 0, camera_info_.K[2], 0, camera_info_.K[4], camera_info_.K[5], 0, 0,
                                           1);
                    cv::Vec4f distortion_coeff(camera_info_.D[0], camera_info_.D[1], camera_info_.D[2], camera_info_.D[3]);
                    double axis_length = default_tag_size_ *2;
                    const bool draw_arrow_heads = true;
                    DrawMarkerAxes(intrinsics, distortion_coeff, rvec, tvec, axis_length, draw_arrow_heads,
                                   subscribed_color_ptr->image);
                }
            }
        }
        marker_pub.publish(marker_transforms);
        apriltag_publisher.publish(apriltag_detections);

        if (viewer_) {
            cv::imshow("HexagonTags", subscribed_color_ptr->image);
            cv::waitKey(3);
        }
    }

    ros::NodeHandle m_node;
    image_transport::ImageTransport m_image;
    ros::Subscriber tag_enable_sub;
    bool tag_enable;

    apriltag_umich::TagFamilyPtr umich_tag_family_;
    apriltag_umich::TagDetectorPtr umich_tag_detector_;
    apriltag_mit::TagDetectorPtr mit_tag_detector_;

    sensor_msgs::CameraInfo camera_info_;
    ros::Subscriber camera_info_sub;
    image_transport::Subscriber image_sub;
    ros::Publisher marker_pub;
    ros::Publisher apriltag_publisher;

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
};

#endif //APRILTAG_DETECTOR_MANAGER_H
