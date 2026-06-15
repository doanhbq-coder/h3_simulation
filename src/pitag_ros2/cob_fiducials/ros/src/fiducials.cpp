// Ported from ROS1 to ROS2 Humble
// Original: Fraunhofer IPA, Jan Fischer
// License: LGPL

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/empty.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cob_object_detection_msgs/msg/detection.hpp>
#include <cob_object_detection_msgs/msg/detection_array.hpp>
#include <cob_object_detection_msgs/srv/detect_objects.hpp>

#include <cob_vision_utils/GlobalDefines.h>
#include <cob_fiducials/FiducialDefines.h>
#include <cob_fiducials/pi/FiducialModelPi.h>
#include <cob_fiducials/aruco/FiducialModelAruco.h>

#include <opencv2/opencv.hpp>

#include <mutex>
#include <condition_variable>
#include <memory>
#include <sstream>
#include <cmath>

namespace ipa_Fiducials
{

class CobFiducialsNode : public rclcpp::Node
{
    enum t_Mode
    {
        MODE_TOPIC = 0,
        MODE_SERVICE,
        MODE_TOPIC_AND_SERVICE
    };

public:
    CobFiducialsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("cob_fiducials", options),
      camera_matrix_initialized_(false),
      prev_marker_array_size_(0),
      service_call_active_(false)
    {
        if (!loadParameters()) {
            RCLCPP_ERROR(get_logger(), "[fiducials] Failed to load parameters, shutting down");
            return;
        }

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscriptions — use SensorDataQoS (BEST_EFFORT) to match camera drivers
        auto sensor_qos = rclcpp::SensorDataQoS();
        color_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_color", sensor_qos,
            std::bind(&CobFiducialsNode::colorImageCallback, this, std::placeholders::_1));

        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera_info", sensor_qos,
            std::bind(&CobFiducialsNode::cameraInfoCallback, this, std::placeholders::_1));

        // Publishers
        if (ros_node_mode_ == MODE_TOPIC || ros_node_mode_ == MODE_TOPIC_AND_SERVICE) {
            detect_fiducials_pub_ = this->create_publisher<cob_object_detection_msgs::msg::DetectionArray>(
                "detect_fiducials", 10);
        }

        fiducials_marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "fiducial_marker_array", 10);

        img2d_pub_ = image_transport::create_publisher(this, "image");

        if (publish_tf_) {
            fiducial_pub_ = this->create_publisher<cob_object_detection_msgs::msg::DetectionArray>(
                "fiducial_detection_array", 10);
        }

        // Services
        if (ros_node_mode_ == MODE_SERVICE || ros_node_mode_ == MODE_TOPIC_AND_SERVICE) {
            detect_service_ = this->create_service<cob_object_detection_msgs::srv::DetectObjects>(
                "get_fiducials",
                std::bind(&CobFiducialsNode::detectFiducialsServiceCallback, this,
                          std::placeholders::_1, std::placeholders::_2));
        }

        if (publish_tf_) {
            stop_tf_service_ = this->create_service<std_srvs::srv::Empty>(
                "stop_tf",
                std::bind(&CobFiducialsNode::stopTfServiceCallback, this,
                          std::placeholders::_1, std::placeholders::_2));

            tf_pub_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&CobFiducialsNode::publishTfCallback, this));
        }

        // Init fiducial detector
        switch (fiducial_type_) {
            case ipa_Fiducials::TYPE_PI:
                tag_detector_ = std::make_shared<FiducialModelPi>();
                break;
            case ipa_Fiducials::TYPE_ARUCO:
                tag_detector_ = std::make_shared<FiducialModelAruco>();
                break;
            default:
                RCLCPP_ERROR(get_logger(), "[fiducials] Unknown fiducial type");
                return;
        }

        RCLCPP_INFO(get_logger(), "[fiducials] Up and running");
    }

private:
    // Parameters
    t_FiducialType fiducial_type_;
    t_Mode ros_node_mode_;
    std::string model_directory_;
    std::string model_filename_;
    bool compute_sharpness_measure_;
    double sharpness_calibration_parameter_m_;
    double sharpness_calibration_parameter_n_;
    bool log_or_calibrate_sharpness_measurements_;
    bool publish_marker_array_;
    bool publish_tf_;
    bool publish_2d_image_;
    int debug_verbosity_;
    bool use_ir_reflective_markers_; // invert image for bright reflective dots on dark IR background

    // Camera state
    cv::Mat camera_matrix_;
    bool camera_matrix_initialized_;
    cv::Mat color_mat_8U3_;
    std::string received_frame_id_;
    rclcpp::Time received_timestamp_;

    // Detection cache (for service mode)
    cob_object_detection_msgs::msg::DetectionArray detection_array_;
    std::mutex mutex_detection_;
    std::condition_variable cond_detection_;
    bool service_call_active_;

    // TF state
    std::string marker_tf_frame_id_;
    geometry_msgs::msg::TransformStamped last_marker_tf_;
    std::mutex tf_lock_;

    // Visualization
    visualization_msgs::msg::MarkerArray marker_array_msg_;
    unsigned int prev_marker_array_size_;

    // Fiducial detector
    std::shared_ptr<ipa_Fiducials::AbstractFiducialModel> tag_detector_;

    // ROS2 interfaces
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Publisher<cob_object_detection_msgs::msg::DetectionArray>::SharedPtr detect_fiducials_pub_;
    rclcpp::Publisher<cob_object_detection_msgs::msg::DetectionArray>::SharedPtr fiducial_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fiducials_marker_array_pub_;
    image_transport::Publisher img2d_pub_;
    rclcpp::Service<cob_object_detection_msgs::srv::DetectObjects>::SharedPtr detect_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_tf_service_;
    rclcpp::TimerBase::SharedPtr tf_pub_timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    bool loadParameters()
    {
        // Declare all parameters with defaults
        declare_parameter("fiducial_type", std::string(""));
        declare_parameter("ros_node_mode", std::string(""));
        declare_parameter("model_directory", std::string(""));
        declare_parameter("model_filename", std::string(""));
        declare_parameter("compute_sharpness_measure", false);
        declare_parameter("sharpness_calibration_parameter_m", 0.0);
        declare_parameter("sharpness_calibration_parameter_n", 0.0);
        declare_parameter("log_or_calibrate_sharpness_measurements", false);
        declare_parameter("publish_marker_array", false);
        declare_parameter("publish_tf", false);
        declare_parameter("publish_2d_image", false);
        declare_parameter("debug_verbosity", 1);
        declare_parameter("use_ir_reflective_markers", false);

        std::string fiducial_type_str = get_parameter("fiducial_type").as_string();
        if (fiducial_type_str == "TYPE_PI") {
            fiducial_type_ = ipa_Fiducials::TYPE_PI;
        } else if (fiducial_type_str == "TYPE_ARUCO") {
            fiducial_type_ = ipa_Fiducials::TYPE_ARUCO;
        } else {
            RCLCPP_ERROR(get_logger(), "[fiducials] fiducial_type '%s' unknown. Use TYPE_PI or TYPE_ARUCO",
                         fiducial_type_str.c_str());
            return false;
        }

        std::string mode_str = get_parameter("ros_node_mode").as_string();
        if (mode_str == "MODE_TOPIC") {
            ros_node_mode_ = MODE_TOPIC;
        } else if (mode_str == "MODE_SERVICE") {
            ros_node_mode_ = MODE_SERVICE;
        } else if (mode_str == "MODE_TOPIC_AND_SERVICE") {
            ros_node_mode_ = MODE_TOPIC_AND_SERVICE;
        } else {
            RCLCPP_ERROR(get_logger(), "[fiducials] ros_node_mode '%s' unknown. Use MODE_TOPIC, MODE_SERVICE, or MODE_TOPIC_AND_SERVICE",
                         mode_str.c_str());
            return false;
        }

        model_directory_ = get_parameter("model_directory").as_string();
        model_filename_ = get_parameter("model_filename").as_string();
        compute_sharpness_measure_ = get_parameter("compute_sharpness_measure").as_bool();
        sharpness_calibration_parameter_m_ = get_parameter("sharpness_calibration_parameter_m").as_double();
        sharpness_calibration_parameter_n_ = get_parameter("sharpness_calibration_parameter_n").as_double();
        log_or_calibrate_sharpness_measurements_ = get_parameter("log_or_calibrate_sharpness_measurements").as_bool();
        publish_marker_array_ = get_parameter("publish_marker_array").as_bool();
        publish_tf_ = get_parameter("publish_tf").as_bool();
        publish_2d_image_ = get_parameter("publish_2d_image").as_bool();
        debug_verbosity_ = get_parameter("debug_verbosity").as_int();
        use_ir_reflective_markers_ = get_parameter("use_ir_reflective_markers").as_bool();
        if (use_ir_reflective_markers_)
            RCLCPP_INFO(get_logger(), "[fiducials] IR reflective marker mode: image will be inverted before detection");

        RCLCPP_INFO(get_logger(), "[fiducials] fiducial_type: %s", fiducial_type_str.c_str());
        RCLCPP_INFO(get_logger(), "[fiducials] ros_node_mode: %s", mode_str.c_str());
        RCLCPP_INFO(get_logger(), "[fiducials] model_directory: %s", model_directory_.c_str());
        RCLCPP_INFO(get_logger(), "[fiducials] model_filename: %s", model_filename_.c_str());

        return true;
    }

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr info)
    {
        if (camera_matrix_initialized_) return;

        camera_matrix_ = cv::Mat::zeros(3, 3, CV_64FC1);
        camera_matrix_.at<double>(0, 0) = info->k[0];
        camera_matrix_.at<double>(0, 2) = info->k[2];
        camera_matrix_.at<double>(1, 1) = info->k[4];
        camera_matrix_.at<double>(1, 2) = info->k[5];
        camera_matrix_.at<double>(2, 2) = 1.0;

        std::string model_path = model_directory_ + model_filename_;
        if (tag_detector_->Init(camera_matrix_, model_path, log_or_calibrate_sharpness_measurements_) & ipa_Utils::RET_FAILED) {
            RCLCPP_ERROR(get_logger(), "[fiducials] Initializing fiducial detector with camera matrix [FAILED]");
            return;
        }
        camera_matrix_initialized_ = true;
        RCLCPP_INFO(get_logger(), "[fiducials] Camera matrix initialized, detector ready");
    }

    void colorImageCallback(const sensor_msgs::msg::Image::SharedPtr img)
    {
        if (!camera_matrix_initialized_) return;
        RCLCPP_DEBUG(get_logger(), "[fiducials] colorImageCallback: enc=%s %dx%d",
                     img->encoding.c_str(), img->width, img->height);

        cv::Mat gray_8U1;
        try {
            cv_bridge::CvImageConstPtr cv_ptr;
            const std::string & enc = img->encoding;

            if (enc == sensor_msgs::image_encodings::MONO8) {
                cv_ptr = cv_bridge::toCvShare(img, sensor_msgs::image_encodings::MONO8);
                gray_8U1 = cv_ptr->image.clone();
            } else if (enc == sensor_msgs::image_encodings::MONO16 ||
                       enc == sensor_msgs::image_encodings::TYPE_16UC1) {
                cv_ptr = cv_bridge::toCvShare(img, sensor_msgs::image_encodings::MONO16);
                // Scale 16-bit [0..65535] down to 8-bit [0..255]
                cv_ptr->image.convertTo(gray_8U1, CV_8U, 1.0 / 256.0);
            } else {
                // Colour camera: decode as BGR then convert to gray
                cv_ptr = cv_bridge::toCvShare(img, sensor_msgs::image_encodings::BGR8);
                cv::cvtColor(cv_ptr->image, gray_8U1, cv::COLOR_BGR2GRAY);
            }
        } catch (cv_bridge::Exception & e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // For reflective markers under IR illumination the dots are bright on a dark
        // background — the opposite of a printed black-on-white marker.
        // Inverting brings them back to dark-on-bright so the detector works unchanged.
        if (use_ir_reflective_markers_) {
            cv::bitwise_not(gray_8U1, gray_8U1);
        }

        // Promote to BGR so renderPose() can draw coloured axes on top
        cv::Mat bgr_mat;
        cv::cvtColor(gray_8U1, bgr_mat, cv::COLOR_GRAY2BGR);

        std::lock_guard<std::mutex> lock(mutex_detection_);
        received_timestamp_ = img->header.stamp;
        received_frame_id_ = img->header.frame_id;
        color_mat_8U3_ = bgr_mat;

        detection_array_.detections.clear();
        detection_array_.header = img->header;

        detectFiducials(detection_array_, color_mat_8U3_);

        if (ros_node_mode_ == MODE_TOPIC || ros_node_mode_ == MODE_TOPIC_AND_SERVICE) {
            detect_fiducials_pub_->publish(detection_array_);
        }

        cond_detection_.notify_all();

        if (service_call_active_) {
            // yield so service thread can acquire the mutex
            rclcpp::sleep_for(std::chrono::milliseconds(200));
            service_call_active_ = false;
        }
    }

    void detectFiducialsServiceCallback(
        const std::shared_ptr<cob_object_detection_msgs::srv::DetectObjects::Request> req,
        std::shared_ptr<cob_object_detection_msgs::srv::DetectObjects::Response> res)
    {
        service_call_active_ = true;

        std::unique_lock<std::mutex> lock(mutex_detection_);
        bool got_data = cond_detection_.wait_for(lock, std::chrono::seconds(5),
            [this] { return !detection_array_.detections.empty() || !service_call_active_; });

        if (!got_data) {
            RCLCPP_WARN(get_logger(), "[fiducials] Timeout waiting for image data");
            return;
        }

        if (req->object_name.data.empty() || req->object_name.data == "ALL") {
            res->object_list = detection_array_;
        } else {
            res->object_list.header = detection_array_.header;
            for (const auto & det : detection_array_.detections) {
                if (det.label == req->object_name.data) {
                    res->object_list.detections.push_back(det);
                }
            }
        }
    }

    void stopTfServiceCallback(
        const std::shared_ptr<std_srvs::srv::Empty::Request>,
        std::shared_ptr<std_srvs::srv::Empty::Response>)
    {
        std::lock_guard<std::mutex> lock(tf_lock_);
        marker_tf_frame_id_ = "";
    }

    void publishTfCallback()
    {
        std::lock_guard<std::mutex> lock(tf_lock_);
        if (!marker_tf_frame_id_.empty()) {
            last_marker_tf_.header.stamp = now();
            tf_broadcaster_->sendTransform(last_marker_tf_);
        }
    }

    bool detectFiducials(cob_object_detection_msgs::msg::DetectionArray & detection_array, cv::Mat & color_image)
    {
        std::vector<ipa_Fiducials::t_pose> tags_vec;
        std::vector<std::vector<double>> vec_vec7d;

        {
            std::lock_guard<std::mutex> lock(tf_lock_);
            marker_tf_frame_id_ = "";
        }

        unsigned long ret_val = tag_detector_->GetPose(color_image, tags_vec);
        unsigned int pose_array_size = (ret_val & ipa_Utils::RET_OK) ? tags_vec.size() : 0;

        // Always publish debug image (matches ROS1 behaviour) so the user can
        // verify images are flowing even when no marker is visible.
        if (publish_2d_image_) {
            for (unsigned int i = 0; i < pose_array_size; i++)
                renderPose(color_image, tags_vec[i].rot, tags_vec[i].trans);
            cv_bridge::CvImage cv_img;
            cv_img.header  = detection_array.header;
            cv_img.image   = color_image;
            cv_img.encoding = "bgr8";
            img2d_pub_.publish(cv_img.toImageMsg());
        }

        if (pose_array_size == 0)
            return false;

        for (unsigned int i = 0; i < pose_array_size; i++) {
            cob_object_detection_msgs::msg::Detection fiducial_instance;

            std::stringstream ss;
            ss << "tag_" << tags_vec[i].id;
            fiducial_instance.header = detection_array.header;
            fiducial_instance.label = ss.str();
            fiducial_instance.id = tags_vec[i].id;
            fiducial_instance.detector = tag_detector_->GetType();
            fiducial_instance.score = 0;

            cv::Mat frame(3, 4, CV_64FC1);
            for (int k = 0; k < 3; k++)
                for (int l = 0; l < 3; l++)
                    frame.at<double>(k, l) = tags_vec[i].rot.at<double>(k, l);
            frame.at<double>(0, 3) = tags_vec[i].trans.at<double>(0, 0);
            frame.at<double>(1, 3) = tags_vec[i].trans.at<double>(1, 0);
            frame.at<double>(2, 3) = tags_vec[i].trans.at<double>(2, 0);

            std::vector<double> vec7d = frameToVec7(frame);
            vec_vec7d.push_back(vec7d);

            fiducial_instance.pose.pose.position.x = vec7d[0];
            fiducial_instance.pose.pose.position.y = vec7d[1];
            fiducial_instance.pose.pose.position.z = vec7d[2];
            fiducial_instance.pose.pose.orientation.w = vec7d[3];
            fiducial_instance.pose.pose.orientation.x = vec7d[4];
            fiducial_instance.pose.pose.orientation.y = vec7d[5];
            fiducial_instance.pose.pose.orientation.z = vec7d[6];
            fiducial_instance.pose.header = detection_array.header;

            if (compute_sharpness_measure_) {
                double sharpness_measure;
                tag_detector_->GetSharpnessMeasure(color_image, tags_vec[i],
                    tag_detector_->GetGeneralFiducialParameters(tags_vec[i].id),
                    sharpness_measure, sharpness_calibration_parameter_m_,
                    sharpness_calibration_parameter_n_);
                fiducial_instance.score = static_cast<float>(sharpness_measure);
            } else {
                // Khi không tính sharpness, dùng score để publish reprojection error [pixels]
                fiducial_instance.score = static_cast<float>(tags_vec[i].reproj_error);
            }

            detection_array.detections.push_back(fiducial_instance);
        }

        // Publish detection array with IDs (for TF mode)
        if (publish_tf_ && fiducial_pub_) {
            cob_object_detection_msgs::msg::DetectionArray container_msg;
            container_msg.header = detection_array.header;
            for (unsigned int i = 0; i < pose_array_size; i++) {
                cob_object_detection_msgs::msg::Detection det;
                det.id = tags_vec[i].id;
                det.pose.pose.position.x = vec_vec7d[i][0];
                det.pose.pose.position.y = vec_vec7d[i][1];
                det.pose.pose.position.z = vec_vec7d[i][2];
                det.pose.pose.orientation.w = vec_vec7d[i][3];
                det.pose.pose.orientation.x = vec_vec7d[i][4];
                det.pose.pose.orientation.y = vec_vec7d[i][5];
                det.pose.pose.orientation.z = vec_vec7d[i][6];
                container_msg.detections.push_back(det);
            }
            fiducial_pub_->publish(container_msg);
        }

        // Publish TF transforms
        if (publish_tf_) {
            for (unsigned int i = 0; i < pose_array_size; i++) {
                geometry_msgs::msg::TransformStamped tf_stamped;
                tf_stamped.header.stamp = now();
                tf_stamped.header.frame_id = detection_array.header.frame_id;
                std::stringstream ss;
                ss << "marker_" << tags_vec[i].id;
                tf_stamped.child_frame_id = ss.str();
                tf_stamped.transform.translation.x = vec_vec7d[i][0];
                tf_stamped.transform.translation.y = vec_vec7d[i][1];
                tf_stamped.transform.translation.z = vec_vec7d[i][2];
                tf_stamped.transform.rotation.w = vec_vec7d[i][3];
                tf_stamped.transform.rotation.x = vec_vec7d[i][4];
                tf_stamped.transform.rotation.y = vec_vec7d[i][5];
                tf_stamped.transform.rotation.z = vec_vec7d[i][6];

                tf_broadcaster_->sendTransform(tf_stamped);

                {
                    std::lock_guard<std::mutex> lock(tf_lock_);
                    last_marker_tf_ = tf_stamped;
                    marker_tf_frame_id_ = tf_stamped.header.frame_id;
                }
            }
        }

        // Publish marker array for RViz
        if (publish_marker_array_) {
            unsigned int marker_array_size = 3 * pose_array_size;
            marker_array_msg_.markers.resize(
                std::max(marker_array_size, prev_marker_array_size_));

            for (unsigned int i = 0; i < pose_array_size; i++) {
                for (unsigned int j = 0; j < 3; j++) {
                    unsigned int idx = 3 * i + j;
                    auto & m = marker_array_msg_.markers[idx];
                    m.header = detection_array.header;
                    m.ns = "fiducials";
                    m.id = 2351 + static_cast<int>(idx);
                    m.type = visualization_msgs::msg::Marker::ARROW;
                    m.action = visualization_msgs::msg::Marker::ADD;
                    m.color.a = 0.85f;
                    m.color.r = m.color.g = m.color.b = 0.0f;

                    m.points.resize(2);
                    m.points[0].x = m.points[0].y = m.points[0].z = 0.0;
                    m.points[1].x = m.points[1].y = m.points[1].z = 0.0;

                    if (j == 0) { m.points[1].x = 0.2; m.color.r = 1.0f; }
                    else if (j == 1) { m.points[1].y = 0.2; m.color.g = 1.0f; }
                    else { m.points[1].z = 0.2; m.color.b = 1.0f; }

                    m.pose.position.x = vec_vec7d[i][0];
                    m.pose.position.y = vec_vec7d[i][1];
                    m.pose.position.z = vec_vec7d[i][2];
                    m.pose.orientation.w = vec_vec7d[i][3];
                    m.pose.orientation.x = vec_vec7d[i][4];
                    m.pose.orientation.y = vec_vec7d[i][5];
                    m.pose.orientation.z = vec_vec7d[i][6];

                    m.lifetime = rclcpp::Duration::from_seconds(1.0);
                    m.scale.x = 0.01;
                    m.scale.y = 0.015;
                    m.scale.z = 0.0;
                }
            }

            for (unsigned int i = marker_array_size; i < prev_marker_array_size_; i++) {
                marker_array_msg_.markers[i].action = visualization_msgs::msg::Marker::DELETE;
            }
            prev_marker_array_size_ = marker_array_size;

            fiducials_marker_array_pub_->publish(marker_array_msg_);
        }

        return !tags_vec.empty();
    }

    void renderPose(cv::Mat & image, cv::Mat & rot, cv::Mat & trans)
    {
        cv::Mat pt_axis(4, 3, CV_64FC1);
        double * p = pt_axis.ptr<double>(0);
        p[0] = p[1] = p[2] = 0;
        p = pt_axis.ptr<double>(1); p[0] = 0.1; p[1] = p[2] = 0;
        p = pt_axis.ptr<double>(2); p[1] = 0.1; p[0] = p[2] = 0;
        p = pt_axis.ptr<double>(3); p[2] = 0.1; p[0] = p[1] = 0;

        std::vector<cv::Point> pts(4);
        for (int i = 0; i < 4; i++) {
            cv::Mat v = pt_axis.row(i).clone().t();
            v = rot * v + trans;
            double * d = v.ptr<double>(0);
            reprojectXYZ(d[0], d[1], d[2], pts[i].x, pts[i].y);
        }
        cv::line(image, pts[0], pts[1], cv::Scalar(0, 0, 255), 1);
        cv::line(image, pts[0], pts[2], cv::Scalar(0, 255, 0), 1);
        cv::line(image, pts[0], pts[3], cv::Scalar(255, 0, 0), 1);
    }

    void reprojectXYZ(double x, double y, double z, int & u, int & v)
    {
        x *= 1000; y *= 1000; z *= 1000;
        cv::Mat xyz = (cv::Mat_<double>(3, 1) << x, y, z);
        cv::Mat uvw = camera_matrix_ * xyz;
        double * d = uvw.ptr<double>(0);
        u = cvRound(d[0] / d[2]);
        v = cvRound(d[1] / d[2]);
    }

    inline float sign(float x) { return (x >= 0.0f) ? 1.0f : -1.0f; }

    std::vector<double> frameToVec7(const cv::Mat & frame)
    {
        std::vector<double> pose(7, 0.0);

        double r11 = frame.at<double>(0, 0), r12 = frame.at<double>(0, 1), r13 = frame.at<double>(0, 2);
        double r21 = frame.at<double>(1, 0), r22 = frame.at<double>(1, 1), r23 = frame.at<double>(1, 2);
        double r31 = frame.at<double>(2, 0), r32 = frame.at<double>(2, 1), r33 = frame.at<double>(2, 2);

        double qw = (r11 + r22 + r33 + 1.0) / 4.0;
        double qx = (r11 - r22 - r33 + 1.0) / 4.0;
        double qy = (-r11 + r22 - r33 + 1.0) / 4.0;
        double qz = (-r11 - r22 + r33 + 1.0) / 4.0;

        qw = std::sqrt(std::max(0.0, qw));
        qx = std::sqrt(std::max(0.0, qx));
        qy = std::sqrt(std::max(0.0, qy));
        qz = std::sqrt(std::max(0.0, qz));

        if (qw >= qx && qw >= qy && qw >= qz) {
            qx *= sign(static_cast<float>(r32 - r23));
            qy *= sign(static_cast<float>(r13 - r31));
            qz *= sign(static_cast<float>(r21 - r12));
        } else if (qx >= qw && qx >= qy && qx >= qz) {
            qw *= sign(static_cast<float>(r32 - r23));
            qy *= sign(static_cast<float>(r21 + r12));
            qz *= sign(static_cast<float>(r13 + r31));
        } else if (qy >= qw && qy >= qx && qy >= qz) {
            qw *= sign(static_cast<float>(r13 - r31));
            qx *= sign(static_cast<float>(r21 + r12));
            qz *= sign(static_cast<float>(r32 + r23));
        } else {
            qw *= sign(static_cast<float>(r21 - r12));
            qx *= sign(static_cast<float>(r31 + r13));
            qy *= sign(static_cast<float>(r32 + r23));
        }

        double r = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
        pose[3] = qw / r;
        pose[4] = qx / r;
        pose[5] = qy / r;
        pose[6] = qz / r;
        pose[0] = frame.at<double>(0, 3);
        pose[1] = frame.at<double>(1, 3);
        pose[2] = frame.at<double>(2, 3);

        return pose;
    }
};

} // namespace ipa_Fiducials

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ipa_Fiducials::CobFiducialsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}