#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <ORB_SLAM3/System.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <deque>
#include <iomanip>
#include <sophus/se3.hpp>


// --- Helper class for moving average filter ---
class MovingAverageFilter {
public:
    MovingAverageFilter(size_t window_size = 10) : window_size_(window_size) {}

    cv::Vec3d update(const cv::Vec3d& new_value) {
        buffer_.push_back(new_value);
        if (buffer_.size() > window_size_) {
            buffer_.pop_front();
        }
        cv::Vec3d sum(0, 0, 0);
        for (const auto& val : buffer_) {
            sum += val;
        }
        return sum / static_cast<double>(buffer_.size());
    }

private:
    size_t window_size_;
    std::deque<cv::Vec3d> buffer_;
};


// --- Helper class for velocity estimation from visual odometry ---
class VelocityEstimation {
public:
    VelocityEstimation(const cv::Mat& camera_matrix) : K_(camera_matrix.clone()) {
        orb_ = cv::ORB::create(1500);
        bf_matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING, true);
    }

    void detect_features(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints) {
        orb_->detect(image, keypoints);
    }

    bool estimate_motion(const cv::Mat& left_image_gray, cv::Mat& delta_pose_out) {
        std::vector<cv::KeyPoint> keypoints_curr;
        cv::Mat descriptors_curr;
        orb_->detectAndCompute(left_image_gray, cv::noArray(), keypoints_curr, descriptors_curr);

        if (!is_initialized_ || keypoints_prev_.empty() || descriptors_prev_.empty()) {
            // Initialize for next frame
            keypoints_prev_ = keypoints_curr;
            descriptors_prev_ = descriptors_curr.clone();
            is_initialized_ = true;
            return false;
        }

        if (keypoints_curr.empty() || descriptors_curr.empty()) {
            return false;
        }

        // Match features between previous and current frame
        std::vector<cv::DMatch> matches;
        bf_matcher_->match(descriptors_prev_, descriptors_curr, matches);

        // Filter matches by distance
        if (matches.size() < 20) {
            keypoints_prev_ = keypoints_curr;
            descriptors_prev_ = descriptors_curr.clone();
            return false;
        }

        std::vector<double> match_distances;
        for (const auto& m : matches) {
            match_distances.push_back(m.distance);
        }
        std::sort(match_distances.begin(), match_distances.end());
        double distance_threshold = match_distances[static_cast<size_t>(match_distances.size() * 0.5)];

        std::vector<cv::Point2f> prev_pts, curr_pts;
        for (const auto& m : matches) {
            if (m.distance < distance_threshold * 1.5) {
                prev_pts.push_back(keypoints_prev_[m.queryIdx].pt);
                curr_pts.push_back(keypoints_curr[m.trainIdx].pt);
            }
        }

        if (prev_pts.size() < 15) {
            keypoints_prev_ = keypoints_curr;
            descriptors_prev_ = descriptors_curr.clone();
            return false;
        }

        // Estimate essential matrix and recover pose
        cv::Mat E, mask;
        E = cv::findEssentialMat(curr_pts, prev_pts, K_, cv::RANSAC, 0.999, 1.0, mask);

        if (E.empty()) {
            keypoints_prev_ = keypoints_curr;
            descriptors_prev_ = descriptors_curr.clone();
            return false;
        }

        cv::Mat R, t;
        int inliers = cv::recoverPose(E, curr_pts, prev_pts, K_, R, t, mask);

        if (inliers < 10) {
            keypoints_prev_ = keypoints_curr;
            descriptors_prev_ = descriptors_curr.clone();
            return false;
        }

        // Create delta pose matrix
        delta_pose_out = cv::Mat::eye(4, 4, CV_64F);
        R.copyTo(delta_pose_out(cv::Rect(0, 0, 3, 3)));
        t.copyTo(delta_pose_out(cv::Rect(3, 0, 1, 3)));

        // Update for next iteration
        keypoints_prev_ = keypoints_curr;
        descriptors_prev_ = descriptors_curr.clone();

        return true;
    }

private:
    cv::Mat K_;
    cv::Ptr<cv::ORB> orb_;
    cv::Ptr<cv::BFMatcher> bf_matcher_;
    std::vector<cv::KeyPoint> keypoints_prev_;
    cv::Mat descriptors_prev_;
    bool is_initialized_ = false;
};


// --- Main ROS 2 Node ---
using ImageMsg = sensor_msgs::msg::Image;
using TimeSyncPolicy = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg>;

class StereoSLAMNode : public rclcpp::Node {
public:
    StereoSLAMNode() : Node("stereo_slam_node") {
   
        this->declare_parameter("vocab_path", "");
        this->declare_parameter("settings_path", "");
        this->declare_parameter("map_frame", "map");
        this->declare_parameter("show_slam_viewer", false);

        std::string vocab_path = this->get_parameter("vocab_path").as_string();
        std::string settings_path = this->get_parameter("settings_path").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        bool show_slam_viewer = this->get_parameter("show_slam_viewer").as_bool();

        if (vocab_path.empty() || settings_path.empty()) {
            RCLCPP_FATAL(this->get_logger(), "Vocabulary and/or settings file paths are empty.");
            rclcpp::shutdown();
            return;
        }
        
        
        double fx = 474.896, fy = 474.896, cx = 400.0, cy = 300.0;
        cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

        // --- ORB-SLAM3 Initialization ---
        slam_ = std::make_shared<ORB_SLAM3::System>(vocab_path, settings_path, ORB_SLAM3::System::STEREO, show_slam_viewer);

        // --- Velocity Estimation Initialization ---
        velocity_est_ = std::make_unique<VelocityEstimation>(K);

        // --- ROS 2 Publishers ---
        path_pub_ = create_publisher<nav_msgs::msg::Path>("slam/camera_path", 10);
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("slam/camera_pose", 10);

        // --- ROS 2 Subscribers ---
        left_sub_.subscribe(this, "/stereo_left");
        right_sub_.subscribe(this, "/stereo_right");
        sync_ = std::make_shared<message_filters::Synchronizer<TimeSyncPolicy>>(
            TimeSyncPolicy(10), left_sub_, right_sub_);
        sync_->registerCallback(std::bind(&StereoSLAMNode::stereo_callback, this, 
                                         std::placeholders::_1, std::placeholders::_2));

        // --- Stereo Matcher ---
        stereo_matcher_ = cv::StereoBM::create(64, 15);
        
        RCLCPP_INFO(this->get_logger(), "Stereo SLAM node initialized.");
    }

    ~StereoSLAMNode() {
        slam_->Shutdown();
        cv::destroyAllWindows();
    }

private:
    void stereo_callback(const ImageMsg::ConstSharedPtr& left_msg, 
                        const ImageMsg::ConstSharedPtr& right_msg) {
        double current_time = left_msg->header.stamp.sec + 
                             left_msg->header.stamp.nanosec * 1e-9;
        
        cv_bridge::CvImageConstPtr left_cv_ptr, right_cv_ptr;
        try {
            left_cv_ptr = cv_bridge::toCvShare(left_msg, "bgr8");
            right_cv_ptr = cv_bridge::toCvShare(right_msg, "bgr8");
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat left_color = left_cv_ptr->image.clone();
        cv::Mat right_color = right_cv_ptr->image.clone();
        
        // --- Resize both images to 800x600 ---
        cv::Size target_size(800, 600);
        if (left_color.size() != target_size) {
            RCLCPP_WARN_ONCE(this->get_logger(), 
                "Left image: %dx%d. Resizing to 800x600",
                left_color.cols, left_color.rows);
            cv::resize(left_color, left_color, target_size);
        }
        if (right_color.size() != target_size) {
            RCLCPP_WARN_ONCE(this->get_logger(), 
                "Right image: %dx%d. Resizing to 800x600",
                right_color.cols, right_color.rows);
            cv::resize(right_color, right_color, target_size);
        }
        
        cv::Mat left_gray, right_gray;
        cv::cvtColor(left_color, left_gray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(right_color, right_gray, cv::COLOR_BGR2GRAY);

        // --- 1. ORB-SLAM3 Processing ---
        Sophus::SE3f pose_se3f = slam_->TrackStereo(left_gray, right_gray, current_time);
        Eigen::Matrix4f pose_eigen = pose_se3f.matrix();
        publish_current_pose(pose_eigen, left_msg->header);
        
        // --- 2. Velocity Estimation ---
        cv::Mat delta_pose_matrix;
        bool motion_estimated = velocity_est_->estimate_motion(left_gray, delta_pose_matrix);
        
        if (motion_estimated && last_time_ > 0) {
            double dt = current_time - last_time_;
            if (dt > 1e-6 && dt < 1.0) {
                // Extract rotation and translation
                cv::Mat R = delta_pose_matrix(cv::Rect(0, 0, 3, 3));
                cv::Mat t = delta_pose_matrix(cv::Rect(3, 0, 1, 3));

                // Linear velocity (translation / time)
                cv::Vec3d raw_linear_vel(t.at<double>(0, 0) / dt,t.at<double>(1, 0) / dt,t.at<double>(2, 0) / dt);
                smoothed_linear_velocity_ = linear_velocity_filter_.update(raw_linear_vel);

                // Angular velocity (from rotation matrix)
                cv::Mat rodrigues;
                cv::Rodrigues(R, rodrigues);
                cv::Vec3d raw_angular_vel(rodrigues.at<double>(0, 0) / dt,
                                         rodrigues.at<double>(1, 0) / dt,
                                         rodrigues.at<double>(2, 0) / dt);
                smoothed_angular_velocity_ = angular_velocity_filter_.update(raw_angular_vel);

                
                std::cout << "\nDelta Pose Matrix:" << std::endl;
                for (int i = 0; i < 4; i++) {
                    std::cout << "  [";
                    for (int j = 0; j < 4; j++) {
                        std::cout << std::fixed << std::setprecision(6) << std::setw(10) 
                                 << delta_pose_matrix.at<double>(i, j);
                        if (j < 3) std::cout << ",";
                    }
                    std::cout << " ]" << std::endl;
                }
                
                std::cout << "\nLinear Velocity (m/s):" << std::endl;
                std::cout << "  X: " << std::fixed << std::setprecision(4) << std::setw(8) 
                         << smoothed_linear_velocity_[0] << std::endl;
                std::cout << "  Y: " << std::fixed << std::setprecision(4) << std::setw(8) 
                         << smoothed_linear_velocity_[1] << std::endl;
                std::cout << "  Z: " << std::fixed << std::setprecision(4) << std::setw(8) 
                         << smoothed_linear_velocity_[2] << std::endl;
                
                std::cout << "\nAngular Velocity (rad/s):" << std::endl;
                std::cout << "  Roll:  " << std::fixed << std::setprecision(4) << std::setw(8) 
                         << smoothed_angular_velocity_[0] << std::endl;
                std::cout << "  Pitch: " << std::fixed << std::setprecision(4) << std::setw(8) 
                         << smoothed_angular_velocity_[1] << std::endl;
                std::cout << "  Yaw:   " << std::fixed << std::setprecision(4) << std::setw(8) 
                         << smoothed_angular_velocity_[2] << std::endl;
                std::cout << "========================================\n" << std::endl;
            }
        } else if (!motion_estimated) {
            std::cout << "[INFO] Initializing velocity estimation..." << std::endl;
        }
        last_time_ = current_time;

        // --- 3. Visualization ---
        update_2d_visualization(left_color, right_color, left_gray, right_gray);
    }

    void publish_current_pose(const Eigen::Matrix4f& pose_matrix, const std_msgs::msg::Header& header) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header = header;
        ps.header.frame_id = map_frame_;
        ps.pose.position.x = pose_matrix(0, 3);
        ps.pose.position.y = pose_matrix(1, 3);
        ps.pose.position.z = pose_matrix(2, 3);
        Eigen::Quaternionf q(pose_matrix.block<3, 3>(0, 0));
        ps.pose.orientation.x = q.x();
        ps.pose.orientation.y = q.y();
        ps.pose.orientation.z = q.z();
        ps.pose.orientation.w = q.w();
        pose_pub_->publish(ps);

        path_msg_.header = ps.header;
        path_msg_.poses.push_back(ps);
        path_pub_->publish(path_msg_);
    }

    void update_2d_visualization(const cv::Mat& left_color, const cv::Mat& right_color,const cv::Mat& left_gray, const cv::Mat& right_gray) {
        int display_w = 640, display_h = 480;
        cv::Size display_size(display_w, display_h);

        // Feature tracking visualization
        cv::Mat tracking_display = left_color.clone();
        std::vector<cv::KeyPoint> keypoints;
        velocity_est_->detect_features(left_gray, keypoints);
        cv::drawKeypoints(tracking_display, keypoints, tracking_display, 
                         cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DEFAULT);
        
        // Depth map
        cv::Mat disparity_map, disparity_normalized, depth_display;
        stereo_matcher_->compute(left_gray, right_gray, disparity_map);
        cv::normalize(disparity_map, disparity_normalized, 0, 255, cv::NORM_MINMAX, CV_8U);
        cv::applyColorMap(disparity_normalized, depth_display, cv::COLORMAP_JET);

        // Resize for display
        cv::Mat left_resized, right_resized, tracking_resized, depth_resized;
        cv::resize(left_color, left_resized, display_size);
        cv::resize(right_color, right_resized, display_size);
        cv::resize(tracking_display, tracking_resized, display_size);
        cv::resize(depth_display, depth_resized, display_size);

        // Add velocity text
        std::string lin_vel_text = cv::format("Lin: [%.2f, %.2f, %.2f] m/s", 
            smoothed_linear_velocity_[0], smoothed_linear_velocity_[1], smoothed_linear_velocity_[2]);
        std::string ang_vel_text = cv::format("Ang: [%.2f, %.2f, %.2f] rad/s", 
            smoothed_angular_velocity_[0], smoothed_angular_velocity_[1], smoothed_angular_velocity_[2]);
        cv::putText(tracking_resized, lin_vel_text, cv::Point(10, display_h - 35), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
        cv::putText(tracking_resized, ang_vel_text, cv::Point(10, display_h - 10), 
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);

        // Add labels
        add_label(left_resized, "LEFT CAMERA");
        add_label(right_resized, "RIGHT CAMERA");
        add_label(tracking_resized, "TRACKING");
        add_label(depth_resized, "DEPTH MAP");

        // Combine views
        cv::Mat top_row, bottom_row, combined_view;
        cv::hconcat(left_resized, right_resized, top_row);
        cv::hconcat(tracking_resized, depth_resized, bottom_row);
        cv::vconcat(top_row, bottom_row, combined_view);

        cv::imshow("Stereo SLAM Monitor", combined_view);
        cv::waitKey(1);
    }

    void add_label(cv::Mat& image, const std::string& label) {
        int font = cv::FONT_HERSHEY_SIMPLEX;
        double font_scale = 0.7;
        int thickness = 2;
        cv::Size text_size = cv::getTextSize(label, font, font_scale, thickness, nullptr);
        cv::Point text_org((image.cols - text_size.width) / 2, text_size.height + 10);
        cv::rectangle(image, text_org + cv::Point(-5, 5), text_org + cv::Point(text_size.width, -text_size.height) + cv::Point(5, -5), cv::Scalar(0, 0, 0), cv::FILLED);
        cv::putText(image, label, text_org, font, font_scale, cv::Scalar(255, 255, 255), thickness);
    }

    // Member Variables
    std::shared_ptr<ORB_SLAM3::System> slam_;
    std::unique_ptr<VelocityEstimation> velocity_est_;
    
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    
    message_filters::Subscriber<ImageMsg> left_sub_;
    message_filters::Subscriber<ImageMsg> right_sub_;
    std::shared_ptr<message_filters::Synchronizer<TimeSyncPolicy>> sync_;
    
    cv::Ptr<cv::StereoBM> stereo_matcher_;
    nav_msgs::msg::Path path_msg_;
    std::string map_frame_;

    double last_time_ = -1.0;
    MovingAverageFilter linear_velocity_filter_;
    MovingAverageFilter angular_velocity_filter_;
    cv::Vec3d smoothed_linear_velocity_{0, 0, 0};
    cv::Vec3d smoothed_angular_velocity_{0, 0, 0};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StereoSLAMNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}