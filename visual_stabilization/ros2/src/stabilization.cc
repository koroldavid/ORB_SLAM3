#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include "System.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <chrono>
#include <atomic>

using sensor_msgs::msg::CompressedImage;
using std::placeholders::_1;

class SLAMStabilizer : public rclcpp::Node {
public:
    SLAMStabilizer(
        const std::string& camera_topic_name_left,
        const std::string& camera_topic_name_right,
        const std::string& vocab_path,
        const std::string& camera_calibration_path
    ): Node("orb_slam3_stabilization") {

        slam = new ORB_SLAM3::System(vocab_path, camera_calibration_path, ORB_SLAM3::System::STEREO, true);

        vision_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/vision_pose/pose", 10);

        img_left_sub.subscribe(this, camera_topic_name_left);
        img_right_sub.subscribe(this, camera_topic_name_right);
        image_sync_sub = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), img_left_sub, img_right_sub);
        image_sync_sub->registerCallback(&SLAMStabilizer::compressedImageCallback, this);
    }

    ~SLAMStabilizer() {
        slam->Shutdown();
        delete slam;
    }

private:
    ORB_SLAM3::System* slam;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr vision_pose_pub;
    
    message_filters::Subscriber<sensor_msgs::msg::CompressedImage> img_left_sub;
    message_filters::Subscriber<sensor_msgs::msg::CompressedImage> img_right_sub;

    typedef message_filters::sync_policies::ApproximateTime<CompressedImage, CompressedImage> SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> image_sync_sub;

    std::thread stabilization_thread;

    void compressedImageCallback(
        const sensor_msgs::msg::CompressedImage::SharedPtr left,
        const sensor_msgs::msg::CompressedImage::SharedPtr right
    ) {
        try {
            cv::Mat decoded_image_left = cv::imdecode(cv::Mat(left->data), cv::IMREAD_COLOR);

            cv_bridge::CvImagePtr cv_ptr_left(new cv_bridge::CvImage);
            cv_ptr_left->image = decoded_image_left;
            cv_ptr_left->encoding = sensor_msgs::image_encodings::BGR8;
            cv_ptr_left->header = left->header;

            cv::Mat decoded_image_right = cv::imdecode(cv::Mat(right->data), cv::IMREAD_COLOR);

            cv_bridge::CvImagePtr cv_ptr_right(new cv_bridge::CvImage);
            cv_ptr_right->image = decoded_image_right;
            cv_ptr_right->encoding = sensor_msgs::image_encodings::BGR8;
            cv_ptr_right->header = right->header;

            processDataAndStibilize(cv_ptr_left, cv_ptr_right);
        } catch (const std::exception& e) {
            throw std::runtime_error("cv_bridge exception: " + std::string(e.what()));
        }
    }

    void processDataAndStibilize(
        const cv_bridge::CvImageConstPtr& left,
        const cv_bridge::CvImageConstPtr& right
    ) {
        double timestamp = rclcpp::Time(left->header.stamp).seconds();
        Sophus::SE3f pose = slam->TrackStereo(left->image,right->image,timestamp);

        vision_pose_pub->publish(trasform_slam_pose_to_vision_pose(pose));
    }

    geometry_msgs::msg::PoseStamped trasform_slam_pose_to_vision_pose(const Sophus::SE3f& pose) {
        float roll_cam = 0.0;           // Camera rotation around X (rad)
        float pitch_cam = 0.0;          // Camera rotation around Y (rad)
        float yaw_cam = -4.7123889f;    // Camera rotation around Z (rad)
        float scaleX = 0.33;
        float scaleY = 0.33;
        float scaleZ = 0.33;
        
        // 1. Scale coordinates to represent real world.
        Eigen::Vector3f position_orig = pose.translation();
        Eigen::Matrix3f rotation_orig = pose.rotationMatrix();
        float gamma_world = std::atan2(rotation_orig(1, 0), rotation_orig(0, 0));
        
        // 2. Rotation from original world frame to world frame with Y forward
        // We use gamma from SLAM yaw, as SLAM switch x and y pose based on its yaw orientation.
        // Shortly we reset coordinates to direction used on yaw 0, when SLAM was initialized.
        Eigen::Vector3f position_body;
        position_body.x() =  std::cos(gamma_world) * position_orig.x() + std::sin(gamma_world) * position_orig.y();
        position_body.y() = -std::sin(gamma_world) * position_orig.x() + std::cos(gamma_world) * position_orig.y();
        position_body.z() = position_orig.z();

        // 3. Camera to body frame rotation
        Eigen::Quaternionf quat_camera = Eigen::Quaternionf(rotation_orig);
        Eigen::Quaternionf quat_camera_x = Eigen::Quaternionf(Eigen::AngleAxisf(roll_cam,  Eigen::Vector3f::UnitX()));
        Eigen::Quaternionf quat_camera_y = Eigen::Quaternionf(Eigen::AngleAxisf(pitch_cam, Eigen::Vector3f::UnitY()));
        Eigen::Quaternionf quat_camera_z = Eigen::Quaternionf(Eigen::AngleAxisf(yaw_cam,   Eigen::Vector3f::UnitZ()));

        Eigen::Quaternionf quat_body = quat_camera * quat_camera_x * quat_camera_y * quat_camera_z;
        quat_body.normalize();

        // 4. Convert left-handed rotation to right-handed
        Eigen::Quaternionf quat_right_hand = Eigen::Quaternionf(quat_body.w(),  - quat_body.x(), - quat_body.y(), quat_body.z());
        quat_right_hand.normalize();

        // Output
        geometry_msgs::msg::PoseStamped msg_body_pose; 
        msg_body_pose.header.stamp = this->now();
        msg_body_pose.header.frame_id = "map";
        // Convet to WNU to ENU by negating some coordinates
        msg_body_pose.pose.position.x = -position_body.x() * scaleX;
        msg_body_pose.pose.position.y = position_body.y() * scaleY;
        msg_body_pose.pose.position.z = position_body.z() * scaleZ;
        msg_body_pose.pose.orientation.x = -quat_right_hand.y();
        msg_body_pose.pose.orientation.y = -quat_right_hand.x();
        msg_body_pose.pose.orientation.z = quat_right_hand.z();
        msg_body_pose.pose.orientation.w = quat_right_hand.w();
        return msg_body_pose;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    if(argc != 5) {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Stabilization $camera_topic_name_left $camera_topic_name_right $path_to_vocabulary $path_to_settings" << endl;        
        rclcpp::shutdown();
        return 1;
    }   

    std::string camera_topic_name_left = argv[1];
    std::string camera_topic_name_right = argv[2];
    std::string vocab_path = argv[3];
    std::string camera_calibration_path = argv[4];

    auto node = std::make_shared<SLAMStabilizer>(camera_topic_name_left, camera_topic_name_right, vocab_path, camera_calibration_path);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
