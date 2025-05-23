#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include "System.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <chrono>
#include <atomic>

using std::placeholders::_1;

class SLAMStabilizer : public rclcpp::Node {
public:
    SLAMStabilizer(
        const std::string& camera_topic_name,
        const std::string& image_format,
        const std::string& vocab_path,
        const std::string& camera_calibration_path
    ): Node("orb_slam3_stabilization") {
        slam = new ORB_SLAM3::System(vocab_path, camera_calibration_path, ORB_SLAM3::System::MONOCULAR, true);

        velocity_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);

        sub_movement_k = this->create_subscription<std_msgs::msg::Float64>(
            "/stabilization/movement_k", 1, std::bind(&SLAMStabilizer::movementCallback, this, _1)
        );

        sub_rotation_k = this->create_subscription<std_msgs::msg::Float64>(
            "/stabilization/rotation_k", 1, std::bind(&SLAMStabilizer::rotationCallback, this, _1)
        );

        sub_hold = this->create_subscription<std_msgs::msg::Bool>(
            "/stabilization/enable_hold", 1, std::bind(&SLAMStabilizer::holdCallback, this, _1)
        );

        sub_save_pose = this->create_subscription<std_msgs::msg::Empty>(
            "/stabilization/save_current_pose", 1, std::bind(&SLAMStabilizer::saveCurrentPoseCallback, this, _1)
        );

        sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(
            "/mavros/imu/data", 1000, std::bind(&SLAMStabilizer::addImuToBuffer, this, _1)
        );

        if (image_format == "raw") {
            sub_image = this->create_subscription<sensor_msgs::msg::Image>(
                camera_topic_name, 100, std::bind(&SLAMStabilizer::rawImageCallback, this, _1)
            );
        } else if (image_format == "compressed_jpeg") {
            sub_compressed_image = this->create_subscription<sensor_msgs::msg::CompressedImage>(
                camera_topic_name, 100, std::bind(&SLAMStabilizer::compressedImageCallback, this, _1)
            );
        }

        stabilization_thread = std::thread(&SLAMStabilizer::doStabilizationSync, this);
    }

    ~SLAMStabilizer() {
        slam->Shutdown();
        delete slam;
    }

private:
    ORB_SLAM3::System* slam;
    Sophus::SE3f checkpoint_pose;
    std::atomic<bool> has_checkpoint{false};
    std::atomic<bool> should_save_pose{false};
    std::atomic<bool> should_go_to_pose{false};

    double movement_k = 5.0;
    double rotation_k = 0.25;

    std::queue<sensor_msgs::msg::Imu::SharedPtr> imuBuf;
    std::mutex imuBufMutex;

    std::queue<cv_bridge::CvImageConstPtr> imgBuf;
    std::mutex imgBufMutex;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_movement_k;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_rotation_k;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_hold;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_save_pose;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_compressed_image;

    std::thread stabilization_thread;

    void movementCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        std::cout << msg->data << "movementCallback:\n";
        movement_k = msg->data;
    }

    void rotationCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        rotation_k = msg->data;
    }

    void holdCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        should_go_to_pose = msg->data;
    }

    void saveCurrentPoseCallback(const std_msgs::msg::Empty::SharedPtr) {
        should_save_pose = true;
    }

    void saveCheckpointPose(const Sophus::SE3f& pose) {
        checkpoint_pose = pose;
        has_checkpoint = true;
        should_save_pose = false;

        Eigen::Vector3f translation = checkpoint_pose.translation();
        std::cout << "Saved Pose:\n";
        std::cout << "x: " << translation.x() << ", y: " << translation.y() << ", z: " << translation.z() << std::endl;
    }

    void addImuToBuffer(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
        imuBufMutex.lock();
        imuBuf.push(imu_msg);
        imuBufMutex.unlock();
    }

    void addImageToBuffer(const cv_bridge::CvImageConstPtr& cv_ptr) {
        imgBufMutex.lock();
        imgBuf.push(cv_ptr);
        imgBufMutex.unlock();
    }

    geometry_msgs::msg::TwistStamped offsetToDroneVelocity(
        const Eigen::Vector3f& offset,
        const Eigen::Matrix3f& rot_offset
    ) {
        geometry_msgs::msg::TwistStamped velocity_command;
        velocity_command.header.stamp = this->now();
        velocity_command.header.frame_id = "base_link";

        float pitch = std::asin(-rot_offset(2, 0));
        float yaw = std::atan2(rot_offset(1, 0), rot_offset(0, 0));
        float roll = std::atan2(rot_offset(2, 1), rot_offset(2, 2));
        std::cout << "pitch: " << pitch << ", yaw: " << yaw << ", roll: " << roll << std::endl;

        velocity_command.twist.linear.x = offset.y() * movement_k;
        velocity_command.twist.linear.y = offset.x() * movement_k;
        velocity_command.twist.linear.z = -offset.z() * movement_k;
        velocity_command.twist.angular.x = -roll * rotation_k;
        velocity_command.twist.angular.y = -pitch * rotation_k;
        velocity_command.twist.angular.z = -yaw * rotation_k;
        return velocity_command;
    }

    void processDataAndStibilize(
        const cv_bridge::CvImageConstPtr& cv_ptr,
        const std::vector<ORB_SLAM3::IMU::Point>& imu_vector
    ) {
        double timestamp = rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds();
        Sophus::SE3f pose = slam->TrackMonocular(cv_ptr->image, timestamp, imu_vector);
        if (pose.translation().norm() == 0) return;
        
        if (should_save_pose) {
            saveCheckpointPose(pose);
        }

        if (!has_checkpoint) return;
        if (!should_go_to_pose) return;

        Eigen::Vector3f offset = pose.translation() - checkpoint_pose.translation();
        Eigen::Matrix3f rotation_offset = pose.rotationMatrix() * checkpoint_pose.rotationMatrix().transpose();

        std::cout << "dx: " << offset.x() << ", dy: " << offset.y() << ", dz: " << offset.z() << std::endl;
        std::cout << "norm" << offset.norm() << "\n";

        if (offset.norm() < 0.002) return;

        geometry_msgs::msg::TwistStamped cmd_vel_stamped = offsetToDroneVelocity(offset, rotation_offset);
        velocity_pub->publish(cmd_vel_stamped);
    }

    cv_bridge::CvImageConstPtr extractOldestImage() {
        while (1) {
            if (imgBuf.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
        
            imgBufMutex.lock();
            cv_bridge::CvImageConstPtr cv_img = imgBuf.front();
            imgBuf.pop();
            imgBufMutex.unlock();
        
            return cv_img;
        }
    }

    std::vector<ORB_SLAM3::IMU::Point> extractImuPointsUpTo(const rclcpp::Time& image_timestamp) {
        std::vector<ORB_SLAM3::IMU::Point> imu_vector;

        std::lock_guard<std::mutex> lock(imuBufMutex);
        while (!imuBuf.empty()) {
            auto imu_msg = imuBuf.front();
            auto imu_timestamp =  rclcpp::Time(imu_msg->header.stamp);
            if (imu_timestamp > image_timestamp) {
                break;
            }

            cv::Point3f acc(
                imu_msg->linear_acceleration.x,
                imu_msg->linear_acceleration.y,
                imu_msg->linear_acceleration.z
            );
            cv::Point3f gyr(
                imu_msg->angular_velocity.x,
                imu_msg->angular_velocity.y,
                imu_msg->angular_velocity.z
            );

            std::cout << "Got IMU:" << imu_timestamp.seconds() << "\n";
            imu_vector.push_back(ORB_SLAM3::IMU::Point(acc, gyr, imu_timestamp.seconds()));
            imuBuf.pop();
        }

        return imu_vector;
    }

    void doStabilizationSync() {
        while (1) {        
            cv_bridge::CvImageConstPtr cv_img = extractOldestImage();
            std::cout << "Got image:" << rclcpp::Time(cv_img->header.stamp).seconds() << "\n";
            std::vector<ORB_SLAM3::IMU::Point> imu_points = extractImuPointsUpTo(cv_img->header.stamp);
            processDataAndStibilize(cv_img, imu_points);

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    
    void rawImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            addImageToBuffer(cv_bridge::toCvShare(msg));
        } catch (cv_bridge::Exception& e) {
            throw std::runtime_error("cv_bridge exception: " + std::string(e.what()));
        }
    }
    

    void compressedImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        try {
            cv::Mat decoded_image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    
            cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
            cv_ptr->image = decoded_image;
            cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;
            cv_ptr->header = msg->header;
    
            addImageToBuffer(cv_ptr);
        } catch (const std::exception& e) {
            throw std::runtime_error("cv_bridge exception: " + std::string(e.what()));
        }
    }
};

std::string getValidatedImageFormat(const std::string& image_format)
{
    bool is_valid = (image_format == "raw") || (image_format == "compressed_jpeg");
    if (!is_valid)
    {
        cerr << endl << "Error: Invalid image format '" << image_format << "'. Allowed formats: 'raw', 'compressed_jpeg'" << endl;
        rclcpp::shutdown();
        exit(1);
    }

    return image_format;
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    if(argc != 5) {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Stabilization $camera_topic_name $image_format $path_to_vocabulary $path_to_settings" << endl;        
        rclcpp::shutdown();
        return 1;
    }   

    std::string camera_topic_name = argv[1];
    std::string image_format = getValidatedImageFormat(argv[2]);
    std::string vocab_path = argv[3];
    std::string camera_calibration_path = argv[4];

    auto node = std::make_shared<SLAMStabilizer>(camera_topic_name, image_format, vocab_path, camera_calibration_path);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
