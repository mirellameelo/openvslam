#ifdef USE_PANGOLIN_VIEWER
    #include <pangolin_viewer/viewer.h>
    #include <openvslam/publish/map_publisher.h>
#elif USE_SOCKET_PUBLISHER
    #include <socket_publisher/publisher.h>
#endif

#include <openvslam/system.h>
#include <openvslam/config.h>

#include <iostream>
#include <chrono>
#include <numeric>

#include "openvslam/data/keyframe.h"
#include "openvslam/data/landmark.h"

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <tf2_ros/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>
#include <popl.hpp>

#ifdef USE_STACK_TRACE_LOGGER
#include <glog/logging.h>
#endif

#ifdef USE_GOOGLE_PERFTOOLS
#include <gperftools/profiler.h>
#endif


tf2::Vector3 publi(auto cam_pose_, auto odometry_pub_, auto node){
    Eigen::Matrix3d rotation_matrix = cam_pose_.block(0, 0, 3, 3);
    Eigen::Vector3d translation_vector = cam_pose_.block(0, 3, 3, 1);

    tf2::Matrix3x3 tf_rotation_matrix(rotation_matrix(0, 0), rotation_matrix(0, 1), rotation_matrix(0, 2),
                                      rotation_matrix(1, 0), rotation_matrix(1, 1), rotation_matrix(1, 2),
                                      rotation_matrix(2, 0), rotation_matrix(2, 1), rotation_matrix(2, 2));
    
    tf2::Vector3 tf_translation_vector(translation_vector(0), translation_vector(1), translation_vector(2));

    //Coordinate transformation matrix from orb coordinate system to ros coordinate system
    tf2::Matrix3x3 tf_open_to_ros (0, 0, 1,
                                 -1, 0, 0,
                                 0,-1, 0);

    //Transform actual coordinate system to ros coordinate system on camera coordinates
    tf_rotation_matrix = tf_open_to_ros*tf_rotation_matrix;
    tf_translation_vector = tf_open_to_ros*tf_translation_vector;

    tf_rotation_matrix = tf_rotation_matrix.transpose();
    tf_translation_vector = -(tf_rotation_matrix*tf_translation_vector);

    tf2::Transform transform_tf(tf_rotation_matrix, tf_translation_vector);

    //RETORNAR ESTE CARINHA AQUI: tf_translation_vector

    // Create odometry message and update it with current camera pose
    nav_msgs::msg::Odometry odom_msg_;
    odom_msg_.header.stamp = node->now();
    odom_msg_.header.frame_id = "map";
    odom_msg_.child_frame_id = "base_link_frame";
    odom_msg_.pose.pose.orientation.x = transform_tf.getRotation().getX();
    odom_msg_.pose.pose.orientation.y = transform_tf.getRotation().getY();
    odom_msg_.pose.pose.orientation.z = transform_tf.getRotation().getZ();
    odom_msg_.pose.pose.orientation.w = transform_tf.getRotation().getW();

    odom_msg_.pose.pose.position.x = transform_tf.getOrigin().getX();
    odom_msg_.pose.pose.position.y = 0.0; //transform_tf.getOrigin().getY();
    odom_msg_.pose.pose.position.z = transform_tf.getOrigin().getZ();
    odometry_pub_->publish(odom_msg_);

    return tf_translation_vector;
}

void mono_tracking(const std::shared_ptr<openvslam::config>& cfg, const std::string& vocab_file_path,
                    const std::string& mask_img_path, const bool eval_log, const std::string& map_db_path){
    // load the mask image
    const cv::Mat mask = mask_img_path.empty() ? cv::Mat{} : cv::imread(mask_img_path, cv::IMREAD_GRAYSCALE);
    // build a SLAM system
    openvslam::system SLAM(cfg, vocab_file_path);
    // startup the SLAM process
    SLAM.startup();
    // create a viewer object
    // and pass the frame_publisher and the map_publisher
    #ifdef USE_PANGOLIN_VIEWER
        pangolin_viewer::viewer viewer(cfg, &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
    #elif USE_SOCKET_PUBLISHER
        socket_publisher::publisher publisher(cfg, &SLAM, SLAM.get_frame_publisher(), SLAM.get_map_publisher());
    #endif

    // initialize this node
    auto node = std::make_shared<rclcpp::Node>("run_slam");
    rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
    custom_qos.depth = 1;


    // cria um topico, canal de envio de msg
    auto point_cloud_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud");
    sensor_msgs::msg::PointCloud2::SharedPtr pc2_msg_;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_;
    // line
    auto line_node = node->create_publisher<visualization_msgs::msg::Marker>("line");
    // cria um topico, canal de envio de msg
    auto odometry_pub_ = node->create_publisher<nav_msgs::msg::Odometry>("pose", 1);

    const auto tp_0 = std::chrono::steady_clock::now();
    
    visualization_msgs::msg::Marker line;
    line.header.frame_id = "map";
    line.type = line.LINE_LIST;
    line.action = line.ADD;
    // marker scale
    line.scale.x = 0.003;
    // marker color
    line.color.a = 1.0;
    line.color.r = 1.0;
    line.color.g = 1.0;

    // run the SLAM as subscriber
    image_transport::Subscriber sub = image_transport::create_subscription(
        node.get(), "/video/image_raw", [&](const sensor_msgs::msg::Image::ConstSharedPtr& msg) {


            const auto tp_1 = std::chrono::steady_clock::now();
            const auto timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(tp_1 - tp_0).count();
            // input the current frame and estimate the camera pose
            auto cam = SLAM.feed_monocular_frame(cv_bridge::toCvShare(msg, "bgr8")->image, timestamp, mask);

            
            geometry_msgs::msg::Point list_point;
            geometry_msgs::msg::Point cam_pose_ros;
            auto cam_pose_xyz = publi(cam, odometry_pub_, node);
            cam_pose_ros.x = -cam_pose_xyz[0];
            cam_pose_ros.y = 0.0;
            cam_pose_ros.z = cam_pose_xyz[2];
            int y;
            auto all_keyframes = SLAM.get_keyframes();
            if(!all_keyframes.empty()){
                for(y = 0; y < all_keyframes.size(); y++){
                    if(all_keyframes[y]->id_ == (all_keyframes.size()-1)){

                        // acessar todos os keyframes
                        cloud_.clear();
                        line.points.clear();
                        // acessar cada landmark visualizado por ele
                        // get just the last keyframe from all 
                        std::set<openvslam::data::landmark*> landm = all_keyframes[y]->get_valid_landmarks();
                        //std::cout << all_keyframes[y]->id_ << std::endl;
                            
                        for (std::set<openvslam::data::landmark*>::iterator it=landm.begin(); it!=landm.end(); ++it){

                            pcl::PointXYZRGB pt;

                            auto point_pos = (*it)->get_pos_in_world();
                            list_point.x = -point_pos[0];
                            list_point.y = 0;
                            list_point.z = point_pos[2];
                            line.points.push_back(cam_pose_ros);
                            line.points.push_back(list_point);

                            pt.x = point_pos[0];
                            pt.y = 0.0;
                            pt.z = point_pos[2];
                            cloud_.points.push_back(pt);
                    }
                }

                }
                
                pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
                pcl::toROSMsg(cloud_, *pc2_msg_);
                pc2_msg_->header.frame_id = "map";
                pc2_msg_->header.stamp = node->now();
                point_cloud_->publish(pc2_msg_);
                line_node->publish(line);
            }

        },
        "raw", custom_qos);

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);

    // Pangolin needs to run in the main thread on OSX
    std::thread thread([&]() {
        exec.spin();
    });

    #ifdef USE_PANGOLIN_VIEWER
        viewer.run();
        if (SLAM.terminate_is_requested()) {
            // wait until the loop BA is finished
            while (SLAM.loop_BA_is_running()) {
                std::this_thread::sleep_for(std::chrono::microseconds(5000));

            }
            rclcpp::shutdown();
        }
    #elif USE_SOCKET_PUBLISHER
        publisher.run();
        if (SLAM.terminate_is_requested()) {
            // wait until the loop BA is finished
            while (SLAM.loop_BA_is_running()) {
                std::this_thread::sleep_for(std::chrono::microseconds(5000));
            }
            rclcpp::shutdown();
        }
    #endif

    // automatically close the viewer
    #ifdef USE_PANGOLIN_VIEWER
        viewer.request_terminate();
        thread.join();
    #elif USE_SOCKET_PUBLISHER
        publisher.request_terminate();
        thread.join();
    #endif

    // shutdown the SLAM process
    SLAM.shutdown();

    if (eval_log) {
        // output the trajectories for evaluation
        SLAM.save_frame_trajectory("/home/mirellameelo/openvslam/ros2/maps/frame_trajectory.txt", "TUM");
        SLAM.save_keyframe_trajectory("/home/mirellameelo/openvslam/ros2/maps/KF_trajectory.txt", "TUM");
        SLAM.save_json_file("/home/mirellameelo/openvslam/ros2/maps/xyz.txt");
    }

    if (!map_db_path.empty()) {
        // output the map database
        SLAM.save_map_database(map_db_path);
    }

}

int main(int argc, char* argv[]) {

    #ifdef USE_STACK_TRACE_LOGGER
        google::InitGoogleLogging(argv[0]);
        google::InstallFailureSignalHandler();
    #endif
        rclcpp::init(argc, argv);

    // create options
    popl::OptionParser op("Allowed options");
    auto help = op.add<popl::Switch>("h", "help", "produce help message");
    auto vocab_file_path = op.add<popl::Value<std::string>>("v", "vocab", "vocabulary file path");
    auto setting_file_path = op.add<popl::Value<std::string>>("c", "config", "setting file path");
    auto mask_img_path = op.add<popl::Value<std::string>>("", "mask", "mask image path", "");
    auto debug_mode = op.add<popl::Switch>("", "debug", "debug mode");
    //switch: por padrao eh false, se vc setar, fica true
    //value: exige um valor apos usar a palavra na linha de comando, ex: --map-db "STRING"
    auto eval_log = op.add<popl::Switch>("", "eval-log", "store trajectory and tracking times for evaluation");
    auto map_db_path = op.add<popl::Value<std::string>>("", "map-db", "store a map database at this path after SLAM", "");
    try {
        op.parse(argc, argv);
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // check validness of options
    if (help->is_set()) {
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }
    if (!vocab_file_path->is_set() || !setting_file_path->is_set()) {
        std::cerr << "invalid arguments" << std::endl;
        std::cerr << std::endl;
        std::cerr << op << std::endl;
        return EXIT_FAILURE;
    }

    // setup logger
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] %^[%L] %v%$");
    if (debug_mode->is_set()) {
        spdlog::set_level(spdlog::level::debug);
    }
    else {
        spdlog::set_level(spdlog::level::info);
    }

    // load configuration
    std::shared_ptr<openvslam::config> cfg;
    try {
        cfg = std::make_shared<openvslam::config>(setting_file_path->value());
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    #ifdef USE_GOOGLE_PERFTOOLS
        ProfilerStart("slam.prof");
    #endif

    // run tracking
    if (cfg->camera_->setup_type_ == openvslam::camera::setup_type_t::Monocular) {
        mono_tracking(cfg, vocab_file_path->value(), mask_img_path->value(), eval_log->is_set(), map_db_path->value());
    }
    else {
        throw std::runtime_error("Invalid setup type: " + cfg->camera_->get_setup_type_string());
    }

    #ifdef USE_GOOGLE_PERFTOOLS
        ProfilerStop();
    #endif

    return EXIT_SUCCESS;
}
