#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <fenv.h>
#include <unistd.h>
#include <iomanip>
#include <std_srvs/srv/trigger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/vector3.hpp>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>

#include "IMU_Processing.hpp"

#include "matching/matching.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/string.hpp>

#define INIT_TIME (0.1)
#define LASER_POINT_COV (0.1)
#define PUBFRAME_PERIOD (20)

/*** Time Log Variables ***/
int add_point_size = 0, kdtree_delete_counter = 0;
bool pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;

std::mutex mtx_buffer;
std::condition_variable sig_buffer;

std::string root_dir = ROOT_DIR;
std::string map_file_path, lid_topic, imu_topic;

double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int scan_count = 0, publish_count = 0;
int feats_down_size = 0, NUM_MAX_ITERATIONS = 0, pcd_index = 0;

bool lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;

std::vector<BoxPointType> cub_needrm;
std::vector<PointVector> Nearest_Points;
std::vector<double> extrinT(3, 0.0);
std::vector<double> extrinR(9, 0.0);
std::vector<double> init_pos(3, 0.0);
std::vector<double> init_rot{0, 0, 0, 1};
std::deque<double> time_buffer;
std::deque<PointCloudXYZI::Ptr> lidar_buffer;
std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_buffer;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr cloud(new PointCloudXYZI());

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE<PointType> ikdtree;

V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;

esekfom::esekf kf;

state_ikfom state_point;
Eigen::Vector3d pos_lid;
Eigen::Vector3d pos_lid_filtered(0, 0, 0);
bool pos_lid_filter_inited = false;
const double ema_alpha = 0.1;

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
geometry_msgs::msg::PoseStamped msg_body_pose;

std::shared_ptr<Preprocess> p_pre(new Preprocess());

// 全局SIGFPE处理器(必须有C链接)
extern "C" void global_fpe_handler(int) {
    write(STDERR_FILENO, "[FPE] 浮点异常已被拦截\n", 21);
}

//relocalization
std::string globalmap_dir;

class FastLioNode : public rclcpp::Node
{
public:
    FastLioNode() : Node("laserMapping")
    {
        // 全局禁用浮点异常，防止NDT/PCL内部崩溃
        fedisableexcept(FE_ALL_EXCEPT);
        signal(SIGFPE, global_fpe_handler);
        
        // 创建TF广播器
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        // 声明参数
        this->declare_parameter("publish/path_en", true);
        this->declare_parameter("publish/scan_publish_en", true);
        this->declare_parameter("publish/dense_publish_en", true);
        this->declare_parameter("publish/scan_bodyframe_pub_en", true);
        this->declare_parameter("max_iteration", 4);
        this->declare_parameter("map_file_path", "");
        this->declare_parameter("common/lid_topic", "/livox/lidar");
        this->declare_parameter("common/imu_topic", "/livox/imu");
        this->declare_parameter("common/time_sync_en", false);
        this->declare_parameter("common/time_offset_lidar_to_imu", 0.0);
        this->declare_parameter("filter_size_corner", 0.5);
        this->declare_parameter("filter_size_surf", 0.5);
        this->declare_parameter("filter_size_map", 0.5);
        this->declare_parameter("cube_side_length", 200.0);
        this->declare_parameter("mapping/det_range", 300.0f);
        this->declare_parameter("mapping/fov_degree", 180.0);
        this->declare_parameter("mapping/gyr_cov", 0.1);
        this->declare_parameter("mapping/acc_cov", 0.1);
        this->declare_parameter("mapping/b_gyr_cov", 0.0001);
        this->declare_parameter("mapping/b_acc_cov", 0.0001);
        this->declare_parameter("preprocess/blind", 0.01);
        this->declare_parameter("preprocess/lidar_type", static_cast<int>(AVIA));
        this->declare_parameter("preprocess/scan_line", 16);
        this->declare_parameter("preprocess/timestamp_unit", static_cast<int>(US));
        this->declare_parameter("preprocess/scan_rate", 10);
        this->declare_parameter("point_filter_num", 2);
        this->declare_parameter("feature_extract_enable", false);
        this->declare_parameter("mapping/extrinsic_est_en", true);
        this->declare_parameter("pcd_save/pcd_save_en", false);
        this->declare_parameter("mapping/extrinsic_T", std::vector<double>());
        this->declare_parameter("mapping/extrinsic_R", std::vector<double>());
        this->declare_parameter("mapping/init_pos", std::vector<double>());
        this->declare_parameter("mapping/init_rot", std::vector<double>());
        this->declare_parameter("globalmap_dir", "/Downloads/LOAM/");
        this->declare_parameter("relocal_publish/relocal_odom_en", false);
        this->declare_parameter("relocal_publish/relocal_scan_en", false);
        this->declare_parameter("relocal_publish/localmap_en", false);
        this->declare_parameter("ininit_resolution_dis", 0.05);
        this->declare_parameter("ininit_resolution_rot", 1.0);
        this->declare_parameter("init_threshold", 0.15);
        
        // 获取参数
        this->get_parameter("publish/path_en", path_en);
        this->get_parameter("publish/scan_publish_en", scan_pub_en);
        this->get_parameter("publish/dense_publish_en", dense_pub_en);
        this->get_parameter("publish/scan_bodyframe_pub_en", scan_body_pub_en);
        this->get_parameter("max_iteration", NUM_MAX_ITERATIONS);
        this->get_parameter("map_file_path", map_file_path);
        this->get_parameter("common/lid_topic", lid_topic);
        this->get_parameter("common/imu_topic", imu_topic);
        this->get_parameter("common/time_sync_en", time_sync_en);
        this->get_parameter("common/time_offset_lidar_to_imu", time_diff_lidar_to_imu);
        this->get_parameter("filter_size_corner", filter_size_corner_min);
        this->get_parameter("filter_size_surf", filter_size_surf_min);
        this->get_parameter("filter_size_map", filter_size_map_min);
        this->get_parameter("cube_side_length", cube_len);
        
        float det_range;
        this->get_parameter("mapping/det_range", det_range);
        DET_RANGE = det_range;
        
        this->get_parameter("mapping/fov_degree", fov_deg);
        this->get_parameter("mapping/gyr_cov", gyr_cov);
        this->get_parameter("mapping/acc_cov", acc_cov);
        this->get_parameter("mapping/b_gyr_cov", b_gyr_cov);
        this->get_parameter("mapping/b_acc_cov", b_acc_cov);
        this->get_parameter("preprocess/blind", p_pre->blind);
        
        int lidar_type;
        this->get_parameter("preprocess/lidar_type", lidar_type);
        p_pre->lidar_type = lidar_type;
        
        int n_scans;
        this->get_parameter("preprocess/scan_line", n_scans);
        p_pre->N_SCANS = n_scans;
        
        int time_unit;
        this->get_parameter("preprocess/timestamp_unit", time_unit);
        p_pre->time_unit = time_unit;
        
        int scan_rate;
        this->get_parameter("preprocess/scan_rate", scan_rate);
        p_pre->SCAN_RATE = scan_rate;
        
        int point_filter_num;
        this->get_parameter("point_filter_num", point_filter_num);
        p_pre->point_filter_num = point_filter_num;
        
        bool feature_enabled;
        this->get_parameter("feature_extract_enable", feature_enabled);
        p_pre->feature_enabled = feature_enabled;
        
        this->get_parameter("mapping/extrinsic_est_en", extrinsic_est_en);
        this->get_parameter("pcd_save/pcd_save_en", pcd_save_en);
        this->get_parameter("mapping/extrinsic_T", extrinT);
        this->get_parameter("mapping/extrinsic_R", extrinR);
        this->get_parameter("mapping/init_pos", init_pos);
        this->get_parameter("mapping/init_rot", init_rot);
        this->get_parameter("globalmap_dir", globalmap_dir);
        this->get_parameter("relocal_publish/relocal_odom_en", relocal_odom_en);
        this->get_parameter("relocal_publish/relocal_scan_en", relocal_scan_en);
        this->get_parameter("relocal_publish/localmap_en", localmap_en);
        this->get_parameter("ininit_resolution_dis", ininit_resolution_dis);
        this->get_parameter("ininit_resolution_rot", ininit_resolution_rot);
        
        // 创建发布者
        pubLaserCloudFull_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 100000);
        pubLaserCloudFull_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered_body", 100000);
        pubLaserCloudEffect_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 100000);
        pubLaserCloudMap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 100000);
        pubOdomAftMapped_ = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry", 100000);
        pubPath_ = this->create_publisher<nav_msgs::msg::Path>("/path", 100000);
        
        // relocalization
        subIniPoseFromRviz_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 8, std::bind(&FastLioNode::initialpose_callback, this, std::placeholders::_1));
        subKeyBoard_ = this->create_subscription<std_msgs::msg::String>(
            "/keyboard_msgs", 8, std::bind(&FastLioNode::keyboard_callback, this, std::placeholders::_1));
        
        pubGobalMap_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/gobal_map_relocal", 10);
        pubBeforeRegistCloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/beforeRegisted", rclcpp::QoS(10).transient_local());
        pubRegistCloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/registed_current_scan", rclcpp::QoS(10).transient_local());
        pub_local_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/local_map_relocal", rclcpp::QoS(10).transient_local());
        pubOdomAft_reg_ = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry_relocal", 100000);
        initguess_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/initguess", 10);
        
        // 创建订阅者
        if (p_pre->lidar_type == AVIA) {
            sub_pcl_livox_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                lid_topic, 200000, std::bind(&FastLioNode::livox_pcl_cbk, this, std::placeholders::_1));
        } else {
            sub_pcl_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                lid_topic, 200000, std::bind(&FastLioNode::standard_pcl_cbk, this, std::placeholders::_1));
        }
        
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 200000, std::bind(&FastLioNode::imu_cbk, this, std::placeholders::_1));
        
        // 初始化path
        path.header.stamp = this->now();
        path.header.frame_id = "camera_init";
        
        downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
        downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
        
        if (extrinT.size() < 3) extrinT = {0.04165, 0.02326, -0.0284};
        if (extrinR.size() < 9) extrinR = {1,0,0,0,1,0,0,0,1};
        Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
        Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
        p_imu1_ = std::make_shared<ImuProcess>();
        p_imu1_->set_param(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU, V3D(gyr_cov, gyr_cov, gyr_cov), V3D(acc_cov, acc_cov, acc_cov),
                          V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov), V3D(b_acc_cov, b_acc_cov, b_acc_cov));
        
        // 地图保存服务
        map_save_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/map_save", std::bind(&FastLioNode::save_map_callback, this, std::placeholders::_1, std::placeholders::_2));
        
        signal(SIGINT, SigHandle);
    }

    void init()
    {
        // 初始化matching (需要在构造函数外调用，因为需要shared_from_this)
        matching_ptr_ = std::make_shared<Matching>(shared_from_this());
        
        std::cout << "\033[1;32m========================================\033[0m" << std::endl;
        std::cout << "\033[1;32m  MID360 SLAM 系统已启动\033[0m" << std::endl;
        std::cout << "\033[1;32m  建图模式: 自动初始化\033[0m" << std::endl;
        std::cout << "\033[1;32m  重定位模式: 在RVIZ中点击 '2D Pose Estimate'\033[0m" << std::endl;
        std::cout << "\033[1;32m========================================\033[0m" << std::endl;
        
        // 启动全局地图定时发布 (2秒后首次，之后每1秒一次)
        map_publish_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&FastLioNode::publish_global_map_timer, this));
        
        // 主循环定时器 (200μs)
        timer_ = this->create_wall_timer(std::chrono::microseconds(200), std::bind(&FastLioNode::main_loop, this));
    }

    void publish_global_map_timer()
    {
        if (!matching_ptr_) return;
        
        pcl::PointCloud<pcl::PointXYZI>::Ptr global_map_ptr(new pcl::PointCloud<pcl::PointXYZI>());
        matching_ptr_->GetGlobalMap(global_map_ptr);
        if (global_map_ptr && !global_map_ptr->empty()) {
            sensor_msgs::msg::PointCloud2 global_map_msg;
            pcl::toROSMsg(*global_map_ptr, global_map_msg);
            global_map_msg.header.stamp = this->now();
            global_map_msg.header.frame_id = "camera_init";
            pubGobalMap_->publish(global_map_msg);
            std::cout << "\033[1;32m[地图] 已发布 /gobal_map_relocal: \033[0m"
                      << global_map_ptr->points.size() << " 点, frame=camera_init" << std::endl;
        }
        
        // 首次发布后，切换为每1秒发布
        map_publish_timer_->cancel();
        map_publish_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&FastLioNode::publish_global_map_timer, this));
    }

private:
    static void SigHandle(int sig)
    {
        flg_exit = true;
        RCLCPP_WARN(rclcpp::get_logger("laserMapping"), "catch sig %d", sig);
        sig_buffer.notify_all();
    }

    void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        mtx_buffer.lock();
        scan_count++;
        static int cbk_count = 0;
        cbk_count++;
        if (cbk_count % 100 == 1) {
            std::cout << "[LiDAR] 回调 #" << cbk_count 
                      << " | 点数=" << msg->width * msg->height
                      << " | 缓冲=" << lidar_buffer.size() << std::endl;
        }
        double preprocess_start_time = omp_get_wtime();
        if (rclcpp::Time(msg->header.stamp).seconds() < last_timestamp_lidar)
        {
            RCLCPP_ERROR(this->get_logger(), "lidar loop back, clear buffer");
            lidar_buffer.clear();
        }

        PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
        p_pre->process(msg, ptr);
        lidar_buffer.push_back(ptr);
        time_buffer.push_back(rclcpp::Time(msg->header.stamp).seconds());
        last_timestamp_lidar = rclcpp::Time(msg->header.stamp).seconds();
        mtx_buffer.unlock();
        sig_buffer.notify_all();
    }

    void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        mtx_buffer.lock();
        double preprocess_start_time = omp_get_wtime();
        scan_count++;
        static int cbk_count = 0;
        cbk_count++;
        if (cbk_count % 100 == 1) {
            std::cout << "[LiDAR] 回调 #" << cbk_count << " | 点数=" << msg->points.size() 
                      << " | 缓冲区=" << lidar_buffer.size() << std::endl;
        }
        if (rclcpp::Time(msg->header.stamp).seconds() < last_timestamp_lidar)
        {
            RCLCPP_ERROR(this->get_logger(), "lidar loop back, clear buffer");
            lidar_buffer.clear();
        }
        last_timestamp_lidar = rclcpp::Time(msg->header.stamp).seconds();

        if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty())
        {
            printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n", last_timestamp_imu, last_timestamp_lidar);
        }

        if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
        {
            timediff_set_flg = true;
            timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
            printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
        }

        PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
        p_pre->process(msg, ptr);
        lidar_buffer.push_back(ptr);
        time_buffer.push_back(last_timestamp_lidar);

        mtx_buffer.unlock();
        sig_buffer.notify_all();
    }

    void imu_cbk(const sensor_msgs::msg::Imu::SharedPtr msg_in)
    {
        publish_count++;
        sensor_msgs::msg::Imu::SharedPtr msg = std::make_shared<sensor_msgs::msg::Imu>(*msg_in);

        if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
        {
            msg->header.stamp = rclcpp::Time(static_cast<int64_t>((timediff_lidar_wrt_imu + rclcpp::Time(msg_in->header.stamp).seconds()) * 1e9));
        }

        msg->header.stamp = rclcpp::Time(static_cast<int64_t>((rclcpp::Time(msg_in->header.stamp).seconds() - time_diff_lidar_to_imu) * 1e9));

        double timestamp = rclcpp::Time(msg->header.stamp).seconds();

        mtx_buffer.lock();

        (void)timestamp;
        timestamp = rclcpp::Clock().now().seconds();
        msg->header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp * 1e9));

        if (timestamp < last_timestamp_imu)
        {
            RCLCPP_WARN(this->get_logger(), "imu loop back, clear buffer");
            imu_buffer.clear();
        }

        last_timestamp_imu = timestamp;

        imu_buffer.push_back(msg);
        mtx_buffer.unlock();
        sig_buffer.notify_all();
    }

    bool sync_packages(MeasureGroup &meas)
    {
        if (lidar_buffer.empty() || imu_buffer.empty())
        {
            return false;
        }

        if (!lidar_pushed)
        {
            meas.lidar = lidar_buffer.front();
            meas.lidar_beg_time = time_buffer.front();
            if (meas.lidar->points.size() <= 5)
            {
                lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
                RCLCPP_WARN(this->get_logger(), "Too few input point cloud!");
            }
            else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
            {
                lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            }
            else
            {
                scan_num++;
                lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
                lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
            }

            meas.lidar_end_time = lidar_end_time;

            lidar_pushed = true;
        }

        // 只要有IMU数据在缓冲区就继续（Livox IMU时间戳可能不准确）
        if (imu_buffer.empty())
        {
            return false;
        }

        double imu_time = rclcpp::Time(imu_buffer.front()->header.stamp).seconds();
        meas.imu.clear();
        while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
        {
            imu_time = rclcpp::Time(imu_buffer.front()->header.stamp).seconds();
            if (imu_time > lidar_end_time)
                break;
            meas.imu.push_back(imu_buffer.front());
            imu_buffer.pop_front();
        }

        lidar_buffer.pop_front();
        time_buffer.pop_front();
        lidar_pushed = false;
        return true;
    }

    void pointBodyToWorld(PointType const *const pi, PointType *const po)
    {
        V3D p_body(pi->x, pi->y, pi->z);
        V3D p_global(state_point.rot.matrix() * (state_point.offset_R_L_I.matrix() * p_body + state_point.offset_T_L_I) + state_point.pos);

        po->x = p_global(0);
        po->y = p_global(1);
        po->z = p_global(2);
        po->intensity = pi->intensity;
    }

    template <typename T>
    void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
    {
        V3D p_body(pi[0], pi[1], pi[2]);
        V3D p_global(state_point.rot.matrix() * (state_point.offset_R_L_I.matrix() * p_body + state_point.offset_T_L_I) + state_point.pos);

        po[0] = p_global(0);
        po[1] = p_global(1);
        po[2] = p_global(2);
    }

    void lasermap_fov_segment()
    {
        cub_needrm.clear();
        kdtree_delete_counter = 0;

        V3D pos_LiD = pos_lid;
        if (!Localmap_Initialized)
        {
            for (int i = 0; i < 3; i++)
            {
                LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
                LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
            }
            Localmap_Initialized = true;
            return;
        }

        float dist_to_map_edge[3][2];
        bool need_move = false;
        for (int i = 0; i < 3; i++)
        {
            dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
            dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
            if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
                need_move = true;
        }
        if (!need_move)
            return;

        BoxPointType New_LocalMap_Points, tmp_boxpoints;
        New_LocalMap_Points = LocalMap_Points;
        float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD - 1)));
        for (int i = 0; i < 3; i++)
        {
            tmp_boxpoints = LocalMap_Points;
            if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE)
            {
                New_LocalMap_Points.vertex_max[i] -= mov_dist;
                New_LocalMap_Points.vertex_min[i] -= mov_dist;
                tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
                cub_needrm.push_back(tmp_boxpoints);
            }
            else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
            {
                New_LocalMap_Points.vertex_max[i] += mov_dist;
                New_LocalMap_Points.vertex_min[i] += mov_dist;
                tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
                cub_needrm.push_back(tmp_boxpoints);
            }
        }
        LocalMap_Points = New_LocalMap_Points;

        PointVector points_history;
        ikdtree.acquire_removed_points(points_history);

        if (cub_needrm.size() > 0)
            kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    }

    void RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po)
    {
        V3D p_body_lidar(pi->x, pi->y, pi->z);
        V3D p_body_imu(state_point.offset_R_L_I.matrix() * p_body_lidar + state_point.offset_T_L_I);

        po->x = p_body_imu(0);
        po->y = p_body_imu(1);
        po->z = p_body_imu(2);
        po->intensity = pi->intensity;
    }

    void init_ikdtree()
    {
        std::string all_points_dir(std::string(std::string(ROOT_DIR) + "PCD/") + "GlobalMap_ikdtree.pcd");
        if (pcl::io::loadPCDFile<PointType>(all_points_dir, *cloud) == -1)
        {
            PCL_ERROR("Read file fail!\n");
        }

        ikdtree.set_downsample_param(filter_size_map_min);
        ikdtree.Build(cloud->points);
        std::cout << "---- ikdtree size: " << ikdtree.size() << std::endl;
    }

    void publish_frame_world(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFull_)
    {
        if (scan_pub_en)
        {
            PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
            int size = laserCloudFullRes->points.size();
            PointCloudXYZI::Ptr laserCloudWorld(
                new PointCloudXYZI(size, 1));

            for (int i = 0; i < size; i++)
            {
                pointBodyToWorld(&laserCloudFullRes->points[i],
                                 &laserCloudWorld->points[i]);
            }

            sensor_msgs::msg::PointCloud2 laserCloudmsg;
            pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
            laserCloudmsg.header.stamp = rclcpp::Time(static_cast<int64_t>(lidar_end_time * 1e9));
            laserCloudmsg.header.frame_id = "camera_init";
            pubLaserCloudFull_->publish(laserCloudmsg);
            publish_count -= PUBFRAME_PERIOD;
        }

        {
            int size = feats_undistort->points.size();
            PointCloudXYZI::Ptr laserCloudWorld(
                new PointCloudXYZI(size, 1));

            for (int i = 0; i < size; i++)
            {
                pointBodyToWorld(&feats_undistort->points[i],
                                 &laserCloudWorld->points[i]);
            }

            static int scan_wait_num = 0;
            scan_wait_num++;

            if (scan_wait_num % 4 == 0)
                *pcl_wait_save += *laserCloudWorld;
        }
    }

    void publish_frame_body(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFull_body)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyLidarToIMU(&feats_undistort->points[i],
                                   &laserCloudIMUBody->points[i]);
        }

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
        laserCloudmsg.header.stamp = rclcpp::Time(static_cast<int64_t>(lidar_end_time * 1e9));
        laserCloudmsg.header.frame_id = "body";
        pubLaserCloudFull_body->publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    void publish_map(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudMap)
    {
        sensor_msgs::msg::PointCloud2 laserCloudMap;
        pcl::toROSMsg(*featsFromMap, laserCloudMap);
        laserCloudMap.header.stamp = rclcpp::Time(static_cast<int64_t>(lidar_end_time * 1e9));
        laserCloudMap.header.frame_id = "camera_init";
        pubLaserCloudMap->publish(laserCloudMap);
    }

    template <typename T>
    void set_posestamp(T &out)
    {
        out.pose.position.x = pos_lid_filtered(0);
        out.pose.position.y = pos_lid_filtered(1);
        out.pose.position.z = pos_lid_filtered(2);

        auto q_ = Eigen::Quaterniond(state_point.rot.matrix());
        out.pose.orientation.x = q_.coeffs()[0];
        out.pose.orientation.y = q_.coeffs()[1];
        out.pose.orientation.z = q_.coeffs()[2];
        out.pose.orientation.w = q_.coeffs()[3];
    }

    void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pubOdomAftMapped)
    {
        odomAftMapped.header.frame_id = "camera_init";
        odomAftMapped.child_frame_id = "body";
        odomAftMapped.header.stamp = rclcpp::Time(static_cast<int64_t>(lidar_end_time * 1e9));
        set_posestamp(odomAftMapped.pose);
        pubOdomAftMapped->publish(odomAftMapped);

        auto P = kf.get_P();
        for (int i = 0; i < 6; i++)
        {
            int k = i < 3 ? i + 3 : i - 3;
            odomAftMapped.pose.covariance[i * 6 + 0] = P(k, 3);
            odomAftMapped.pose.covariance[i * 6 + 1] = P(k, 4);
            odomAftMapped.pose.covariance[i * 6 + 2] = P(k, 5);
            odomAftMapped.pose.covariance[i * 6 + 3] = P(k, 0);
            odomAftMapped.pose.covariance[i * 6 + 4] = P(k, 1);
            odomAftMapped.pose.covariance[i * 6 + 5] = P(k, 2);
        }

        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = rclcpp::Time(static_cast<int64_t>(lidar_end_time * 1e9));
        transformStamped.header.frame_id = "camera_init";
        transformStamped.child_frame_id = "body";
        transformStamped.transform.translation.x = odomAftMapped.pose.pose.position.x;
        transformStamped.transform.translation.y = odomAftMapped.pose.pose.position.y;
        transformStamped.transform.translation.z = odomAftMapped.pose.pose.position.z;
        transformStamped.transform.rotation = odomAftMapped.pose.pose.orientation;
        tf_broadcaster_->sendTransform(transformStamped);
    }

    void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &pubPath)
    {
        set_posestamp(msg_body_pose);
        msg_body_pose.header.stamp = rclcpp::Time(static_cast<int64_t>(lidar_end_time * 1e9));
        msg_body_pose.header.frame_id = "camera_init";

        static int jjj = 0;
        jjj++;
        if (jjj % 10 == 0)
        {
            path.poses.push_back(msg_body_pose);
            pubPath->publish(path);
        }
    }

    sensor_msgs::msg::PointCloud2 publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &thisPub, pcl::PointCloud<pcl::PointXYZI>::Ptr thisCloud, rclcpp::Time thisStamp, std::string thisFrame)
    {
        sensor_msgs::msg::PointCloud2 tempCloud;
        pcl::toROSMsg(*thisCloud, tempCloud);
        tempCloud.header.stamp = thisStamp;
        tempCloud.header.frame_id = thisFrame;
        if (thisPub->get_subscription_count() != 0)
            thisPub->publish(tempCloud);
        return tempCloud;
    }

    void initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_msg){
        if(init_flag == InitializedFlag::Initialized)
            return;
        auto now = this->now();
        pose_received = true;
        if((now - last_pose_time_).seconds() < 2.0)
            return;
        last_pose_time_ = now;
        const auto& pose = pose_msg->pose.pose;
        Eigen::Quaterniond quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        initial_guess.block<3,3>(0,0) = quaternion.matrix();
        initial_guess(0,3) = pose.position.x;
        initial_guess(1,3) = pose.position.y;
        initial_guess(2,3) = pose.position.z;

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "camera_init";
        marker.header.stamp = this->now();
        marker.ns = "arrow_marker";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose = pose_msg->pose.pose;
        marker.scale.x = 3.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        initguess_pub_->publish(marker);

        init_flag = InitializedFlag::NonInitialized;
        pcl::PointCloud<pcl::PointXYZI>::Ptr after_guess(new pcl::PointCloud<pcl::PointXYZI>(first_init_scan->size(),1));
        pcl::transformPointCloud(*first_init_scan,*after_guess,initial_guess);
        publishCloud(pubBeforeRegistCloud_, after_guess, this->now(), "camera_init");
    }

    void set_IMU_guess(){
        IMU_guess.block<3,3>(0,0) = state_point.rot.matrix();
        IMU_guess(0,3) = state_point.pos(0);
        IMU_guess(1,3) = state_point.pos(1);
        IMU_guess(2,3) = state_point.pos(2);
    }

    void publishRegOdom(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pubOdomAft_reg){
        odomAft_reg.header.frame_id = "camera_init";
        odomAft_reg.child_frame_id = "body";
        odomAft_reg.header.stamp = rclcpp::Time(static_cast<int64_t>(lidar_end_time * 1e9));
        Eigen::Quaterniond quaternion(after_registe.block<3,3>(0,0));
        odomAft_reg.pose.pose.orientation.w = quaternion.w();
        odomAft_reg.pose.pose.orientation.x = quaternion.x();
        odomAft_reg.pose.pose.orientation.y = quaternion.y();
        odomAft_reg.pose.pose.orientation.z = quaternion.z();
        odomAft_reg.pose.pose.position.x = after_registe(0,3);
        odomAft_reg.pose.pose.position.y = after_registe(1,3);
        odomAft_reg.pose.pose.position.z = after_registe(2,3);
        pubOdomAft_reg->publish(odomAft_reg);
    }

    void keyboard_callback(const std_msgs::msg::String::SharedPtr keyboard_input){
        if(keyboard_input->data == "Moving forward") {
            initial_guess(0,3) += ininit_resolution_dis;
        }else if(keyboard_input->data == "Moving backward"){
            initial_guess(0,3) -= ininit_resolution_dis;
        }else if(keyboard_input->data == "Moving left"){
            initial_guess(1,3) += ininit_resolution_dis;
        }else if(keyboard_input->data == "Moving right"){
            initial_guess(1,3) -= ininit_resolution_dis;
        }else if(keyboard_input->data == "rotate left"){
            Eigen::Matrix3d rotation_matrix = Eigen::AngleAxisd(ininit_resolution_rot*(M_PI / 180), Eigen::Vector3d::UnitZ()).toRotationMatrix();
            initial_guess.block<3,3>(0,0) = initial_guess.block<3,3>(0,0) * rotation_matrix;
        }else if(keyboard_input->data == "rotate right"){
            Eigen::Matrix3d rotation_matrix = Eigen::AngleAxisd(-ininit_resolution_rot*(M_PI / 180), Eigen::Vector3d::UnitZ()).toRotationMatrix();
            initial_guess.block<3,3>(0,0) = initial_guess.block<3,3>(0,0) * rotation_matrix;
        }else if(keyboard_input->data == "inc rot resolu"){
            ininit_resolution_rot += 1;
            std::cout << "rot resolu: " << ininit_resolution_rot;
        }else if(keyboard_input->data == "red rot resolu"){
            ininit_resolution_rot -= 1;
            std::cout << "rot resolu: " << ininit_resolution_rot;
        }else if(keyboard_input->data == "inc dis resolu"){
            ininit_resolution_dis += 0.02;
            std::cout << "dis resolu: " << ininit_resolution_dis;
        }else if(keyboard_input->data == "red dis resolu"){
            ininit_resolution_dis -= 0.02;
            std::cout << "dis resolu: " << ininit_resolution_dis;
        }else if(keyboard_input->data == "finish, start reg"){
            init_flag = InitializedFlag::NonInitialized;
        }else{
            std::cout << "无效输入" << std::endl;
            return;
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr after_guess(new pcl::PointCloud<pcl::PointXYZI>(first_init_scan->size(),1));
        pcl::transformPointCloud(*first_init_scan,*after_guess,initial_guess);
        publishCloud(pubBeforeRegistCloud_, after_guess, this->now(), "camera_init");
    }

    void main_loop()
    {
        static int marker_count = 0;
        marker_count++;

        // 每1秒输出系统状态
        if(marker_count % 5000 == 0){
            if(!inited_pose){
                if(!pose_received){
                    std::cout << "\033[1;33m[状态] 等待雷达数据初始化... (接收=" << scan_count << "帧)\033[0m" << std::endl;
                }
            }
        }
        
        if (flg_exit) {
            rclcpp::shutdown();
            return;
        }

        if (sync_packages(Measures))
        {
            double t00 = omp_get_wtime();

            if(!inited_pose){
                if(pose_received){
                    // 用户通过2D Pose Estimate给出了初始位置 → 使用NDT重定位
                    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
                    int size = Measures.lidar->points.size();
                    PointCloudXYZI::Ptr scan_IMUframe(new PointCloudXYZI(size,1));
                    for (int i = 0; i < size; i++)
                    {
                        RGBpointBodyLidarToIMU(&Measures.lidar->points[i],
                                            &scan_IMUframe->points[i]);
                    }
                    pcl::copyPointCloud(*scan_IMUframe,*first_init_scan);
                    if(matching_ptr_->InitFromGuess(initial_guess,scan_IMUframe,result) != true){
                        std::cout << "initialization false , register score:" << matching_ptr_->GetFitnessScore() << std::endl;
                        publishCloud(pubRegistCloud_, matching_ptr_->GetCurrentScan(), this->now(), "camera_init");
                        init_flag = InitializedFlag::Initializing;
                        std::cout << "Offer A New Guess Please " << std::endl;
                        return;
                    }
                    double rx = result(0,3);
                    double ry = result(1,3);
                    double rz = result(2,3);
                    if(rx < -60 || rx > 15 || ry < -15 || ry > 50 || rz < -5 || rz > 25){
                        std::cout << "initialization result out of map range (" << rx << "," << ry << "," << rz << ")" << std::endl;
                        init_flag = InitializedFlag::Initializing;
                        std::cout << "Offer A New Guess Please " << std::endl;
                        return;
                    }
                    matching_ptr_->SetInitPose(result);
                    matching_ptr_->SetInited();
                    state_point = kf.get_x();
                    state_point.pos[0] = result(0,3);
                    state_point.pos[1] = result(1,3);
                    state_point.pos[2] = result(2,3);
                    Eigen::Matrix3d init_rot = result.block<3,3>(0,0);
                    Eigen::JacobiSVD<Eigen::Matrix3d> svd(init_rot, Eigen::ComputeFullU | Eigen::ComputeFullV);
                    init_rot = svd.matrixU() * svd.matrixV().transpose();
                    if(init_rot.determinant() < 0) init_rot = -init_rot;
                    state_point.rot = Sophus::SO3<double>(init_rot);
                    state_point.vel = Eigen::Vector3d::Zero();
                    kf.change_x(state_point);
                    pos_lid_filtered = state_point.pos + state_point.rot.matrix() * state_point.offset_T_L_I;
                    pos_lid_filter_inited = true;
                    inited_pose = true;
                    clean_buff = true;
                    std::cout << "initialization success , register score:" << matching_ptr_->GetFitnessScore() << std::endl;
                } else {
                    // 建图模式: 自动以原点初始化
                    std::cout << "\033[1;32m[建图模式] 自动初始化于原点 (0,0,0)\033[0m" << std::endl;
                    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
                    matching_ptr_->SetInitPose(result);
                    matching_ptr_->SetInited();
                    state_point = kf.get_x();
                    state_point.pos[0] = 0.0;
                    state_point.pos[1] = 0.0;
                    state_point.pos[2] = 0.0;
                    state_point.rot = Sophus::SO3<double>(Eigen::Matrix3d::Identity());
                    state_point.vel = Eigen::Vector3d::Zero();
                    kf.change_x(state_point);
                    pos_lid_filtered = state_point.pos + state_point.rot.matrix() * state_point.offset_T_L_I;
                    pos_lid_filter_inited = true;
                    inited_pose = true;
                    clean_buff = true;
                    std::cout << "\033[1;32m[建图模式] 初始化完成, 开始建图\033[0m" << std::endl;
                }
            }
            if(clean_buff){
                clean_buff = false;
                lidar_buffer.clear();
                imu_buffer.clear();
                time_buffer.clear();
                return;
            }
            if(clean_buff){
                clean_buff = false;
                lidar_buffer.clear();
                imu_buffer.clear();
                time_buffer.clear();
                return;
            }

            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu1_->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                return;
            }

            // 直接使用原始点云（跳过IMU处理，Livox IMU数据不可靠）
            pcl::copyPointCloud(*Measures.lidar, *feats_undistort);

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!");
                return;
            }

            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot.matrix() * state_point.offset_T_L_I;

            if (!pos_lid_filter_inited) {
                pos_lid_filtered = pos_lid;
                pos_lid_filter_inited = true;
            } else {
                pos_lid_filtered = ema_alpha * pos_lid + (1.0 - ema_alpha) * pos_lid_filtered;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;

            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            feats_down_size = feats_down_body->points.size();

            if (feats_down_size < 5)
            {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!");
                return;
            }

            if(matching_ptr_->HasInited()){
                int size = feats_undistort->points.size();
                PointCloudXYZI::Ptr scan_IMUframe(new PointCloudXYZI(size,1));
                for (int i = 0; i < size; i++)
                {
                    RGBpointBodyLidarToIMU(&feats_undistort->points[i],
                                        &scan_IMUframe->points[i]);
                }
                set_IMU_guess();
                matching_ptr_->Registe_2_globalmap(scan_IMUframe,IMU_guess,after_registe);
                kf.update_with_reg(LASER_POINT_COV,after_registe);

                if(relocal_odom_en){
                    publishRegOdom(pubOdomAft_reg_);
                }
                if(relocal_scan_en){
                    publishCloud(pubRegistCloud_, matching_ptr_->GetCurrentScan(), rclcpp::Time(static_cast<int64_t>(lidar_end_time * 1e9)), "camera_init");
                }
                if(localmap_en && matching_ptr_->HasNewLocalMap()){
                    publishCloud(pub_local_map_, matching_ptr_->GetLocalMap(), rclcpp::Time(static_cast<int64_t>(lidar_end_time * 1e9)), "camera_init");
                }
            }

            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot.matrix() * state_point.offset_T_L_I;
            pos_lid_filtered = ema_alpha * pos_lid + (1.0 - ema_alpha) * pos_lid_filtered;

            publish_odometry(pubOdomAftMapped_);

            feats_down_world->resize(feats_down_size);

            if (path_en)
                publish_path(pubPath_);
            if (scan_pub_en || pcd_save_en)
                publish_frame_world(pubLaserCloudFull_);
            if (scan_pub_en && scan_body_pub_en)
                publish_frame_body(pubLaserCloudFull_body_);

            double t11 = omp_get_wtime();
            std::cout << "feats_down_size: " << feats_down_size << "  Whole mapping time(ms):  " << (t11 - t00) * 1000 << std::endl
                      << std::endl;

            // 输出当前位姿
            if(pose_received && matching_ptr_->HasInited()){
                std::cout << "========== 重定位已初始化 ==========" << std::endl;
            } else if(!pose_received){
                std::cout << "========== 建图模式 ==========" << std::endl;
            } else {
                std::cout << "========== 等待重定位 (请在RVIZ中点击 '2D Pose Estimate') ==========" << std::endl;
            }
            std::cout << "  X: " << std::fixed << std::setprecision(3) << pos_lid_filtered(0)
                      << "  Y: " << pos_lid_filtered(1)
                      << "  Z: " << pos_lid_filtered(2) << std::endl;
            std::cout << "======================================" << std::endl;
        }
    }

    void save_map_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        (void)req;
        map_file_path = this->get_parameter("map_file_path").as_string();
        if (map_file_path.empty()) {
            map_file_path = "/home/b1/mid360_map/map.pcd";
        }
        mkdir("/home/b1/mid360_map", 0755);
        if (pcl_wait_save->points.empty()) {
            std::cout << "[地图保存] 无数据可保存" << std::endl;
            res->success = false;
            res->message = "No points to save";
            return;
        }
        pcl::io::savePCDFileBinary(map_file_path, *pcl_wait_save);
        std::cout << "[地图保存] 已保存: " << map_file_path 
                  << " (" << pcl_wait_save->points.size() << " 点)" << std::endl;
        res->success = true;
        res->message = "Saved to " + map_file_path;
    }

    // 成员变量
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr map_publish_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_srv_;
    
    // 发布者
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
    
    // relocalization发布者
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubGobalMap_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubBeforeRegistCloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRegistCloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_local_map_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAft_reg_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr initguess_pub_;
    
    // 订阅者
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subIniPoseFromRviz_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subKeyBoard_;
    
    // 其他变量
    std::shared_ptr<Matching> matching_ptr_;
    std::shared_ptr<ImuProcess> p_imu1_;
    
    enum InitializedFlag
    {
        NonInitialized = 0,
        Initializing = 1,
        Initialized = 2
    };
    InitializedFlag init_flag = InitializedFlag::NonInitialized;
    
    bool inited_pose = false;
    bool clean_buff = true;
    bool pose_received = false;
    bool relocal_odom_en = false;
    bool relocal_scan_en = false;
    bool localmap_en = false;
    
    Eigen::Matrix4d initial_guess = Eigen::Matrix4d::Identity();
    pcl::PointCloud<pcl::PointXYZI>::Ptr first_init_scan{new pcl::PointCloud<pcl::PointXYZI>()};
    
    Eigen::Matrix4d IMU_guess = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d after_registe = Eigen::Matrix4d::Identity();
    nav_msgs::msg::Odometry odomAft_reg;
    
    double ininit_resolution_dis = 0.05;
    double ininit_resolution_rot = 1;
    rclcpp::Time last_pose_time_{0, 0, RCL_ROS_TIME};
    
    double lidar_mean_scantime = 0.0;
    int scan_num = 0;
    
    bool timediff_set_flg = false;
    double timediff_lidar_wrt_imu = 0.0;
    
    BoxPointType LocalMap_Points;
    bool Localmap_Initialized = false;
    
    PointCloudXYZI::Ptr pcl_wait_pub{new PointCloudXYZI(500000, 1)};
    PointCloudXYZI::Ptr pcl_wait_save{new PointCloudXYZI()};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FastLioNode>();
    node->init();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
