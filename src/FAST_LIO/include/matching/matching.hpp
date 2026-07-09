#ifndef FAST_LIO_MATCHING_HPP_
#define FAST_LIO_MATCHING_HPP_
#include <rclcpp/rclcpp.hpp>

#include <deque>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <cmath>
#include <csignal>
#include <fenv.h>

// NDT内部可能触发浮点异常，用fedisableexcept屏蔽，让NDT安全产生NaN
// (不能用siglongjmp，会破坏PCL堆栈导致stack smashing)
extern "C" void ndt_fpe_handler(int) {
    fedisableexcept(FE_ALL_EXCEPT);
}

//ndt匹配
class NDTRegistration{
  public:
    NDTRegistration(float res, float step_size, float trans_eps, int max_iter);

    bool SetInputTarget(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_target) ;
    bool ScanMatch(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_source, 
                   const Eigen::Matrix4d& predict_pose, 
                   pcl::PointCloud<pcl::PointXYZI>::Ptr& result_cloud_ptr,
                   Eigen::Matrix4d& result_pose) ;
    float GetFitnessScore() ;
  
  private:
    bool SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter);

  private:
    pcl::NormalDistributionsTransform<pcl::PointXYZI,pcl::PointXYZI>::Ptr ndt_ptr_;
};

NDTRegistration::NDTRegistration(float res, float step_size, float trans_eps, int max_iter)
    :ndt_ptr_(new pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>()) {
    SetRegistrationParam(res, step_size, trans_eps, max_iter);
}
bool NDTRegistration::SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter) {
    ndt_ptr_->setResolution(res);
    ndt_ptr_->setStepSize(step_size);
    ndt_ptr_->setTransformationEpsilon(trans_eps);
    ndt_ptr_->setMaximumIterations(max_iter);

    std::cout << "NDT params:" << std::endl
              << "res: " << res << ", "
              << "step_size: " << step_size << ", "
              << "trans_eps: " << trans_eps << ", "
              << "max_iter: " << max_iter 
              << std::endl << std::endl;

    return true;
}
bool NDTRegistration::SetInputTarget(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_target) {
    if (!input_target || input_target->points.size() < 5) {
        std::cout << "[NDT] SetInputTarget: 点数不足" << std::endl;
        return false;
    }
    fedisableexcept(FE_ALL_EXCEPT);
    signal(SIGFPE, ndt_fpe_handler);
    ndt_ptr_->setInputTarget(input_target);
    signal(SIGFPE, SIG_DFL);
    return true;
}

bool NDTRegistration::ScanMatch(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_source, 
                                const Eigen::Matrix4d& predict_pose, 
                                pcl::PointCloud<pcl::PointXYZI>::Ptr& result_cloud_ptr,
                                Eigen::Matrix4d& result_pose) {
    ndt_ptr_->setInputSource(input_source);
    
    fedisableexcept(FE_ALL_EXCEPT);
    signal(SIGFPE, ndt_fpe_handler);
    ndt_ptr_->align(*result_cloud_ptr, predict_pose.cast<float>());
    result_pose = ndt_ptr_->getFinalTransformation().cast<double>();
    signal(SIGFPE, SIG_DFL);
    
    return true;
}

float NDTRegistration::GetFitnessScore() {
    return ndt_ptr_->getFitnessScore();
}

//点云滤波的接口，通过多态选择滤波器
class CloudFilterInterface {
  public:
    virtual ~CloudFilterInterface() = default;

    virtual bool Filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr& filtered_cloud_ptr) = 0;
};

//不需要点云滤波
class NoFilter: public CloudFilterInterface {
  public:
    NoFilter();

    bool Filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr& filtered_cloud_ptr) override;

};
NoFilter::NoFilter() {}

bool NoFilter::Filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud_ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr& filtered_cloud_ptr) {
    filtered_cloud_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>(*input_cloud_ptr));
    return true;
}

//体素滤波
class VoxelFilter: public CloudFilterInterface {
  public:
    VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z);

    bool Filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr& filtered_cloud_ptr) override;

  private:
    bool SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z);

  private:
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter_;
};
VoxelFilter::VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z) {
    SetFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
}

bool VoxelFilter::SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z) {
    voxel_filter_.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);

    std::cout << "Voxel Filter params:" << std::endl
            << leaf_size_x << ", "
            << leaf_size_y << ", "
            << leaf_size_z 
            << std::endl << std::endl;

    return true;
}
bool VoxelFilter::Filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr& filtered_cloud_ptr) {
    voxel_filter_.setInputCloud(input_cloud_ptr);
    voxel_filter_.filter(*filtered_cloud_ptr);

    return true;
}

//长方体滤波，用于生成（匹配的局部地图）
class BoxFilter: public CloudFilterInterface {
  public:
    BoxFilter(std::vector<float> input_par);
    BoxFilter() = default;

    bool Filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr& filtered_cloud_ptr) override;

    void SetSize(std::vector<float> size);
    void SetOrigin(std::vector<float> origin);
    std::vector<float> GetEdge();

  private:
    void CalculateEdge();

  private:
    pcl::CropBox<pcl::PointXYZI> pcl_box_filter_;

    std::vector<float> origin_;
    std::vector<float> size_;
    std::vector<float> edge_;
};

BoxFilter::BoxFilter(std::vector<float> input_par) {
    size_.resize(6);
    edge_.resize(6);
    origin_.resize(3);

    for (size_t i = 0; i < size_.size(); i++) {
        size_.at(i) = input_par.at(i);
    }
    SetSize(size_);
}

bool BoxFilter::Filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud_ptr,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr& output_cloud_ptr) {
    output_cloud_ptr->clear();
    pcl_box_filter_.setMin(Eigen::Vector4f(edge_.at(0), edge_.at(2), edge_.at(4), 1.0e-6));
    pcl_box_filter_.setMax(Eigen::Vector4f(edge_.at(1), edge_.at(3), edge_.at(5), 1.0e6));
    pcl_box_filter_.setInputCloud(input_cloud_ptr);
    pcl_box_filter_.filter(*output_cloud_ptr);

    return true;
}

void BoxFilter::SetSize(std::vector<float> size) {
    size_ = size;
    std::cout << "Box Filter params:" << std::endl
              << "min_x: " << size.at(0) << ", "
              << "max_x: " << size.at(1) << ", "
              << "min_y: " << size.at(2) << ", "
              << "max_y: " << size.at(3) << ", "
              << "min_z: " << size.at(4) << ", "
              << "max_z: " << size.at(5)
              << std::endl << std::endl;
    
    CalculateEdge();
}

void BoxFilter::SetOrigin(std::vector<float> origin) {
    origin_ = origin;
    CalculateEdge();
}

void BoxFilter::CalculateEdge() {
    for (size_t i = 0; i < origin_.size(); ++i) {
        edge_.at(2 * i) = size_.at(2 * i) + origin_.at(i);
        edge_.at(2 * i + 1) = size_.at(2 * i + 1) + origin_.at(i);
    }
}

std::vector<float> BoxFilter::GetEdge() {
    return edge_;
}


class Matching {
  public:
    Matching(std::shared_ptr<rclcpp::Node> node);

    bool Registe_2_globalmap(const PointCloudXYZI::Ptr& cloud_data,Eigen::Matrix4d& predict_from_imu,Eigen::Matrix4d& cloud_pose);

    bool InitFromGuess(const Eigen::Matrix4d& guess_pose,const PointCloudXYZI::Ptr &init_pointcloud,Eigen::Matrix4d& result);

    bool SetInitPose(const Eigen::Matrix4d& init_pose);
    bool SetInited(void);

    Eigen::Matrix4d GetInitPose(void);
    void GetGlobalMap(pcl::PointCloud<pcl::PointXYZI>::Ptr& global_map);
    pcl::PointCloud<pcl::PointXYZI>::Ptr& GetLocalMap();
    pcl::PointCloud<pcl::PointXYZI>::Ptr& GetCurrentScan();
    bool HasInited();
    bool HasNewGlobalMap();
    bool HasNewLocalMap();
    float GetFitnessScore() { return registration_ptr_->GetFitnessScore(); }

  private:
    bool InitWithConfig();
    bool InitDataPath();
    bool InitScanContextManager();
    bool InitRegistration(std::shared_ptr<NDTRegistration>& registration_ptr, std::shared_ptr<rclcpp::Node> node);
    bool InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, std::shared_ptr<rclcpp::Node> node);
    bool InitBoxFilter(std::shared_ptr<rclcpp::Node> node);

    bool InitGlobalMap();
    bool ResetLocalMap(float x, float y, float z);

  private:
    std::string map_path_ = "";

    std::shared_ptr<NDTRegistration> registration_ptr_; 

    std::shared_ptr<CloudFilterInterface> global_map_filter_ptr_;

    std::shared_ptr<BoxFilter> box_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;

    std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr global_map_ptr_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr local_map_ptr_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_scan_ptr_;

    Eigen::Matrix4d init_pose_ = Eigen::Matrix4d::Identity();

    bool has_inited_ = false;
    bool has_new_global_map_ = false;
    bool has_new_local_map_ = false;

    float init_threshold;

};

Matching::Matching(std::shared_ptr<rclcpp::Node> node)
    : global_map_ptr_(new pcl::PointCloud<pcl::PointXYZI>),
      local_map_ptr_(new pcl::PointCloud<pcl::PointXYZI>),
      current_scan_ptr_(new pcl::PointCloud<pcl::PointXYZI>) 
{
    std::cout << std::endl
              << "-----------------Init Localization-------------------" 
              << std::endl;
    
    // 参数已在FastLioNode中声明，这里只获取
    node->get_parameter("globalmap_dir", map_path_);
    node->get_parameter("init_threshold", init_threshold);
    
    std::cout << "全局地图路径" << map_path_ << std::endl;

    InitRegistration(registration_ptr_, node);

    // a. global map filter -- downsample point cloud map for visualization:
    InitFilter("global_map", global_map_filter_ptr_, node);
    // b. local map filter -- downsample & ROI filtering for scan-map matching:
    InitBoxFilter(node);
    InitFilter("local_map", local_map_filter_ptr_, node);
    // c. scan filter -- 
    InitFilter("frame", frame_filter_ptr_, node);

    InitGlobalMap();

    ResetLocalMap(0.0, 0.0, 0.0);
}

bool Matching::InitRegistration(std::shared_ptr<NDTRegistration>& registration_ptr, std::shared_ptr<rclcpp::Node> node) {
    float res,step_size,trans_eps;
    int max_iter ;
    
    node->declare_parameter("NDT/res", 1.0f);
    node->declare_parameter("NDT/step_size", 0.1f);
    node->declare_parameter("NDT/trans_eps", 0.01f);
    node->declare_parameter("NDT/max_iter", 10);
    
    node->get_parameter("NDT/res", res);
    node->get_parameter("NDT/step_size", step_size);
    node->get_parameter("NDT/trans_eps", trans_eps);
    node->get_parameter("NDT/max_iter", max_iter);
    
    registration_ptr = std::make_shared<NDTRegistration>(res,step_size,trans_eps,max_iter);
    return true;
}

bool Matching::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, std::shared_ptr<rclcpp::Node> node) {
    std::string filter_method;
    
    node->declare_parameter(filter_user + "_filter", "empty");
    node->get_parameter(filter_user + "_filter", filter_method);
    
    std::cout << "\tFilter Method for " << filter_user << ": " << filter_method << std::endl;

    if (filter_method == "voxel_filter") {
        std::vector<double> filter_vec_d;
        float leaf_size_x,leaf_size_y,leaf_size_z;
        
        node->declare_parameter(filter_method + "/" + filter_user, std::vector<double>());
        node->get_parameter(filter_method + "/" + filter_user, filter_vec_d);
        std::vector<float> filter_vec(filter_vec_d.begin(), filter_vec_d.end());
        
        leaf_size_x = filter_vec[0];
        leaf_size_y = filter_vec[1];
        leaf_size_z = filter_vec[2];
        filter_ptr = std::make_shared<VoxelFilter>(leaf_size_x,leaf_size_y,leaf_size_z);
    } else if (filter_method == "no_filter") {
        filter_ptr = std::make_shared<NoFilter>();
    } else {
        std::cout << "Filter method " << filter_method << " for " << filter_user << " NOT FOUND!";
        return false;
    }

    return true;
}

bool Matching::InitBoxFilter(std::shared_ptr<rclcpp::Node> node) {
    std::vector<double> box_vec_d;
    
    node->declare_parameter("box_filter_size", std::vector<double>());
    node->get_parameter("box_filter_size", box_vec_d);
    std::vector<float> box_vec(box_vec_d.begin(), box_vec_d.end());
    
    box_filter_ptr_ = std::make_shared<BoxFilter>(box_vec);
    return true;
}

bool Matching::InitGlobalMap() {
    pcl::io::loadPCDFile(map_path_, *global_map_ptr_);
    std::cout << "Load global map, size:" << global_map_ptr_->points.size();

    // since scan-map matching is used, here apply the same filter to local map & scan:
    local_map_filter_ptr_->Filter(global_map_ptr_, global_map_ptr_);
    std::cout << "Filtered global map, size:" << global_map_ptr_->points.size() << std::endl;

    has_new_global_map_ = true;

    return true;
}

bool Matching::ResetLocalMap(float x, float y, float z) {
    std::vector<float> origin = {x, y, z};
    box_filter_ptr_->SetOrigin(origin);
    box_filter_ptr_->Filter(global_map_ptr_, local_map_ptr_);
    local_map_filter_ptr_->Filter(local_map_ptr_,local_map_ptr_);

    if (!registration_ptr_->SetInputTarget(local_map_ptr_)) {
        std::cout << "[NDT] ResetLocalMap: SetInputTarget 失败, 局部地图点数=" 
                  << local_map_ptr_->points.size() << std::endl;
        return false;
    }

    has_new_local_map_ = true;

    std::vector<float> edge = box_filter_ptr_->GetEdge();
    std::cout << "New local map:" << edge.at(0) << ","
                                  << edge.at(1) << ","
                                  << edge.at(2) << ","
                                  << edge.at(3) << ","
                                  << edge.at(4) << ","
                                  << edge.at(5) << std::endl << std::endl;

    return true;
}

bool Matching::SetInitPose(const Eigen::Matrix4d& init_pose) {
    init_pose_ = init_pose;
    ResetLocalMap(init_pose(0,3), init_pose(1,3), init_pose(2,3));

    return true;
}

bool Matching::SetInited(void) {
    has_inited_ = true;

    return true;
}

Eigen::Matrix4d Matching::GetInitPose(void) {
    return init_pose_;
}

void Matching::GetGlobalMap(pcl::PointCloud<pcl::PointXYZI>::Ptr& global_map) {
    // downsample global map for visualization:
    global_map_filter_ptr_->Filter(global_map_ptr_, global_map);

    has_new_global_map_ = false;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr& Matching::GetLocalMap() {
    has_new_local_map_ = false;
    return local_map_ptr_;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr& Matching::GetCurrentScan() {
    return current_scan_ptr_;
}

bool Matching::HasInited() {
    return has_inited_;
}

bool Matching::HasNewGlobalMap() {
    return has_new_global_map_;
}

bool Matching::HasNewLocalMap() {
    return has_new_local_map_;
}

bool Matching::InitFromGuess(const Eigen::Matrix4d& guess_pose,const PointCloudXYZI::Ptr &init_pointcloud,Eigen::Matrix4d& result){
    // 检查输入点云有效性
    if (!init_pointcloud || init_pointcloud->points.size() < 50) {
        std::cout << "[NDT] 扫描点数太少 (" << (init_pointcloud ? init_pointcloud->points.size() : 0) << "), 跳过匹配" << std::endl;
        result = guess_pose;
        return false;
    }
    
    // 检查NaN
    for (size_t i = 0; i < init_pointcloud->points.size(); i++) {
        const auto& p = init_pointcloud->points[i];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
            std::cout << "[NDT] 扫描含NaN/Inf, 跳过匹配" << std::endl;
            result = guess_pose;
            return false;
        }
    }
    
    if (!ResetLocalMap(guess_pose(0,3),guess_pose(1,3),guess_pose(2,3))) {
        std::cout << "[NDT] ResetLocalMap失败, 尝试扩大搜索范围" << std::endl;
        result = guess_pose;
        return false;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr result_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr init_input(new pcl::PointCloud<pcl::PointXYZI>());

    pcl::copyPointCloud(*init_pointcloud,*init_input);
    registration_ptr_->ScanMatch(init_input, guess_pose, result_cloud_ptr, result);

    // 检查匹配结果
    if (!std::isfinite(result(0,3)) || !std::isfinite(result(1,3)) || !std::isfinite(result(2,3))) {
        std::cout << "[NDT] 匹配结果含NaN, 使用初始猜测" << std::endl;
        result = guess_pose;
        return false;
    }

    if(registration_ptr_->GetFitnessScore()>init_threshold){
        std::cout << "initialization false , register score:" << registration_ptr_->GetFitnessScore() << std::endl;
        pcl::transformPointCloud(*init_input, *current_scan_ptr_, result);
        return false;
    }
    std::cout << "initialization success , register score:" << registration_ptr_->GetFitnessScore() << std::endl;
    pcl::transformPointCloud(*init_input, *current_scan_ptr_, result);
    return true;
}

bool Matching::Registe_2_globalmap(const PointCloudXYZI::Ptr& cloud_data,Eigen::Matrix4d& predict_from_imu ,Eigen::Matrix4d& cloud_pose){
    if (!cloud_data || cloud_data->points.size() < 10) return false;
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr result_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr reg_input(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::copyPointCloud(*cloud_data,*reg_input);
    // downsample:
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    frame_filter_ptr_->Filter(reg_input, filtered_cloud_ptr);
    
    if (filtered_cloud_ptr->points.size() < 10) return false;
        
    // matching:
    registration_ptr_->ScanMatch(filtered_cloud_ptr, predict_from_imu, result_cloud_ptr, cloud_pose);
    
    // 检查匹配结果
    if (!std::isfinite(cloud_pose(0,3)) || !std::isfinite(cloud_pose(1,3)) || !std::isfinite(cloud_pose(2,3))) {
        std::cout << "[NDT] 连续匹配结果含NaN, 跳过" << std::endl;
        cloud_pose = predict_from_imu;
        return false;
    }
    
    pcl::transformPointCloud(*reg_input, *current_scan_ptr_, cloud_pose);

    // 匹配之后判断是否需要更新局部地图
    std::vector<float> edge = box_filter_ptr_->GetEdge();
    for (int i = 0; i < 3; i++) {
        if (
            fabs(cloud_pose(i, 3) - edge.at(2 * i)) > 50.0 &&
            fabs(cloud_pose(i, 3) - edge.at(2 * i + 1)) > 50.0
        ) {
            continue;
        }
            
        ResetLocalMap(cloud_pose(0,3), cloud_pose(1,3), cloud_pose(2,3));
        break;
    }

    return true;
}


#endif
