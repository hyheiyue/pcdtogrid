#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/surface/mls.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include <chrono>
#include <fstream>
#include <iostream>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <string>

class PcdToGridMap : public rclcpp::Node {
public:
    PcdToGridMap() : Node("pcd_to_grid_map") {
        rclcpp::QoS map_qos(10);
        map_qos.transient_local();
        map_qos.reliable();
        map_qos.keep_last(1);
        map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", map_qos);

        // 参数初始化
        this->declare_parameter("min_z", 0.5);
        this->declare_parameter("max_z", 1.0);
        this->declare_parameter("radius", 0.5);
        this->declare_parameter("min_pts", 30);
        this->declare_parameter("file_name", "transformed_cloud.pcd"); // 使用相对路径

        this->get_parameter("file_name", file_name);
        this->get_parameter("min_z", min_z);
        this->get_parameter("max_z", max_z);
        this->get_parameter("radius", radius);
        this->get_parameter("min_pts", min_pts);

        RCLCPP_INFO(this->get_logger(), "min_z: %f, max_z: %f, radius: %f, min_pts: %d, file_name: %s", 
                    min_z, max_z, radius, min_pts, file_name.c_str());

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *cloud) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file!");
            throw std::runtime_error("Failed to load PCD file");
        }
        RCLCPP_INFO(this->get_logger(), "Loaded PCD file with %zu points", cloud->size());

        // 处理点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(cloud); // 复用输入点云对象
        filterPointCloud(filtered_cloud);

        // 生成栅格地图
        grid_map_ = createGridMap(filtered_cloud);
        this->publishMap();
    }

private:
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
    nav_msgs::msg::OccupancyGrid grid_map_;
    double min_z;
    double max_z;
    double radius;
    int min_pts;
    std::string file_name;

    void filterPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f);
        voxel_filter.filter(*cloud);

        pcl::PassThrough<pcl::PointXYZ> pass_filter;
        pass_filter.setInputCloud(cloud);
        pass_filter.setFilterFieldName("z");
        pass_filter.setFilterLimits(min_z, max_z);
        pass_filter.filter(*cloud);

        pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_filter;
        radius_filter.setInputCloud(cloud);
        radius_filter.setRadiusSearch(radius);
        radius_filter.setMinNeighborsInRadius(min_pts);
        radius_filter.filter(*cloud);
    }

    nav_msgs::msg::OccupancyGrid createGridMap(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
        nav_msgs::msg::OccupancyGrid grid_map;
        grid_map.header.frame_id = "map";
        grid_map.header.stamp = this->get_clock()->now();
        grid_map.info.resolution = 0.1;

        double min_x = std::numeric_limits<double>::max();
        double min_y = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();
        double max_y = std::numeric_limits<double>::lowest();

        for (const auto &point : cloud->points) {
            min_x = std::min(min_x, point.x);
            min_y = std::min(min_y, point.y);
            max_x = std::max(max_x, point.x);
            max_y = std::max(max_y, point.y);
        }

        int width = static_cast<int>(std::ceil((max_x - min_x) / grid_map.info.resolution));
        int height = static_cast<int>(std::ceil((max_y - min_y) / grid_map.info.resolution));
        grid_map.info.width = width;
        grid_map.info.height = height;

        grid_map.info.origin.position.x = min_x;
        grid_map.info.origin.position.y = min_y;
        grid_map.info.origin.position.z = 0.0;
        grid_map.info.origin.orientation.w = 1.0;

        std::vector<int8_t> occupancy_data(width * height, 0);

        for (const auto &point : cloud->points) {
            int grid_x = static_cast<int>((point.x - min_x) / grid_map.info.resolution);
            int grid_y = static_cast<int>((point.y - min_y) / grid_map.info.resolution);

            if (grid_x >= 0 && grid_x < width && grid_y >= 0 && grid_y < height) {
                int index = grid_y * width + grid_x;
                occupancy_data[index] = 100;
            }
        }

        grid_map.data = occupancy_data;
        return grid_map;
    }

    void publishMap() {
        grid_map_.header.stamp = this->get_clock()->now();
        map_publisher_->publish(grid_map_);
        RCLCPP_INFO(this->get_logger(), "Published map");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<PcdToGridMap>());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("pcd_to_grid_map"), "Exception occurred: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}