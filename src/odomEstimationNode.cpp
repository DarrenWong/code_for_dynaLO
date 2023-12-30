// Author of FLOAM: Wang Han
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

// c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

// ros lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <novatel_msgs/INSPVAX.h> // novatel_msgs/INSPVAX
#include <nav_msgs/Path.h>
// pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// local lib
#include "lidar.h"
#include "odomEstimationClass.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

OdomEstimationClass odomEstimation;
std::mutex mutex_lock;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudEdgeBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudSurfBuf;
lidar::Lidar lidar_param;

ros::Publisher pubLaserOdometry;

ros::Publisher pub_surf_cloud;
ros::Publisher pub_weighted_cloud;

struct PointXYZIRTL
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    double time;
    uint16_t ring;
    int32_t label;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRTL,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(double, time, time)(int, label, label))

void velodyneSurfHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();

    pointCloudSurfBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}
void velodyneEdgeHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudEdgeBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}

bool first_data = true;

Eigen::Quaterniond Eulerangle2Quaternion(double &fi, double &thita, double &posi)
{
    Eigen::Matrix3d R;
    R << cos(posi) * cos(fi) - sin(posi) * sin(thita) * sin(fi), -sin(posi) * cos(thita), cos(posi) * sin(fi) + sin(posi) * sin(thita) * cos(fi),
        sin(posi) * cos(fi) + cos(posi) * sin(thita) * sin(fi), cos(posi) * cos(thita), sin(posi) * sin(fi) - cos(posi) * sin(thita) * cos(fi),
        -cos(thita) * sin(fi), sin(thita), cos(thita) * cos(fi);
    Eigen::Quaterniond q(R);
    return q.normalized();
}

std::string data_path_gps = "/tmp/gps.txt";

std::ofstream output_file_gps_(data_path_gps);
std::string data_path_odom = "/tmp/odom.txt";

std::ofstream output_file_odom_(data_path_odom);

bool is_odom_inited = false;
double total_time = 0;
int total_frame = 0;
void odom_estimation()
{
    while (1)
    {
        if (!pointCloudEdgeBuf.empty() && !pointCloudSurfBuf.empty())
        {

            // read data
            mutex_lock.lock();
            if (!pointCloudSurfBuf.empty() && (pointCloudSurfBuf.front()->header.stamp.toSec() < pointCloudEdgeBuf.front()->header.stamp.toSec() - 0.5 * lidar_param.scan_period))
            {
                pointCloudSurfBuf.pop();
                ROS_WARN_ONCE("time stamp unaligned with extra point cloud, pls check your data --> odom correction");
                mutex_lock.unlock();
                continue;
            }

            if (!pointCloudEdgeBuf.empty() && (pointCloudEdgeBuf.front()->header.stamp.toSec() < pointCloudSurfBuf.front()->header.stamp.toSec() - 0.5 * lidar_param.scan_period))
            {
                pointCloudEdgeBuf.pop();
                ROS_WARN_ONCE("time stamp unaligned with extra point cloud, pls check your data --> odom correction");
                mutex_lock.unlock();
                continue;
            }
            // if time aligned
            pcl::PointCloud<PointXYZIRTL>::Ptr pointcloud_surf_in_raw(new pcl::PointCloud<PointXYZIRTL>());

            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge_in(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg(*pointCloudEdgeBuf.front(), *pointcloud_edge_in);
            pcl::fromROSMsg(*pointCloudSurfBuf.front(), *pointcloud_surf_in_raw);

            for (int i = 0; i < pointcloud_surf_in_raw->points.size(); i++)
            {
                pcl::PointXYZI point_convert;
                point_convert.x = pointcloud_surf_in_raw->points[i].x;
                point_convert.y = pointcloud_surf_in_raw->points[i].y;
                point_convert.z = pointcloud_surf_in_raw->points[i].z;

                point_convert.intensity = 0;
                if (pointcloud_surf_in_raw->points[i].label > 0)
                {
                    // continue;
                    point_convert.intensity = pointcloud_surf_in_raw->points[i].label;
                }
                pointcloud_surf_in->points.push_back(point_convert);
            }
            sensor_msgs::PointCloud2 msg;
            pcl::toROSMsg(*pointcloud_surf_in, msg);
            msg.header.frame_id = "map";
            msg.header.stamp = ros::Time::now();

            pub_surf_cloud.publish(msg);

            ros::Time pointcloud_time = (pointCloudSurfBuf.front())->header.stamp;
            pointCloudEdgeBuf.pop();
            pointCloudSurfBuf.pop();
            mutex_lock.unlock();

            if (is_odom_inited == false)
            {
                odomEstimation.initMapWithPoints(pointcloud_edge_in, pointcloud_surf_in);
                is_odom_inited = true;

                ROS_INFO("odom inited");
            }
            else
            {
                std::chrono::time_point<std::chrono::system_clock> start, end;
                start = std::chrono::system_clock::now();
                odomEstimation.updatePointsToMap(pointcloud_edge_in, pointcloud_surf_in, pointcloud_time);
                end = std::chrono::system_clock::now();
                std::chrono::duration<float> elapsed_seconds = end - start;
                total_frame++;
                float time_temp = elapsed_seconds.count() * 1000;
                total_time += time_temp;
                ROS_INFO("average odom estimation time %f ms \n \n", total_time / total_frame);
            }

            // publish weighted object points
            sensor_msgs::PointCloud2 msg_obj;
            pcl::toROSMsg(*(odomEstimation.plannar_feature_with_weight), msg_obj);
            msg_obj.header.frame_id = "velodyne";
            msg_obj.header.stamp = ros::Time::now();

            pub_weighted_cloud.publish(msg_obj);

            Eigen::Quaterniond q_current(odomEstimation.odom.rotation());
            // q_current.normalize();
            Eigen::Vector3d t_current = odomEstimation.odom.translation();

            static tf::TransformBroadcaster br;
            tf::Transform transform;
            transform.setOrigin(tf::Vector3(t_current.x(), t_current.y(), t_current.z()));
            tf::Quaternion q(q_current.x(), q_current.y(), q_current.z(), q_current.w());
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

            // publish odometry
            nav_msgs::Odometry laserOdometry;
            laserOdometry.header.frame_id = "map";
            laserOdometry.child_frame_id = "base_link";
            laserOdometry.header.stamp = pointcloud_time;
            laserOdometry.pose.pose.orientation.x = q_current.x();
            laserOdometry.pose.pose.orientation.y = q_current.y();
            laserOdometry.pose.pose.orientation.z = q_current.z();
            laserOdometry.pose.pose.orientation.w = q_current.w();
            laserOdometry.pose.pose.position.x = t_current.x();
            laserOdometry.pose.pose.position.y = t_current.y();
            laserOdometry.pose.pose.position.z = t_current.z();
            pubLaserOdometry.publish(laserOdometry);

            output_file_odom_ << std::setprecision(20) << pointcloud_time.toSec() << " " << t_current.x() << " " << t_current.y()
                              << " " << t_current.z() << " " << q_current.x() << " " << q_current.y() << " " << q_current.z() << " " << q_current.w() << std::endl;
        }
        // sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = true;

    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    int scan_line = 64;
    double vertical_angle = 2.0;
    double scan_period = 0.1;
    double max_dis = 60.0;
    double min_dis = 2.0;
    double map_resolution = 0.4;
    bool use_reweight = false;
    bool use_remove_only = false;

    nh.getParam("/scan_period", scan_period);
    nh.getParam("/vertical_angle", vertical_angle);
    nh.getParam("/max_dis", max_dis);
    nh.getParam("/min_dis", min_dis);
    nh.getParam("/scan_line", scan_line);
    nh.getParam("/map_resolution", map_resolution);
    nh.getParam("/use_reweight", use_reweight);
    nh.getParam("/use_remove_only", use_remove_only);

    lidar_param.setScanPeriod(scan_period);
    lidar_param.setVerticalAngle(vertical_angle);
    lidar_param.setLines(scan_line);
    lidar_param.setMaxDistance(max_dis);
    lidar_param.setMinDistance(min_dis);
    lidar_param.setUseReweight(use_reweight);
    lidar_param.setUseRemove(use_remove_only);

    odomEstimation.init(lidar_param, map_resolution);
    // ros::Subscriber subEdgeLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_edge", 100, velodyneEdgeHandler);
    // ros::Subscriber subSurfLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf", 100, velodyneSurfHandler);
    // //LOAM
    ros::Subscriber subEdgeLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 100, velodyneEdgeHandler);
    ros::Subscriber subSurfLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 100, velodyneSurfHandler);

    pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/odom", 100);
    std::thread odom_estimation_process{odom_estimation};

    pub_surf_cloud =
        nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_surf", 2);
    pub_weighted_cloud =
        nh.advertise<sensor_msgs::PointCloud2>("/object_weighted", 2);
    pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/odom", 100);
    ros::spin();

    return 0;
}
