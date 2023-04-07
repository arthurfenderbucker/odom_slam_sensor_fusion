#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <odom_slam_sensor_fusion/SetScale.h> 

class PointCloudScaler {
public:
    PointCloudScaler() {
        // Subscribe to the input point cloud topic
        pointcloud_sub_ = nh_.subscribe("input_pointcloud_topic", 1, &PointCloudScaler::pointcloudCallback, this);


        service_ = nh_.advertiseService("set_scale", &PointCloudScaler::setScaleCallback, this);
        // Advertise the output point cloud topic
        pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("scaled_pointcloud_topic", 1);
    }

private:
    double f = 1.0;
    // double offset_x = 0.0;

    bool setScaleCallback(odom_slam_sensor_fusion::SetScale::Request &req,
                      odom_slam_sensor_fusion::SetScale::Response &res)
    {
        f = req.scale;
        // offset_x = req.x;
        // offset_y = req.y;
        // offset_z = req.z;


        ROS_INFO("New scale set to: %f", f);
        res.success = true;
        return true;
    }

    void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_pointcloud_msg) {
        // Convert the input point cloud message to a PCL point cloud


        if (pointcloud_pub_.getNumSubscribers() > 0){
            pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*input_pointcloud_msg, *input_cloud);

            // Scale the point cloud by a factor 'f'
            // double f = 2.0; // Replace with desired scaling factor
            pcl::PointCloud<pcl::PointXYZ>::Ptr scaled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*input_cloud, *scaled_cloud);
            for (size_t i = 0; i < scaled_cloud->points.size(); ++i) {
                // scaled_cloud->points[i].x += f;
                // scaled_cloud->points[i].y += f;
                // scaled_cloud->points[i].z += f;
                scaled_cloud->points[i].x *= f;
                scaled_cloud->points[i].y *= f;
                scaled_cloud->points[i].z *= f;
            }

            // Convert the scaled point cloud back to a ROS message
            sensor_msgs::PointCloud2 output_pointcloud_msg;
            pcl::toROSMsg(*scaled_cloud, output_pointcloud_msg);
            output_pointcloud_msg.header = input_pointcloud_msg->header;
            output_pointcloud_msg.header.frame_id = "slam_scaled";

            // Publish the scaled point cloud message to the output topic
            pointcloud_pub_.publish(output_pointcloud_msg);
        }
    }

    ros::NodeHandle nh_;
    ros::Subscriber pointcloud_sub_;
    ros::Publisher pointcloud_pub_;
    ros::ServiceServer service_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_scaler_node",ros::init_options::AnonymousName);
    PointCloudScaler scaler;
    ros::spin();
    return 0;
}