#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

struct Pointxyzi
{
    PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
    float i;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (Pointxyzi, // here we assume a XYZ + "i" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, i, i)
)

struct PointXYZIRR
{
    PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
    float intensity;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRR, // here we assume a XYZ + "i" (as fields)
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, intensity, intensity)
                                   (uint16_t, ring, ring)
)
class AddRing {
private:
    ros::NodeHandle nh;
    ros::Subscriber cloud_sub;
    ros::Publisher cloud_pub;

public:
    AddRing() {
        cloud_sub = nh.subscribe("/kitti/velo/pointcloud", 100,
                &AddRing::callback, this);
        cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/kitti/velo/pointcloud_withring", 100);
    }
    int get_quadrant(pcl::PointXYZ pt_in) {
        int quadrant = 0;
        double x = pt_in.x;
        double y = pt_in.y;
        if(x > 0 && y >= 0) {
            quadrant = 1;
        } else if(x <= 0 && y > 0) {
            quadrant = 2;
        } else if(x < 0 && y <= 0) {
            quadrant = 3;
        } else {
            quadrant = 4;
        }
        return quadrant;
    }

    pcl::PointCloud<PointXYZIRR>::Ptr add_ring_info(pcl::PointCloud<Pointxyzi>::Ptr cloud_msg) {
        pcl::PointCloud<PointXYZIRR>::Ptr cloud_out(new pcl::PointCloud<PointXYZIRR>);
        cloud_out->points.resize(cloud_msg->size());
        int num_of_rings = 64;
        int prev_quadrant = 0;
        int ring = 0;
        int i = 0;
        for(pcl::PointCloud<Pointxyzi>::iterator it = cloud_msg->begin();
            it != cloud_msg->end(); it++, i++){
            pcl::PointXYZ pt;
            float x, y, z;
            x = pt.x = cloud_out->points[i].x = it->x;
            y = pt.y = cloud_out->points[i].y = it->y;
            z = pt.z = cloud_out->points[i].z = it->z;
            cloud_out->points[i].intensity = it->i;
            int quadrant = get_quadrant(pt);
            if((quadrant == 1) && (prev_quadrant == 4) && (ring < num_of_rings -1)) {
                ring ++;
            }
            cloud_out->points[i].ring = ring;
            prev_quadrant = quadrant;
        }

        return cloud_out;
    }

    void callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
        pcl::PointCloud<Pointxyzi>::Ptr cloud (new pcl::PointCloud<Pointxyzi>);
        pcl::PointCloud<PointXYZIRR>::Ptr cloud_out(new pcl::PointCloud<PointXYZIRR>);
        pcl::fromROSMsg(*cloud_msg, *cloud);
        cloud_out = add_ring_info(cloud);
        sensor_msgs::PointCloud2 cloud_out_ros;;
        pcl::toROSMsg(*cloud_out, cloud_out_ros);
        cloud_out_ros.header.stamp = cloud_msg->header.stamp;
        cloud_out_ros.header.frame_id = cloud_msg->header.frame_id;
        cloud_pub.publish(cloud_out_ros);
    }
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "Add Ring");
    AddRing ar;
    ros::spin();
    return 0;
}