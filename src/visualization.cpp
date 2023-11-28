#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>


#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Path.h>

using namespace std;
typedef pcl::PointXYZI PointType;
std::vector<std::vector<int>> pixel_classes;

int pixel_class_value;


deque<pcl::PointCloud<PointType>> cloudQueue;
deque<double> timeQueue;
pcl::PointCloud<PointType>::Ptr depthCloud(new pcl::PointCloud<PointType>());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr outside_image_RGB_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

// distortion_parameters:
  double k1 = -0.07253575;
  double k2 = 0.06272369;  
  double k3 = 0.0;
  double p1 = -0.00102158;
  double p2 = 0.00360962;  
// projection_parameters: 
  double fx = 539.12184959;
  double fy = 537.93396235;  
  double cx = 477.46520972; 
  double cy = 270.09177125; 


Eigen::Matrix4f camera_lidar_tf;

cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
cv::Mat dist_coeffs = (cv::Mat_<double>(5,1) << k1, k2, p1, p2, k3);
double L_C_TX;
double L_C_TY;
double L_C_TZ;
double L_C_RX;
double L_C_RY;
double L_C_RZ;
Eigen::Affine3f transOffset;
// Eigen::Affine3f transOffset = pcl::getTransformation(L_C_TX, L_C_TY, L_C_TZ, L_C_RX, L_C_RY, L_C_RZ);

void publishRGBCloud(ros::Publisher *thisPub, pcl::PointCloud<pcl::PointXYZRGB>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    if (thisPub->getNumSubscribers() == 0)
        return;
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    thisPub->publish(tempCloud); 
}


void setCameraLidarTf()  
{
  pixel_classes.resize(540, std::vector<int>(960));

  L_C_TX = 0.143;
  L_C_TY = -0.30;
  L_C_TZ = -0.012;
  L_C_RX = 0.0;
  L_C_RY = -1.5707963;
  L_C_RZ = 1.5707963;

  transOffset = pcl::getTransformation(L_C_TX, L_C_TY, L_C_TZ, L_C_RX, L_C_RY, L_C_RZ);


}

cv::Point2f project3DTo2D(float x, float y, float z, const cv::Mat& camera_matrix)
{
  cv::Point2f pixel;
  pixel.x = (x * camera_matrix.at<double>(0, 0) / z) + camera_matrix.at<double>(0, 2);
  pixel.y = (y * camera_matrix.at<double>(1, 1) / z) + camera_matrix.at<double>(1, 2);
  return pixel;
}


class SubscribeAndPublish
{
public:
  tf::Transform transform__;
  tf::TransformListener listener__;
  tf::Transform inv_transform;


  SubscribeAndPublish()
  {

    pub_1 = n_.advertise<sensor_msgs::PointCloud2>("transformed_velodyne",1);
    path_pub = n_.advertise<nav_msgs::Path>("path", 10);


    //섭스크라이브 할 토픽픽 선언
    sub_img = n_.subscribe<sensor_msgs::Image>("/yonsei_rs/prior", 5, &SubscribeAndPublish::imageCallback,this);
    sub_cloud = n_.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 5, &SubscribeAndPublish::pointCloudCallback,this);
    sub_odom = n_.subscribe<nav_msgs::Odometry>("/odom", 1, &SubscribeAndPublish::odometryCallback, this);

  	
  }


  void publishRGBCloud(ros::Publisher *thisPub, pcl::PointCloud<pcl::PointXYZRGB>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame)
  {
      if (thisPub->getNumSubscribers() == 0)
          return;
      sensor_msgs::PointCloud2 tempCloud;
      sensor_msgs::PointCloud2 transform_tempCloud;
      pcl::toROSMsg(*thisCloud, tempCloud);
      tempCloud.header.stamp = thisStamp;
      tempCloud.header.frame_id = thisFrame;
      static tf::TransformBroadcaster br;

      // Broadcast the transform. Here, we assume that the child frame is "velodyne" and the parent frame is "map".
      br.sendTransform(tf::StampedTransform(transform__, ros::Time::now(), "map", "velodyne"));

      try
      {
          // Make sure we have the transformation available.
          listener__.waitForTransform("map", "velodyne", tempCloud.header.stamp, ros::Duration(1.0));

          // Perform the transformation. 
          pcl_ros::transformPointCloud("map", tempCloud, transform_tempCloud, listener__);
      }
      catch (tf::TransformException &ex)
      {
          ROS_ERROR("%s", ex.what());
          ros::Duration(1.0).sleep();
          return;
      }

      thisPub->publish(transform_tempCloud); 
  }




  void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
      geometry_msgs::PoseStamped pose;
      // public_pose.header.stamp = msg->header.stamp;
      pose.header = msg->header;
      pose.pose = msg->pose.pose;
      
      path.header = msg->header;
      path.poses.push_back(pose);
      
      // Construct a quaternion from the orientation components
      tf::Quaternion q(msg->pose.pose.orientation.x, 
                      msg->pose.pose.orientation.y, 
                      msg->pose.pose.orientation.z, 
                      msg->pose.pose.orientation.w);

      // Construct a 3D vector from the position components
      tf::Vector3 v(msg->pose.pose.position.x, 
                    msg->pose.pose.position.y, 
                    msg->pose.pose.position.z);

      // Update the transform object with the new position and orientation
      transform__.setOrigin(v);
      transform__.setRotation(q);
      inv_transform = transform__.inverse();
      path_pub.publish(path);
  }



  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {

    
    int image_height = msg->height;
    int image_width = msg->width;


    for (int row = 0; row < image_height; ++row)
    {
        for (int col = 0; col < image_width; ++col)
        {
            pixel_classes[row][col] = static_cast<int>(msg->data[row * image_width + col]);
        }
    }

  }


  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {

    outside_image_RGB_cloud->clear();
    pcl::PointCloud<PointType>::Ptr laser_cloud_in(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*cloud_msg, *laser_cloud_in);

    // 2. downsample new cloud (save memory)
    pcl::PointCloud<PointType>::Ptr laser_cloud_in_ds(new pcl::PointCloud<PointType>());
    static pcl::VoxelGrid<PointType> downSizeFilter;
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilter.setInputCloud(laser_cloud_in);
    downSizeFilter.filter(*laser_cloud_in_ds);
    *laser_cloud_in = *laser_cloud_in_ds; // downsize filter 된 point cloud : laser_cloud_in_ds, 이걸 다시 laser_cloud_in에 넣음

    // 3. filter lidar points (only keep points in camera view)
    pcl::PointCloud<PointType>::Ptr laser_cloud_in_filter(new pcl::PointCloud<PointType>());
    for (int i = 0; i < (int)laser_cloud_in->size(); ++i)
    {
        PointType p = laser_cloud_in->points[i];
        if (p.x >= 0 && abs(p.y / p.x) <= 10 && abs(p.z / p.x) <= 10)
            {
                laser_cloud_in_filter->push_back(p);
            }
    }   // laser_cloud_in에서 Camera view 안에 있는 포인트 클라우드들만 남김 laser_cloud_in_filter
    *laser_cloud_in = *laser_cloud_in_filter;   // laser_cloud_in_filter를 다시 laser_cloud_in에 넣음



    // 4. transformed_cloud에 transOffset 만큼 transform된 laser_cloud_in을 저장
    pcl::PointCloud<PointType>::Ptr transformed_cloud(new pcl::PointCloud<PointType>);
    // pcl::PointCloud<PointType>::Ptr inv_transformed_cloud(new pcl::PointCloud<PointType>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inv_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    // Lidar to Camera Transformation (in local frame)
    Eigen::Affine3f transOffset = pcl::getTransformation(L_C_TX, L_C_TY, L_C_TZ, L_C_RX, L_C_RY, L_C_RZ);
    pcl::transformPointCloud(*laser_cloud_in, *transformed_cloud, transOffset);
    // output : transformed_cloud



    pcl::PointXYZRGB point_;


   
    for (int i = 0; i < (int)transformed_cloud->size(); ++i)
    {     

      point_.x = transformed_cloud->points[i].x;
      point_.y = transformed_cloud->points[i].y;
      point_.z = transformed_cloud->points[i].z;

      // project3DTo2D 함수를 이용해서 point cloud의 x,y,z를 2D pixel로 변환
      cv::Point2f pixel = project3DTo2D(point_.x, point_.y, point_.z, camera_matrix);

      // 2D 픽셀로 변환된 pixel.x, pixel.y를 이용해서 pixel_classes에 접근해서 class value를 가져옴
      if ((int)pixel.x < 0 || (int)pixel.x >= 960 || (int)pixel.y < 0 || (int)pixel.y >= 540)
      {
        // 이미지 픽셀 범위 밖에 있는 포인트들은 제외
      }
      else
      {
        // pixel_class_value를 이용해서 point cloud의 RGB 값을 설정
        pixel_class_value  = pixel_classes[(int)pixel.y][(int)pixel.x];

        if(pixel_class_value == 0) // 분홍
        {
            point_.r = 255;  // Set the red channel value
            point_.g = 191;    // Set the green channel value
            point_.b = 190;    // Set the blue channel value
        }
        else if(pixel_class_value == 1) // 회색
        {
            point_.r = 200;  // Set the red channel value
            point_.g = 200;    // Set the green channel value
            point_.b = 200;    // Set the blue channel value
        }
        else if(pixel_class_value == 2) // 노랑
        {
            point_.r = 245;  // Set the red channel value
            point_.g = 243;    // Set the green channel value
            point_.b = 131;    // Set the blue channel value
        }
        else if(pixel_class_value == 3) // 탁한 분홍(?)
        {
            point_.r = 190;  // Set the red channel value
            point_.g = 153;    // Set the green channel value
            point_.b = 153;    // Set the blue channel value
        }
        else if(pixel_class_value == 4) // 파랑
        {
            point_.r = 0;  // Set the red channel value
            point_.g = 0;    // Set the green channel value
            point_.b = 255;    // Set the blue channel value
        }
        else if(pixel_class_value == 5) // 하늘
        {
            point_.r = 0;  // Set the red channel value
            point_.g = 224;    // Set the green channel value
            point_.b = 228;    // Set the blue channel value
        }
        else if(pixel_class_value == 6) // 주황
        {
            point_.r = 250;  // Set the red channel value
            point_.g = 170;    // Set the green channel value
            point_.b = 31;    // Set the blue channel value
        }
        else 
        {
            point_.r = 255;  // Set the red channel value
            point_.g = 0;    // Set the green channel value
            point_.b = 0;    // Set the blue channel value
        }

        // 동적 메모리로 정의된 PointXYZRGB type의 포인트 클라우드 outside_image_RGB_cloud에 point_를 push_back
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr outside_image_RGB_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        outside_image_RGB_cloud->push_back(point_); 
      }
    }

    // 6. outside_image_RGB_cloud를 Global frame으로 transform - output : inv_transformed_cloud
    pcl::transformPointCloud(*outside_image_RGB_cloud, *inv_transformed_cloud, transOffset.inverse());

    // 7. inv_transformed_cloud를 publish
    publishRGBCloud(&pub_1, inv_transformed_cloud, cloud_msg->header.stamp, "velodyne");
    outside_image_RGB_cloud->clear();

  }




private: //private으로 NodeHandle과 publisher, subscriber를 선언한다.
  ros::NodeHandle n_; 
  ros::Publisher pub_1; 
  ros::Publisher path_pub;


  ros::Subscriber sub_img;
  ros::Subscriber sub_cloud;
  ros::Subscriber sub_odom;

  sensor_msgs::PointCloud2 transformed_velodyne;
  sensor_msgs::Image transformed_Image;
  nav_msgs::Path path;



};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_pointcloud_fusion");
  setCameraLidarTf();
  // ros::Publisher labeled_point;

  SubscribeAndPublish SAPObject;
  ros::NodeHandle nh;

  // Initialize camera parameters, distortion coefficients and transformation matrix here

  // Subscriptions


  ros::spin();

  return 0;
}


