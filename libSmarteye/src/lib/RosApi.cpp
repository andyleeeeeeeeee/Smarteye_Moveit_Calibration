//
// Created by andylee on 2021/2/24.
//

#include <cmath>
#include <libSmartEye/RosApi.h>
#include <unistd.h>

namespace smart_eye
{
  RosApi::RosApi(ros::NodeHandle &nh, ros::NodeHandle &pnh) : nh_(nh), pnh_(pnh), init_(false)
  {
    initConnect("");

    // Initialize get point cloud service
    std::string get_pointcloud_srv_id;
    pnh_.getParam("get_pointcloud_srv_id", get_pointcloud_srv_id);
    if (get_pointcloud_srv_id.empty())
    {
      throw std::runtime_error("get_pointcloud_srv_id is None");
    }
    get_pointcloud_srv_ = nh_.advertiseService(get_pointcloud_srv_id, &RosApi::getPointCloudSrvCb, this);
    ROS_INFO_STREAM("Advertising service " << get_pointcloud_srv_id);

    // Initialize image publisher
    std::string pub_image_msg_id;
    pnh_.getParam("pub_image_msg_id", pub_image_msg_id);
    if (pub_image_msg_id.empty())
    {
      throw std::runtime_error("pub_image_msg_id is None");
    }
    image_pub_ = nh_.advertise<sensor_msgs::Image>(pub_image_msg_id, 1);
    ROS_INFO_STREAM("Publish image to " << pub_image_msg_id);

    // Initialize camera info publisher
    std::string camera_info_msg_id;
    pnh_.getParam("camera_info_msg_id", camera_info_msg_id);
    if (camera_info_msg_id.empty())
    {
      throw std::runtime_error("camera_info_msg_id is None");
    }
    camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>(camera_info_msg_id, 1);
    ROS_INFO_STREAM("Publish image to " << camera_info_msg_id);


    // Initialize camera frame
    pnh_.getParam("camera_frame", camera_frame_);   

    //pub once camera info
    // pubCameraInfo();
  }

  RosApi::~RosApi() {}

  bool RosApi::initConnect(std::string serial_no){
    printf("SmartEyeCameraController connect: %s ", serial_no.c_str());
    int result;
    result = controller_.initDevice();
    ROS_WARN("result value is: %d. If it is not 0, you should check common_type.h to debug \n", result);
    if (SE_STATUS_SUCCESS == result)
    {
      // set z range
      float min_z, max_z;
      pnh_.getParam("min_depth", min_z);
      pnh_.getParam("max_depth", max_z);
      controller_.setZRange(min_z, max_z); // in millimeter
      ROS_WARN("Initialized depth range: %.3f, %.3f mm \n", min_z, max_z);

      // set xy range
      float min_x, max_x, min_y, max_y;
      pnh_.getParam("min_length", min_x);
      pnh_.getParam("max_length", max_x);
      pnh_.getParam("min_width", min_y);
      pnh_.getParam("max_width", max_y);
      controller_.setXYRange(min_x, max_x, min_y, max_y); // in millimeter
      ROS_WARN("Initialized length range: %.3f, %.3f mm \n", min_x, max_x);
      ROS_WARN("Initialized width range: %.3f, %.3f mm \n", min_y, max_y);

      // set exposure time
      int ExposureTime2D, ExposureTime3D;
      pnh_.getParam("exp_time_2D", ExposureTime2D);
      pnh_.getParam("exp_time_3D", ExposureTime3D);
      controller_.setExposureTime2D(ExposureTime2D);
      controller_.setExposureTime3D(ExposureTime3D);
      ROS_WARN("Initialized ExposureTime2D to: %d ms \n", ExposureTime2D);
      ROS_WARN("Initialized ExposureTime3D to: %d ms \n", ExposureTime3D);

      // set MaxCoeff
      float MaxCoeff = 0.95;
      controller_.setMaxCoeff(MaxCoeff);
      ROS_WARN("Initialized MaxCoeff to: %f \n", MaxCoeff);

      init_ = true;
      ROS_WARN("Initialize device succeed.");
    }
    else
    {
      init_ = false;
      ROS_ERROR("Initialize device failed.");
    }
  }

  bool RosApi::getPointCloudSrvCb(smarteye::GetPointCloud::Request &req,
                                  smarteye::GetPointCloud::Response &res)
  {
    if (!init_)
    {
      res.result_status = res.FAILED;
      return true;
    }

    // get image
    cv::Mat img, img_right;
    cv_bridge::CvImage img_bridge, img_bridge_raw, img_bridge_depth;
    sensor_msgs::Image img_msg, img_msg_raw, img_msg_depth;
    std_msgs::Header header;
    controller_.captureUnrectifiedImages(img,img_right);
    header.stamp = ros::Time::now();
    header.frame_id = camera_frame_;
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, img);
    img_bridge.toImageMsg(img_msg);
    image_pub_.publish(img_msg);

    ROS_WARN("SUCCESS PUBLISH IMAGE");

    // pub camera info
    int img_rows = img.rows;
    int img_cols = img.cols;
    ROS_WARN("SIZE OF IMAGE: %d,%d",img.rows,img.cols);
    sensor_msgs::CameraInfo cam_info;
    cam_info.header.stamp = ros::Time::now();
    cam_info.header.frame_id = camera_frame_;

    cam_info.height = img_rows;
    cam_info.width = img_cols;

    std::string distortion_model = "plumb_bob";
    cam_info.distortion_model = distortion_model;

    std::vector<double> D={ -1.0915087515655258e-01, 3.1817312840189016e-01,
       -1.9268259483023425e-03, 1.3395472814911422e-03,
       -5.0770861830649194e-01};
    cam_info.D = D;

    boost::array<double, 9> K={ 2.3859574916620668e+03, 0., 1.2684917797536505e+03, 0.,
       2.3847639039551354e+03, 1.0453598393308355e+03, 0., 0., 1. };
    cam_info.K = K;

    boost::array<double, 9> R={ 9.8986216507414326e-01, -7.2762002004105020e-04,
       -1.4202945019902072e-01, 1.2796774860712227e-03,
       9.9999197780638371e-01, 3.7956222689823961e-03,
       1.4202554904052056e-01, -3.9388947667573062e-03,
       9.8985515532715951e-01 };
    cam_info.R = R;

    boost::array<double, 12> P={ 2.2681988179331315e+03, 0., 1.1809547576904297e+03, 0., 0.,
       2.2681988179331315e+03, 1.0216730499267578e+03, 0., 0., 0., 1.,
       0. };
    cam_info.P = P;

    camera_info_pub_.publish(cam_info);
    ROS_WARN("SUCCESS PUBLISH CAM INFO");

    return true;
  }

  bool RosApi::convert2PCLPointCloud(const PointCloud_SE_Ptr &se_cloud,
                                     const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud)
  {
    if (nullptr == se_cloud || nullptr == pcl_cloud)
    {
      return false;
    }

    pcl_cloud->points.clear();
    int numOfPoints = se_cloud->points.size();
    for (int i = 0; i < numOfPoints; i++)
    {
      pcl::PointXYZRGB point;
      point.x = se_cloud->points[i].x;
      point.y = se_cloud->points[i].y;
      point.z = se_cloud->points[i].z;
      point.rgb = se_cloud->points[i].rgb;
      point.r = se_cloud->points[i].r;
      point.g = se_cloud->points[i].g;
      point.b = se_cloud->points[i].b;
      pcl_cloud->points.push_back(point);
    }
    pcl_cloud->width = se_cloud->width;
    pcl_cloud->height = se_cloud->height;
    return true;
  }
  
  void RosApi::pubCameraInfo(){
    cv::Mat img, img_right;
    controller_.captureUnrectifiedImages(img,img_right);
    int img_rows = img.rows;
    int img_cols = img.cols;
    ROS_WARN("SIZE OF IMAGE: %d,%d",img.rows,img.cols);

    // pub camera info
    sensor_msgs::CameraInfo cam_info;
    cam_info.header.stamp = ros::Time::now();
    cam_info.header.frame_id = camera_frame_;

    cam_info.height = img_rows;
    cam_info.width = img_cols;

    std::string distortion_model = "plumb_bob";
    cam_info.distortion_model = distortion_model;

    std::vector<double> D={ -1.0915087515655258e-01, 3.1817312840189016e-01,
       -1.9268259483023425e-03, 1.3395472814911422e-03,
       -5.0770861830649194e-01};
    cam_info.D = D;

    boost::array<double, 9> K={ 2.3859574916620668e+03, 0., 1.2684917797536505e+03, 0.,
       2.3847639039551354e+03, 1.0453598393308355e+03, 0., 0., 1. };
    cam_info.K = K;

    boost::array<double, 9> R={ 9.8986216507414326e-01, -7.2762002004105020e-04,
       -1.4202945019902072e-01, 1.2796774860712227e-03,
       9.9999197780638371e-01, 3.7956222689823961e-03,
       1.4202554904052056e-01, -3.9388947667573062e-03,
       9.8985515532715951e-01 };
    cam_info.R = R;

    boost::array<double, 12> P={ 2.2681988179331315e+03, 0., 1.1809547576904297e+03, 0., 0.,
       2.2681988179331315e+03, 1.0216730499267578e+03, 0., 0., 0., 1.,
       0. };
    cam_info.P = P;

    camera_info_pub_.publish(cam_info);
    }
}

