
#include <assert.h>
#include <stdlib.h>

#include "SyncTransfer.h"
#include "SyncTransfer_Controller_ROS.h"

KIROSyncTransferControllerROS::KIROSyncTransferControllerROS() : it_(nh_Pub)
{
    threadProc_KIROSyncTransferControllerROS_RunFlag = false;
    _bPublishRunFlag = false;
    _ROSState = 0;  // ROS Creation

    Initialize();

    InitServiceProvider();
    ROS_INFO("KIROSyncTransferControllerROS::Constructor: Init Service Provider");

    InitServiceClient();
    ROS_INFO("KIROSyncTransferControllerROS::Constructor: Init Service Client");

    InitPublisher();
    ROS_INFO("KIROSyncTransferControllerROS::Constructor: Init Publisher");

    InitSubscriber();
    ROS_INFO("KIROSyncTransferControllerROS::Constructor: Init Subscriber");
}

KIROSyncTransferControllerROS::~KIROSyncTransferControllerROS()
{
    Finalize();

    nh_Pub.shutdown();
    nh_Sub.shutdown();
    nh_Service.shutdown();
    nh_Client.shutdown();

    ROS_INFO("KIROSyncTransferControllerROS::Destructor");
}

int KIROSyncTransferControllerROS::Initialize()
{
    if(threadProc_KIROSyncTransferControllerROS_RunFlag != false) {
        StopThreadLoop();
    }

    ROS_INFO("KIROSyncTransferControllerROS::Initialize");

    return 0;
}

int KIROSyncTransferControllerROS::Finalize()
{
    if(threadProc_KIROSyncTransferControllerROS_RunFlag != false) {
        StopThreadLoop();
    }

    ROS_INFO("KIROSyncTransferControllerROS::Finalize");
    
    return 0;
}

int KIROSyncTransferControllerROS::StartThreadLoop()
{
    std::lock_guard<std::mutex> lock(_mtx);

    if(threadProc_KIROSyncTransferControllerROS_RunFlag == false) {
        threadProc_KIROSyncTransferControllerROS_RunFlag = true;
        _thKIROSyncTransferControllerROS = std::unique_ptr<std::thread>(new std::thread(ThreadProc_KIROSyncTransferControllerROS, this));
    }

    ROS_INFO("KIROSyncTransferControllerROS::StartThreadLoop: start threadloop");

    return 0;    
}

int KIROSyncTransferControllerROS::StopThreadLoop()
{
    std::lock_guard<std::mutex> lock(_mtx);

    if(threadProc_KIROSyncTransferControllerROS_RunFlag != false) {
        threadProc_KIROSyncTransferControllerROS_RunFlag = false;

        if(_thKIROSyncTransferControllerROS.get() != nullptr && _thKIROSyncTransferControllerROS->joinable() == true) _thKIROSyncTransferControllerROS->join();
        if(_thKIROSyncTransferControllerROS.get() != nullptr) _thKIROSyncTransferControllerROS.reset();
    }

    ROS_INFO("KIROSyncTransferControllerROS::StopThreadLoop: stop threadloop");

    return 0;    
}

/////////////////////////////////////////////////////////////////////////////////////////////////
///// ROS Interface
void KIROSyncTransferControllerROS::InitPublisher()
{
    //pub_ImageRaw = nh_Pub.advertise<rgbdepth_camera::ImageRaw>("rgbdepth_camera/image_raw", 100, std::bind(publishHandler_ImageRaw, std::placeholders::_1));
    //pub_ImageDepth = nh_Pub.advertise<rgbdepth_camera::ImageDepth>("rgbdepth_camera/image_depth", 100, std::bind(publishHandler_ImageDepth, std::placeholders::_1));
    //pub_ImageIR = nh_Pub.advertise<rgbdepth_camera::ImageIR>("rgbdepth_camera/image_ir", 100, std::bind(publishHandler_ImageIR, std::placeholders::_1));

    pub_ImageRSColor = it_.advertise("osr/image_color", 1, std::bind(publishHandler_ImageRSColor, std::placeholders::_1));
    pub_ImageRSDepth = it_.advertise("osr/image_depth", 1, std::bind(publishHandler_ImageRSDepth, std::placeholders::_1));
    pub_ImageRSAlignedDepth = it_.advertise("osr/image_aligned_depth", 1, std::bind(publishHandler_ImageRSAlignedDepth, std::placeholders::_1));
    pub_ImageRSIR = it_.advertise("osr/image_ir", 1, std::bind(publishHandler_ImageRSIR, std::placeholders::_1));
    pub_ImageNV1 = it_.advertise("osr/image_nv1", 1, std::bind(publishHandler_ImageNV1, std::placeholders::_1));
    pub_ImageThermal = it_.advertise("osr/image_thermal", 1, std::bind(publishHandler_ImageThermal, std::placeholders::_1));
    pub_ImageGS = it_.advertise("osr/image_gs", 1, std::bind(publishHandler_ImageGS, std::placeholders::_1));

    pub_ImageRSColorCameraInfo = nh_Pub.advertise<sensor_msgs::CameraInfo>("osr/image_color_camerainfo", 1, std::bind(publishHandler_ImageRSColorCameraInfo, std::placeholders::_1));
    pub_ImageRSIRCameraInfo = nh_Pub.advertise<sensor_msgs::CameraInfo>("osr/image_ir_camerainfo", 1, std::bind(publishHandler_ImageRSIRCameraInfo, std::placeholders::_1));
    pub_ImageRSDepthCameraInfo = nh_Pub.advertise<sensor_msgs::CameraInfo>("osr/image_depth_camerainfo", 1, std::bind(publishHandler_ImageRSDepthCameraInfo, std::placeholders::_1));

    pub_PointCloud = nh_Pub.advertise<sensor_msgs::PointCloud2>("osr/lidar_pointcloud", 1, std::bind(publishHandler_PointCloud, std::placeholders::_1));

    pub_RobotOdometry = nh_Pub.advertise<nav_msgs::Odometry>("osr/robot_odometry", 10, std::bind(publishHandler_RobotOdometry, std::placeholders::_1));

    if(!pub_ImageRSColor || !pub_ImageRSDepth || !pub_ImageRSAlignedDepth || !pub_ImageRSIR || !pub_ImageNV1 || !pub_ImageThermal || !pub_ImageGS || 
        !pub_ImageRSColorCameraInfo || !pub_ImageRSIRCameraInfo || !pub_ImageRSDepthCameraInfo || !pub_PointCloud || !pub_RobotOdometry) 
    {
        ROS_INFO("KIROSyncTransferControllerROS::InitPublisher: publish handle is invalid!");
    }

    ///// disable unused topic
    std::string command = "";
    command = "rosparam set /osr/image_aligned_depth/disable_pub_plugins \"['image_transport/compressed', 'image_transport/compressedDepth', 'image_transport/theora']\"";
    system(command.c_str());

    command = "rosparam set /osr/image_color/disable_pub_plugins \"['image_transport/compressed', 'image_transport/compressedDepth', 'image_transport/theora']\"";
    system(command.c_str());

    command = "rosparam set /osr/image_depth/disable_pub_plugins \"['image_transport/compressed', 'image_transport/compressedDepth', 'image_transport/theora']\"";
    system(command.c_str());

    command = "rosparam set /osr/image_ir/disable_pub_plugins \"['image_transport/compressed', 'image_transport/compressedDepth', 'image_transport/theora']\"";
    system(command.c_str());

    command = "rosparam set /osr/image_nv1/disable_pub_plugins \"['image_transport/compressed', 'image_transport/compressedDepth', 'image_transport/theora']\"";
    system(command.c_str());

    command = "rosparam set /osr/image_thermal/disable_pub_plugins \"['image_transport/compressed', 'image_transport/compressedDepth', 'image_transport/theora']\"";
    system(command.c_str());

    command = "rosparam set /osr/image_gs/disable_pub_plugins \"['image_transport/compressed', 'image_transport/compressedDepth', 'image_transport/theora']\"";
    system(command.c_str());

    ROS_INFO("KIROSyncTransferControllerROS::InitPublisher: init publisher");
}

void KIROSyncTransferControllerROS::InitSubscriber()
{
    sub_Odometry = nh_Sub.subscribe("robot_odom", 100, &subscribeHandler_Odometry);

    if(!sub_Odometry) {
        ROS_INFO("KIROSyncTransferControllerROS::InitSubscriber: subscriber handle is invalid!");
    }

    ROS_INFO("KIROSyncTransferControllerROS::InitSubscriber: init subscriber");
}

void KIROSyncTransferControllerROS::InitServiceProvider()
{
    srv_Initialize = nh_Service.advertiseService("KIROSyncTransfer_Initialize", serviceHandler_Initialize);
    srv_Finalize = nh_Service.advertiseService("KIROSyncTransfer_Finalize", serviceHandler_Finalize);
    srv_Run = nh_Service.advertiseService("KIROSyncTransfer_Run", serviceHandler_Run);
    srv_Stop = nh_Service.advertiseService("KIROSyncTransfer_Stop", serviceHandler_Stop);
    srv_SyncAllData = nh_Service.advertiseService("KIROSyncTransfer_SyncAllData", serviceHandler_SyncAllData);

    if(!srv_Initialize || !srv_Finalize || !srv_Run || !srv_Stop || !srv_SyncAllData)
    {
        ROS_INFO("KIROSyncTransferControllerROS::InitServiceProvider: serviceserver handle is invalid!");
    }

    ROS_INFO("KIROSyncTransferControllerROS::InitServiceProvider: init service provider");
}

void KIROSyncTransferControllerROS::InitServiceClient()
{
    // empty
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
///// publish handler (at connect)
void KIROSyncTransferControllerROS::publishHandler_ImageRSColor(const image_transport::SingleSubscriberPublisher& pub)
{
    ROS_INFO("KIROSyncTransferControllerROS::publishHandler_ImageRSColor: connect from %s", pub.getSubscriberName().c_str());
}

void KIROSyncTransferControllerROS::publishHandler_ImageRSDepth(const image_transport::SingleSubscriberPublisher& pub)
{
    ROS_INFO("KIROSyncTransferControllerROS::publishHandler_ImageRSDepth: connect from %s", pub.getSubscriberName().c_str());
}

void KIROSyncTransferControllerROS::publishHandler_ImageRSAlignedDepth(const image_transport::SingleSubscriberPublisher& pub)
{
    ROS_INFO("KIROSyncTransferControllerROS::publishHandler_ImageRSAlignedDepth: connect from %s", pub.getSubscriberName().c_str());
}

void KIROSyncTransferControllerROS::publishHandler_ImageRSIR(const image_transport::SingleSubscriberPublisher& pub)
{
    ROS_INFO("KIROSyncTransferControllerROS::publishHandler_ImageRSIR: connect from %s", pub.getSubscriberName().c_str());
}

void KIROSyncTransferControllerROS::publishHandler_ImageNV1(const image_transport::SingleSubscriberPublisher& pub)
{
    ROS_INFO("KIROSyncTransferControllerROS::publishHandler_ImageNV1: connect from %s", pub.getSubscriberName().c_str());
}

void KIROSyncTransferControllerROS::publishHandler_ImageThermal(const image_transport::SingleSubscriberPublisher& pub)
{
    ROS_INFO("KIROSyncTransferControllerROS::publishHandler_ImageThermal: connect from %s", pub.getSubscriberName().c_str());
}

void KIROSyncTransferControllerROS::publishHandler_ImageGS(const image_transport::SingleSubscriberPublisher& pub)
{
    ROS_INFO("KIROSyncTransferControllerROS::publishHandler_ImageGS: connect from %s", pub.getSubscriberName().c_str());
}

void KIROSyncTransferControllerROS::publishHandler_ImageRSColorCameraInfo(const ros::SingleSubscriberPublisher& pub)
{
    ROS_INFO("KIROSyncTransferControllerROS::publishHandler_ImageRSColorCameraInfo: connect from %s", pub.getSubscriberName().c_str());
}

void KIROSyncTransferControllerROS::publishHandler_ImageRSIRCameraInfo(const ros::SingleSubscriberPublisher& pub)
{
    ROS_INFO("KIROSyncTransferControllerROS::publishHandler_ImageRSIRCameraInfo: connect from %s", pub.getSubscriberName().c_str());
}

void KIROSyncTransferControllerROS::publishHandler_ImageRSDepthCameraInfo(const ros::SingleSubscriberPublisher& pub)
{
    ROS_INFO("KIROSyncTransferControllerROS::publishHandler_ImageRSDepthCameraInfo: connect from %s", pub.getSubscriberName().c_str());
}

void KIROSyncTransferControllerROS::publishHandler_PointCloud(const ros::SingleSubscriberPublisher& pub)
{
    ROS_INFO("KIROSyncTransferControllerROS::publishHandler_PointCloud: connect from %s", pub.getSubscriberName().c_str());
}

void KIROSyncTransferControllerROS::publishHandler_RobotOdometry(const ros::SingleSubscriberPublisher& pub)
{
    ROS_INFO("KIROSyncTransferControllerROS::publishHandler_RobotOdometry: connect from %s", pub.getSubscriberName().c_str());
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
///// subscribe handler 
void KIROSyncTransferControllerROS::subscribeHandler_Odometry(const nav_msgs::Odometry &msg)
{
    ros::Time timestamp = msg.header.stamp;

    OdometryInfo odometry_info;
    odometry_info.seq = msg.header.seq;
    odometry_info.frame_id = msg.header.frame_id;
    odometry_info.child_frame_id = msg.child_frame_id;
    odometry_info.position_x = msg.pose.pose.position.x;
    odometry_info.position_y = msg.pose.pose.position.y;
    odometry_info.position_z = msg.pose.pose.position.z;
    odometry_info.orientation_x = msg.pose.pose.orientation.x;
    odometry_info.orientation_y = msg.pose.pose.orientation.y;
    odometry_info.orientation_z = msg.pose.pose.orientation.z;
    odometry_info.orientation_w = msg.pose.pose.orientation.w;
    odometry_info.linear_velocity_x = msg.twist.twist.linear.x;
    odometry_info.linear_velocity_y = msg.twist.twist.linear.y;
    odometry_info.angular_velocity_z = msg.twist.twist.angular.z;

    KIROSyncTransfer::getModel()->SetData_OdometryInfo(odometry_info, (unsigned long long)timestamp.toNSec());
}

/////////////////////////////////////////////////////////////////////////////////////////////
///// service handler 
bool KIROSyncTransferControllerROS::serviceHandler_Initialize(kiro_sync_transfer::Initialize::Request &req, kiro_sync_transfer::Initialize::Response &res)
{
    int flag = req.flag;

    if(flag == 1) {
        res.result = (KIROSyncTransfer::getControllerROS()->Call_Initialize() == true) ? 1 : 0;
        return true;
    }
    else {
        return false;
    }
}

bool KIROSyncTransferControllerROS::serviceHandler_Finalize(kiro_sync_transfer::Finalize::Request &req, kiro_sync_transfer::Finalize::Response &res)
{
    int flag = req.flag;

    if(flag == 1) {
        res.result = (KIROSyncTransfer::getControllerROS()->Call_Finalize() == true) ? 1 : 0;
        return true;
    }
    else {
        return false;
    }
}

bool KIROSyncTransferControllerROS::serviceHandler_Run(kiro_sync_transfer::Run::Request &req, kiro_sync_transfer::Run::Response &res)
{
    int flag = req.flag;

    if(flag == 1) {
        res.result = (KIROSyncTransfer::getControllerROS()->Call_Run() == true) ? 1 : 0;
        return true;
    }
    else {
        return false;
    }
}

bool KIROSyncTransferControllerROS::serviceHandler_Stop(kiro_sync_transfer::Stop::Request &req, kiro_sync_transfer::Stop::Response &res)
{
    int flag = req.flag;

    if(flag == 1) {
        res.result = (KIROSyncTransfer::getControllerROS()->Call_Stop() == true) ? 1 : 0;
        return true;
    }
    else {
        return false;
    }
}

bool KIROSyncTransferControllerROS::serviceHandler_SyncAllData(kiro_sync_transfer::SyncAllData::Request &req, kiro_sync_transfer::SyncAllData::Response &res)
{
    unsigned long long dummy_timestamp;

    int req_val = req.req_val;

    res.timestamp = ros::Time::now();

    struct OdometryInfo odometry_info;
    KIROSyncTransfer::getModel()->GetData_OdometryInfo(odometry_info, dummy_timestamp);

    res.odom.header.seq = odometry_info.seq;
    res.odom.header.stamp = res.timestamp;
    res.odom.header.frame_id = odometry_info.frame_id;
    res.odom.child_frame_id = odometry_info.child_frame_id;
    res.odom.pose.pose.position.x = odometry_info.position_x;
    res.odom.pose.pose.position.y = odometry_info.position_y;
    res.odom.pose.pose.position.z = odometry_info.position_z;
    res.odom.pose.pose.orientation.x = odometry_info.orientation_x;
    res.odom.twist.twist.linear.x = odometry_info.linear_velocity_x;
    res.odom.twist.twist.linear.y = odometry_info.linear_velocity_y;
    res.odom.twist.twist.angular.z = odometry_info.angular_velocity_z;

    int img_rs_width = KIROSyncTransfer::getModel()->GetImageRSWidth();
    int img_rs_height = KIROSyncTransfer::getModel()->GetImageRSHeight();
    int img_rs_color_pixelbytes = KIROSyncTransfer::getModel()->GetRSColorPixelBytes();
    int img_rs_depth_pixelbytes = KIROSyncTransfer::getModel()->GetRSDepthPixelBytes();
    int img_rs_ir_pixelbytes = KIROSyncTransfer::getModel()->GetRSIRPixelBytes();

    cv::Mat imgColor(cv::Size(img_rs_width, img_rs_height), CV_8UC3);
    cv::Mat imgDepth(cv::Size(img_rs_width, img_rs_height), CV_16UC1);
    cv::Mat imgAlignedDepth(cv::Size(img_rs_width, img_rs_height), CV_16UC1);
    cv::Mat imgIR(cv::Size(img_rs_width, img_rs_height), CV_8UC1);    

    int img_nv_width = KIROSyncTransfer::getModel()->GetImageNVWidth();
    int img_nv_height = KIROSyncTransfer::getModel()->GetImageNVHeight();
    int img_nv_pixelbytes = KIROSyncTransfer::getModel()->GetImageNVPixelBytes();

    cv::Mat imgNV1(cv::Size(img_nv_width, img_nv_height), CV_8UC3);

    int img_thermal_width = KIROSyncTransfer::getModel()->GetImageThermalWidth();
    int img_thermal_height = KIROSyncTransfer::getModel()->GetImageThermalHeight();
    int img_thermal_pixelbytes = KIROSyncTransfer::getModel()->GetImageThermalPixelBytes();

    cv::Mat imgThermal(cv::Size(img_thermal_width, img_thermal_height), CV_16UC1);

    int img_resize_width = KIROSyncTransfer::getModel()->GetImageResizeWidth();
    int img_resize_height = KIROSyncTransfer::getModel()->GetImageResizeHeight();

    cv::Mat imgResizeColor(cv::Size(img_resize_width, img_resize_height), CV_8UC3);
    cv::Mat imgResizeDepth(cv::Size(img_resize_width, img_resize_height), CV_16UC1);
    cv::Mat imgResizeAlignedDepth(cv::Size(img_resize_width, img_resize_height), CV_16UC1);
    cv::Mat imgResizeIR(cv::Size(img_resize_width, img_resize_height), CV_8UC1);
    cv::Mat imgResizeNV1(cv::Size(img_resize_width, img_resize_height), CV_8UC3);
    cv::Mat imgResizeThermal(cv::Size(img_resize_width, img_resize_height), CV_16UC1);

    struct CameraInfo rs_color_camerainfo, rs_ir_camerainfo, rs_depth_camerainfo;

    unsigned int numPointCloud = KIROSyncTransfer::getModel()->GetNumPointCloud();
    std::shared_ptr<ptcloud_t> ptcloud_data = std::shared_ptr<ptcloud_t>(new ptcloud_t[numPointCloud]);

    KIROSyncTransfer::getModel()->GetData_Synchronized((char*&)(imgColor.data), (char*&)(imgDepth.data), 
                                                        (char*&)(imgAlignedDepth.data), (char*&)(imgIR.data),
                                                        rs_color_camerainfo, rs_ir_camerainfo,
                                                        (char*&)(imgNV1.data), (char*&)(imgThermal.data), 
                                                        (ptcloud_t*)(ptcloud_data.get()), dummy_timestamp);

    cv::resize(imgColor, imgResizeColor, cv::Size(img_resize_width, img_resize_height), 0, 0, CV_INTER_CUBIC);
    cv::resize(imgDepth, imgResizeDepth, cv::Size(img_resize_width, img_resize_height), 0, 0, CV_INTER_CUBIC);
    cv::resize(imgAlignedDepth, imgResizeAlignedDepth, cv::Size(img_resize_width, img_resize_height), 0, 0, CV_INTER_CUBIC);
    cv::resize(imgIR, imgResizeIR, cv::Size(img_resize_width, img_resize_height), 0, 0, CV_INTER_CUBIC);
    cv::resize(imgNV1, imgResizeNV1, cv::Size(img_resize_width, img_resize_height), 0, 0, CV_INTER_CUBIC);
    cv::resize(imgThermal, imgResizeThermal, cv::Size(img_resize_width, img_resize_height), 0, 0, CV_INTER_CUBIC);

    res.RSColor.header.seq = 0;
    res.RSColor.header.stamp = res.timestamp;
    res.RSColor.header.frame_id = "0";
    res.RSColor.height = img_resize_height;
    res.RSColor.width = img_resize_width;
    res.RSColor.encoding = "bgr8";
    res.RSColor.is_bigendian = false;
    res.RSColor.step = img_resize_width * img_rs_color_pixelbytes;
    res.RSColor.data.resize(img_resize_height * img_resize_width * img_rs_color_pixelbytes);
    std::memcpy((void*)&res.RSColor.data[0], (const void*)imgResizeColor.data, (img_resize_height * img_resize_width * img_rs_color_pixelbytes));

    res.RSDepth.header.seq = 0;
    res.RSDepth.header.stamp = res.timestamp;
    res.RSDepth.header.frame_id = "0";
    res.RSDepth.height = img_resize_height;
    res.RSDepth.width = img_resize_width;
    res.RSDepth.encoding = "mono16";
    res.RSDepth.is_bigendian = false;
    res.RSDepth.step = img_resize_width * img_rs_depth_pixelbytes;
    res.RSDepth.data.resize(img_resize_height * img_resize_width * img_rs_depth_pixelbytes);
    std::memcpy((void*)&res.RSDepth.data[0], (const void*)imgResizeDepth.data, (img_resize_height * img_resize_width * img_rs_depth_pixelbytes));

    res.RSAlignedDepth.header.seq = 0;
    res.RSAlignedDepth.header.stamp = res.timestamp;
    res.RSAlignedDepth.header.frame_id = "0";
    res.RSAlignedDepth.height = img_resize_height;
    res.RSAlignedDepth.width = img_resize_width;
    res.RSAlignedDepth.encoding = "mono16";
    res.RSAlignedDepth.is_bigendian = false;
    res.RSAlignedDepth.step = img_resize_width * img_rs_depth_pixelbytes;
    res.RSAlignedDepth.data.resize(img_resize_height * img_resize_width * img_rs_depth_pixelbytes);
    std::memcpy((void*)&res.RSAlignedDepth.data[0], (const void*)imgResizeAlignedDepth.data, (img_resize_height * img_resize_width * img_rs_depth_pixelbytes));

    res.RSIR.header.seq = 0;
    res.RSIR.header.stamp = res.timestamp;
    res.RSIR.header.frame_id = "0";
    res.RSIR.height = img_resize_height;
    res.RSIR.width = img_resize_width;
    res.RSIR.encoding = "mono8";
    res.RSIR.is_bigendian = false;
    res.RSIR.step = img_resize_width * img_rs_ir_pixelbytes;
    res.RSIR.data.resize(img_resize_height * img_resize_width * img_rs_ir_pixelbytes);
    std::memcpy((void*)&res.RSIR.data[0], (const void*)imgResizeIR.data, (img_resize_height * img_resize_width * img_rs_ir_pixelbytes));

    res.RSCameraInfo_Color.header.seq = 0;
    res.RSCameraInfo_Color.header.stamp = res.timestamp;
    res.RSCameraInfo_Color.header.frame_id = "0";
    res.RSCameraInfo_Color.width = rs_color_camerainfo.width;
    res.RSCameraInfo_Color.height = rs_color_camerainfo.height;
    res.RSCameraInfo_Color.distortion_model = "plumb_bob";
    for(int i=0; i<9; i++) res.RSCameraInfo_Color.K.at(i) = rs_color_camerainfo.K[i];
    for(int i=0; i<12; i++) res.RSCameraInfo_Color.P.at(i) = rs_color_camerainfo.P[i];
    for(int i=0; i<9; i++) res.RSCameraInfo_Color.R.at(i) = rs_color_camerainfo.R[i];
    res.RSCameraInfo_Color.D.resize(5);
    for(int i=0; i<5; i++) res.RSCameraInfo_Color.D.at(i) = rs_color_camerainfo.D[i];

    res.RSCameraInfo_IR.header.seq = 0;
    res.RSCameraInfo_IR.header.stamp = res.timestamp;
    res.RSCameraInfo_IR.header.frame_id = "0";
    res.RSCameraInfo_IR.width = rs_ir_camerainfo.width;
    res.RSCameraInfo_IR.height = rs_ir_camerainfo.height;
    res.RSCameraInfo_IR.distortion_model = "plumb_bob";
    for(int i=0; i<9; i++) res.RSCameraInfo_IR.K.at(i) = rs_ir_camerainfo.K[i];
    for(int i=0; i<12; i++) res.RSCameraInfo_IR.P.at(i) = rs_ir_camerainfo.P[i];
    for(int i=0; i<9; i++) res.RSCameraInfo_IR.R.at(i) = rs_ir_camerainfo.R[i];
    res.RSCameraInfo_IR.D.resize(5);
    for(int i=0; i<5; i++) res.RSCameraInfo_IR.D.at(i) = rs_ir_camerainfo.D[i];

    res.NV1.header.seq = 0;
    res.NV1.header.stamp = res.timestamp;
    res.NV1.header.frame_id = "0";
    res.NV1.height = img_resize_height;
    res.NV1.width = img_resize_width;
    res.NV1.encoding = "bgr8";
    res.NV1.is_bigendian = false;
    res.NV1.step = img_resize_width * img_nv_pixelbytes;
    res.NV1.data.resize(img_resize_height * img_resize_width * img_nv_pixelbytes);
    std::memcpy((void*)&res.NV1.data[0], (const void*)imgResizeNV1.data, (img_resize_height * img_resize_width * img_nv_pixelbytes));

    res.Thermal.header.seq = 0;
    res.Thermal.header.stamp = res.timestamp;
    res.Thermal.header.frame_id = "0";
    res.Thermal.height = img_resize_height;
    res.Thermal.width = img_resize_width;
    res.Thermal.encoding = "mono16";
    res.Thermal.is_bigendian = false;
    res.Thermal.step = img_resize_width * img_thermal_pixelbytes;
    res.Thermal.data.resize(img_resize_height * img_resize_width * img_thermal_pixelbytes);
    std::memcpy((void*)&res.Thermal.data[0], (const void*)imgResizeThermal.data, (img_resize_height * img_resize_width * img_thermal_pixelbytes));

    res.LiDAR.header.seq = 0;
    res.LiDAR.header.stamp = res.timestamp;
    res.LiDAR.header.frame_id = "map";
    res.LiDAR.fields.resize(4);
    res.LiDAR.fields[0].name = "x";
    res.LiDAR.fields[0].offset = 0;
    res.LiDAR.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    res.LiDAR.fields[0].count = 1;
    res.LiDAR.fields[1].name = "y";
    res.LiDAR.fields[1].offset = 4;
    res.LiDAR.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    res.LiDAR.fields[1].count = 1;
    res.LiDAR.fields[2].name = "z";
    res.LiDAR.fields[2].offset = 8;
    res.LiDAR.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    res.LiDAR.fields[2].count = 1;
    res.LiDAR.fields[3].name = "intensity";
    res.LiDAR.fields[3].offset = 12;
    res.LiDAR.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
    res.LiDAR.fields[3].count = 1;
    res.LiDAR.data.resize(std::max((size_t)1, (size_t)numPointCloud) * sizeof(ptcloud_t), 0);
    res.LiDAR.point_step = sizeof(ptcloud_t);
    res.LiDAR.row_step = res.LiDAR.data.size();
    res.LiDAR.height = 1;
    res.LiDAR.width = res.LiDAR.row_step / sizeof(ptcloud_t);
    res.LiDAR.is_bigendian = false;
    res.LiDAR.is_dense = true;

    uint8_t *ptr = res.LiDAR.data.data();
    for(size_t i=0; i < numPointCloud; i++) {
        *(reinterpret_cast<float*>(ptr + 0)) = ptcloud_data.get()[i].at(0);
        *(reinterpret_cast<float*>(ptr + 4)) = ptcloud_data.get()[i].at(1);
        *(reinterpret_cast<float*>(ptr + 8)) = ptcloud_data.get()[i].at(2);
        *(reinterpret_cast<float*>(ptr + 12)) = ptcloud_data.get()[i].at(3);
        ptr += sizeof(ptcloud_t);
    }

    return true;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////
// macro functions
bool KIROSyncTransferControllerROS::Call_Initialize()
{
    bool retCall = true;

    _ROSState = 1;  // ROS Initialize

    // model init.
    if(KIROSyncTransfer::getModel().get() != nullptr) {
        KIROSyncTransfer::getModel()->Initialize();
        ROS_INFO("KIROSyncTransferControllerROS::Call_Initialize: call model initialize()");
        retCall = true;
    }
    else {
        ROS_INFO("KIROSyncTransferControllerROS::Call_Initialize: model ptr is nullptr");
        _ROSState = 19; // fail to transite ROS Initialize
        retCall = false;
    }

    // controllerROS init.
    if(KIROSyncTransfer::getControllerROS().get() != nullptr) {
        KIROSyncTransfer::getControllerROS()->Initialize();
        ROS_INFO("KIROSyncTransferControllerROS::Call_Initialize: call controllerROS initialize()");
        retCall = true;
    }
    else {
        ROS_INFO("KIROSyncTransferControllerROS::Call_Initialize: controller ptr is nullptr");
        _ROSState = 19; // fail to transite ROS Initialize
        retCall = false;
    }

    return retCall;
}

bool KIROSyncTransferControllerROS::Call_Finalize()
{
    bool retCall = true;

    _ROSState = 4;  // ROS Finalize

    // stop publisher
    if(KIROSyncTransfer::getControllerROS().get() != nullptr) {
        KIROSyncTransfer::getControllerROS()->SetFlag_PublishRunning(false);
        ROS_INFO("KIROSyncTransferControllerROS::Call_Finalize: set false to publish flag");
        retCall = true;
    }
    else {
        ROS_INFO("KIROSyncTransferControllerROS::Call_Finalize: controller ptr is nullptr");
        _ROSState = 49; // fail to transite ROS Finalize
        retCall = false;
    }

    // controllerROS finalize
    if(KIROSyncTransfer::getControllerROS().get() != nullptr) {
        KIROSyncTransfer::getControllerROS()->Finalize();
        ROS_INFO("KIROSyncTransferControllerROS::Call_Finalize: call controllerROS finalzie()");
        retCall = true;
    }
    else {
        ROS_INFO("KIROSyncTransferControllerROS::Call_Finalize: controller ptr is nullptr");
        _ROSState = 49; // fail to transite ROS Finalize
        retCall = false;
    }

    // model finalize
    if(KIROSyncTransfer::getModel().get() != nullptr) {
        KIROSyncTransfer::getModel()->Finalize();
        ROS_INFO("KIROSyncTransferControllerROS::Call_Finalize: call model finalize()");
        retCall = true;
    }
    else {
        ROS_INFO("KIROSyncTransferControllerROS::Call_Finalize: model ptr is nullptr");
        _ROSState = 49; // fail to transite ROS Finalize
        retCall = false;
    }

    return retCall;
}

bool KIROSyncTransferControllerROS::Call_Run()
{
    bool retCall = true;

    _ROSState = 2;  // ROS Running

    // run model threadloop
    if(KIROSyncTransfer::getModel().get() != nullptr) {
        KIROSyncTransfer::getModel()->StartThreadLoop();
        ROS_INFO("KIROSyncTransferControllerROS::Call_Run: run model threadloop");
        retCall = true;
    }
    else {
        ROS_INFO("KIROSyncTransferControllerROS::Call_Run: model ptr is nullptr");
        _ROSState = 29; // fail to transite ROS Running
        retCall = false;
    }

    // run publisher
    if(KIROSyncTransfer::getControllerROS().get() != nullptr) {
        KIROSyncTransfer::getControllerROS()->SetFlag_PublishRunning(true);
        ROS_INFO("KIROSyncTransferControllerROS::Call_Run: set true to publish flag");
        retCall = true;
    }
    else {
        ROS_INFO("KIROSyncTransferControllerROS::Call_Run: controller ptr is nullptr");
        _ROSState = 29; // fail to transite ROS Running
        retCall = false;
    }

    // run controllerROS threadloop
    if(KIROSyncTransfer::getControllerROS().get() != nullptr) {
        KIROSyncTransfer::getControllerROS()->StartThreadLoop();
        ROS_INFO("KIROSyncTransferControllerROS::Call_Run: run controllerROS threadloop");
        retCall = true;
    }
    else {
        ROS_INFO("KIROSyncTransferControllerROS::Call_Run: controller ptr is nullptr");
        _ROSState = 29; // fail to transite ROS Running
        retCall = false;
    }

    return retCall;
}

bool KIROSyncTransferControllerROS::Call_Stop()
{
    bool retCall = true;

    _ROSState = 3;  // ROS Stop

    // stop publisher
    if(KIROSyncTransfer::getControllerROS().get() != nullptr) {
        KIROSyncTransfer::getControllerROS()->SetFlag_PublishRunning(false);
        ROS_INFO("KIROSyncTransferControllerROS::Call_Stop: set false to publish flag");
        retCall = true;
    }
    else {
        ROS_INFO("KIROSyncTransferControllerROS::Call_Stop: controller ptr is nullptr");
        _ROSState = 39; // fail to transite ROS Stop
        retCall = false;
    }

    // stop controllerROS threadloop
    if(KIROSyncTransfer::getControllerROS().get() != nullptr) {
        KIROSyncTransfer::getControllerROS()->StopThreadLoop();
        ROS_INFO("KIROSyncTransferControllerROS::Call_Stop: stop controllerROS threadloop");
        retCall = true;
    }
    else {
        ROS_INFO("KIROSyncTransferControllerROS::Call_Stop: controller ptr is nullptr");
        _ROSState = 39; // fail to transite ROS Stop
        retCall = false;
    }

    // stop model threadloop
    if(KIROSyncTransfer::getModel().get() != nullptr) {
        KIROSyncTransfer::getModel()->StopThreadLoop();
        ROS_INFO("KIROSyncTransferControllerROS::Call_Stop: stop model threadloop");
        retCall = true;
    }
    else {
        ROS_INFO("KIROSyncTransferControllerROS::Call_Stop: stop model's threadloop");
        _ROSState = 39; // fail to transite ROS Stop
        retCall = false;
    }

    return retCall;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Thread Loop 
void KIROSyncTransferControllerROS::ThreadProc_KIROSyncTransferControllerROS(void *arg)
{
    #define CHECK_KIROSYNCTRANSFERCONTROLLERROS_THREADLOOP   if(pKIROSyncTransferControllerROS == nullptr || pKIROSyncTransferControllerROS->threadProc_KIROSyncTransferControllerROS_RunFlag != true) break;
    #define CHECK_KIROSYNCTRANSFERCONTROLLERROS_CONDITION    (pKIROSyncTransferControllerROS != nullptr && pKIROSyncTransferControllerROS->threadProc_KIROSyncTransferControllerROS_RunFlag == true)

    KIROSyncTransferControllerROS *pKIROSyncTransferControllerROS = (KIROSyncTransferControllerROS*)arg;

	//kiro_camera::ImageColor msg_ImageColor;
    //kiro_camera::ImageDepth msg_ImageDepth;
    //kiro_camera::ImageDepth msg_ImageAlignedDepth;
    //kiro_camera::ImageIR msg_ImageIR;
    //kiro_camera::ImageNV msg_ImageNV1;
    //kiro_camera::ImageNV msg_ImageNV2;
    //kiro_camera::ImageThermal msg_ImageThermal;
    //kiro_camera::ImageGS msg_ImageGS;

    unsigned int imageColor_SeqID = 0;
    unsigned int imageDepth_SeqID = 0;
    unsigned int imageAlignedDepth_SeqID = 0;
    unsigned int imageIR_SeqID = 0;
    unsigned int imageNV1_SeqID = 0;
    //unsigned int imageNV2_SeqID = 0;
    unsigned int imageThermal_SeqID = 0;
    unsigned int imageGS_SeqID = 0;
    unsigned int RSColorCameraInfo_seqID = 0;
    unsigned int RSIRCameraInfo_seqID = 0;
    unsigned int RSDepthCameraInfo_seqID = 0;
    unsigned int PointCloud_seqID = 0;

    assert(KIROSyncTransfer::getModel().get() != nullptr && "KIROSyncTransferControllerROS::ThreadProc: getModel().get() is nullptr");
    assert(KIROSyncTransfer::getControllerROS().get() != nullptr && "KIROSyncTransferControllerROS::ThreadProc: getControllerROS().get() is nullptr");

    int img_rs_width = KIROSyncTransfer::getModel()->GetImageRSWidth();
    int img_rs_height = KIROSyncTransfer::getModel()->GetImageRSHeight();

    cv::Mat imgColor(cv::Size(img_rs_width, img_rs_height), CV_8UC3);
    cv::Mat imgDepth(cv::Size(img_rs_width, img_rs_height), CV_16UC1);
    cv::Mat imgAlignedDepth(cv::Size(img_rs_width, img_rs_height), CV_16UC1);
    cv::Mat imgIR(cv::Size(img_rs_width, img_rs_height), CV_8UC1);    

    int img_nv_width = KIROSyncTransfer::getModel()->GetImageNVWidth();
    int img_nv_height = KIROSyncTransfer::getModel()->GetImageNVHeight();

    cv::Mat imgNV1(cv::Size(img_nv_width, img_nv_height), CV_8UC3);
    cv::Mat imgNV2(cv::Size(img_nv_width, img_nv_height), CV_8UC3);

    int img_thermal_width = KIROSyncTransfer::getModel()->GetImageThermalWidth();
    int img_thermal_height = KIROSyncTransfer::getModel()->GetImageThermalHeight();

    cv::Mat imgThermal(cv::Size(img_thermal_width, img_thermal_height), CV_16UC1);

    int img_gs_width = KIROSyncTransfer::getModel()->GetImageGSWidth();
    int img_gs_height = KIROSyncTransfer::getModel()->GetImageGSHeight();

    cv::Mat imgGS(cv::Size(img_gs_width, img_gs_height), CV_8UC3);

    int img_resize_width = KIROSyncTransfer::getModel()->GetImageResizeWidth();
    int img_resize_height = KIROSyncTransfer::getModel()->GetImageResizeHeight();

    cv::Mat imgResizeColor(cv::Size(img_resize_width, img_resize_height), CV_8UC3);
    cv::Mat imgResizeDepth(cv::Size(img_resize_width, img_resize_height), CV_16UC1);
    cv::Mat imgResizeAlignedDepth(cv::Size(img_resize_width, img_resize_height), CV_16UC1);
    cv::Mat imgResizeIR(cv::Size(img_resize_width, img_resize_height), CV_8UC1);
    cv::Mat imgResizeNV1(cv::Size(img_resize_width, img_resize_height), CV_8UC3);
    cv::Mat imgResizeThermal(cv::Size(img_resize_width, img_resize_height), CV_16UC1);
    cv::Mat imgResizeGS(cv::Size(img_resize_width, img_resize_height), CV_8UC3);

    struct CameraInfo rs_color_camerainfo, rs_ir_camerainfo, rs_depth_camerainfo;

    unsigned int numPointCloud = KIROSyncTransfer::getModel()->GetNumPointCloud();
    std::shared_ptr<ptcloud_t> ptcloud_data = std::shared_ptr<ptcloud_t>(new ptcloud_t[numPointCloud]);

    struct OdometryInfo odometryinfo;

    unsigned long long color_timestamp = 0;
    unsigned long long depth_timestamp = 0;
    unsigned long long aligned_depth_timestamp = 0;
    unsigned long long ir_timestamp = 0;
    unsigned long long intrinsic_timestamp = 0;
    unsigned long long nv1_timestamp = 0;
    //unsigned long long nv2_timestamp = 0;
    unsigned long long thermal_timestamp = 0;
    unsigned long long gs_timestamp = 0;
    unsigned long long pointcloud_data_timestamp = 0;
    unsigned long long odometryinfo_timestamp = 0;

    uint32_t timestamp_sec = 0;
    uint32_t timestamp_usec = 0;

    // Loop at 10Hz until the node is shutdown
    ros::Rate rate(10);

    sensor_msgs::ImagePtr msg_imgColor;
    sensor_msgs::ImagePtr msg_imgDepth;
    sensor_msgs::ImagePtr msg_imgAlignedDepth;
    sensor_msgs::ImagePtr msg_imgIR;
    sensor_msgs::ImagePtr msg_imgNV1;
    sensor_msgs::ImagePtr msg_imgThermal;
    sensor_msgs::ImagePtr msg_imgGS;
    sensor_msgs::CameraInfo msg_RSColorCameraInfo;
    sensor_msgs::CameraInfo msg_RSIRCameraInfo;
    sensor_msgs::CameraInfo msg_RSDepthCameraInfo;
    sensor_msgs::PointCloud2 msg_PointCloud;
    nav_msgs::Odometry msg_RobotOdometry;

    while(CHECK_KIROSYNCTRANSFERCONTROLLERROS_CONDITION && ros::ok())
    {
        // Publish 
        if(KIROSyncTransfer::getModel().get() != nullptr && 
           KIROSyncTransfer::getModel()->isThreadRunning() == true &&
           KIROSyncTransfer::getControllerROS().get() != nullptr &&
           KIROSyncTransfer::getControllerROS()->isThreadRunning() == true &&
           KIROSyncTransfer::getControllerROS()->GetFlag_PublishRunning() == true) 
        {
            auto publish_timestamp = ros::Time::now();

            // ImageColor (RealSense) /////////////////////////////////////////////////////////////////////////////////////////////////
            KIROSyncTransfer::getModel()->GetData_ImageRSColor((char*&)(imgColor.data), color_timestamp);
            timestamp_sec = (uint32_t)(color_timestamp / 1000000);
            timestamp_usec = (uint32_t)(color_timestamp % 1000000);
            CHECK_KIROSYNCTRANSFERCONTROLLERROS_THREADLOOP;

            cv::resize(imgColor, imgResizeColor, cv::Size(img_resize_width, img_resize_height), 0, 0, CV_INTER_CUBIC);
            CHECK_KIROSYNCTRANSFERCONTROLLERROS_THREADLOOP;

            msg_imgColor = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgResizeColor).toImageMsg();
            msg_imgColor->width = KIROSyncTransfer::getModel()->GetImageResizeWidth();
            msg_imgColor->height = KIROSyncTransfer::getModel()->GetImageResizeHeight();
            msg_imgColor->is_bigendian = false;
            msg_imgColor->step = KIROSyncTransfer::getModel()->GetImageResizeWidth() * KIROSyncTransfer::getModel()->GetRSColorPixelBytes();
            msg_imgColor->header.seq = imageColor_SeqID++;
            msg_imgColor->header.stamp = publish_timestamp;
            msg_imgColor->header.frame_id = "0";

            KIROSyncTransfer::getControllerROS()->getPubImageRSColor().publish(msg_imgColor);
            CHECK_KIROSYNCTRANSFERCONTROLLERROS_THREADLOOP;

            // ImageDepth (RealSense) //////////////////////////////////////////////////////////////////////////////////////////////////
            KIROSyncTransfer::getModel()->GetData_ImageRSDepth((char*&)(imgDepth.data), depth_timestamp);
            timestamp_sec = (uint32_t)(depth_timestamp / 1000000);
            timestamp_usec = (uint32_t)(depth_timestamp % 1000000);
            CHECK_KIROSYNCTRANSFERCONTROLLERROS_THREADLOOP;

            cv::resize(imgDepth, imgResizeDepth, cv::Size(img_resize_width, img_resize_height), 0, 0, CV_INTER_CUBIC);
            CHECK_KIROSYNCTRANSFERCONTROLLERROS_THREADLOOP;

            msg_imgDepth = cv_bridge::CvImage(std_msgs::Header(), "mono16", imgResizeDepth).toImageMsg();
            msg_imgDepth->width = KIROSyncTransfer::getModel()->GetImageResizeWidth();
            msg_imgDepth->height = KIROSyncTransfer::getModel()->GetImageResizeHeight();
            msg_imgDepth->is_bigendian = false;
            msg_imgDepth->step = KIROSyncTransfer::getModel()->GetImageResizeWidth() * KIROSyncTransfer::getModel()->GetRSDepthPixelBytes();
            msg_imgDepth->header.seq = imageDepth_SeqID++;
            msg_imgDepth->header.stamp = publish_timestamp;
            msg_imgDepth->header.frame_id = "0";

            KIROSyncTransfer::getControllerROS()->getPubImageRSDepth().publish(msg_imgDepth);
            CHECK_KIROSYNCTRANSFERCONTROLLERROS_THREADLOOP;

            // ImageAlignedDepth (RealSense) ////////////////////////////////////////////////////////////////////////////////////////////
            KIROSyncTransfer::getModel()->GetData_ImageRSAlignedDepth((char*&)(imgAlignedDepth.data), aligned_depth_timestamp);
            timestamp_sec = (uint32_t)(aligned_depth_timestamp / 1000000);
            timestamp_usec = (uint32_t)(aligned_depth_timestamp % 1000000);
            CHECK_KIROSYNCTRANSFERCONTROLLERROS_THREADLOOP;

            cv::resize(imgAlignedDepth, imgResizeAlignedDepth, cv::Size(img_resize_width, img_resize_height), 0, 0, CV_INTER_CUBIC);
            CHECK_KIROSYNCTRANSFERCONTROLLERROS_THREADLOOP;

            msg_imgAlignedDepth = cv_bridge::CvImage(std_msgs::Header(), "mono16", imgResizeAlignedDepth).toImageMsg();
            msg_imgAlignedDepth->width = KIROSyncTransfer::getModel()->GetImageResizeWidth();
            msg_imgAlignedDepth->height = KIROSyncTransfer::getModel()->GetImageResizeHeight();
            msg_imgAlignedDepth->is_bigendian = false;
            msg_imgAlignedDepth->step = KIROSyncTransfer::getModel()->GetImageResizeWidth() * KIROSyncTransfer::getModel()->GetRSDepthPixelBytes();
            msg_imgAlignedDepth->header.seq = imageAlignedDepth_SeqID++;
            msg_imgAlignedDepth->header.stamp = publish_timestamp;
            msg_imgAlignedDepth->header.frame_id = "0";

            KIROSyncTransfer::getControllerROS()->getPubImageRSAlignedDepth().publish(msg_imgAlignedDepth);
            CHECK_KIROSYNCTRANSFERCONTROLLERROS_THREADLOOP;

            // ImageIR (RealSense) //////////////////////////////////////////////////////////////////////////////////////////////////////
            KIROSyncTransfer::getModel()->GetData_ImageRSIR((char*&)(imgIR.data), ir_timestamp);
            timestamp_sec = (uint32_t)(ir_timestamp / 1000000);
            timestamp_usec = (uint32_t)(ir_timestamp % 1000000);
            CHECK_KIROSYNCTRANSFERCONTROLLERROS_THREADLOOP;

            cv::resize(imgIR, imgResizeIR, cv::Size(img_resize_width, img_resize_height), 0, 0, CV_INTER_CUBIC);
            CHECK_KIROSYNCTRANSFERCONTROLLERROS_THREADLOOP;

            msg_imgIR = cv_bridge::CvImage(std_msgs::Header(), "mono8", imgResizeIR).toImageMsg();
            msg_imgIR->width = KIROSyncTransfer::getModel()->GetImageResizeWidth();
            msg_imgIR->height = KIROSyncTransfer::getModel()->GetImageResizeHeight();
            msg_imgIR->is_bigendian = false;
            msg_imgIR->step = KIROSyncTransfer::getModel()->GetImageResizeWidth() * KIROSyncTransfer::getModel()->GetRSIRPixelBytes();
            msg_imgIR->header.seq = imageIR_SeqID++;
            msg_imgIR->header.stamp = publish_timestamp;
            msg_imgIR->header.frame_id = "0";

            KIROSyncTransfer::getControllerROS()->getPubImageRSIR().publish(msg_imgIR);
            CHECK_KIROSYNCTRANSFERCONTROLLERROS_THREADLOOP;

            // Image Camera Info (RealSense) //////////////////////////////////////////////////////////////////////////////////////////////
            KIROSyncTransfer::getModel()->GetData_ImageRSCameraInfo_Color(rs_color_camerainfo, intrinsic_timestamp);
            timestamp_sec = (uint32_t)(intrinsic_timestamp / 1000000);
            timestamp_usec = (uint32_t)(intrinsic_timestamp % 1000000);

            msg_RSColorCameraInfo.header.seq = RSColorCameraInfo_seqID++;
            msg_RSColorCameraInfo.header.stamp = publish_timestamp;
            msg_RSColorCameraInfo.header.frame_id = "0";
            msg_RSColorCameraInfo.width = rs_color_camerainfo.width;
            msg_RSColorCameraInfo.height = rs_color_camerainfo.height;
            msg_RSColorCameraInfo.distortion_model = "plumb_bob";
            for(int i=0; i<9; i++) msg_RSColorCameraInfo.K.at(i) = rs_color_camerainfo.K[i];
            for(int i=0; i<12; i++) msg_RSColorCameraInfo.P.at(i) = rs_color_camerainfo.P[i];
            for(int i=0; i<9; i++) msg_RSColorCameraInfo.R.at(i) = rs_color_camerainfo.R[i];
            msg_RSColorCameraInfo.D.resize(5);
            for(int i=0; i<5; i++) msg_RSColorCameraInfo.D.at(i) = rs_color_camerainfo.D[i];

            KIROSyncTransfer::getControllerROS()->getPubImageRSColorCameraInfo().publish(msg_RSColorCameraInfo);
            CHECK_KIROSYNCTRANSFERCONTROLLERROS_THREADLOOP;
            
            KIROSyncTransfer::getModel()->GetData_ImageRSCameraInfo_IR(rs_ir_camerainfo, intrinsic_timestamp);
            timestamp_sec = (uint32_t)(intrinsic_timestamp / 1000000);
            timestamp_usec = (uint32_t)(intrinsic_timestamp % 1000000);

            msg_RSIRCameraInfo.header.seq = RSIRCameraInfo_seqID++;
            msg_RSIRCameraInfo.header.stamp = publish_timestamp;
            msg_RSIRCameraInfo.header.frame_id = "0";
            msg_RSIRCameraInfo.width = rs_ir_camerainfo.width;
            msg_RSIRCameraInfo.height = rs_ir_camerainfo.height;
            msg_RSIRCameraInfo.distortion_model = "plumb_bob";
            for(int i=0; i<9; i++) msg_RSIRCameraInfo.K.at(i) = rs_ir_camerainfo.K[i];
            for(int i=0; i<12; i++) msg_RSIRCameraInfo.P.at(i) = rs_ir_camerainfo.P[i];
            for(int i=0; i<9; i++) msg_RSIRCameraInfo.R.at(i) = rs_ir_camerainfo.R[i];
            msg_RSIRCameraInfo.D.resize(5);
            for(int i=0; i<5; i++) msg_RSIRCameraInfo.D.at(i) = rs_ir_camerainfo.D[i];

            KIROSyncTransfer::getControllerROS()->getPubImageRSIRCameraInfo().publish(msg_RSIRCameraInfo);
            CHECK_KIROSYNCTRANSFERCONTROLLERROS_THREADLOOP;
            
            KIROSyncTransfer::getModel()->GetData_ImageRSCameraInfo_IR(rs_depth_camerainfo, intrinsic_timestamp);
            timestamp_sec = (uint32_t)(intrinsic_timestamp / 1000000);
            timestamp_usec = (uint32_t)(intrinsic_timestamp % 1000000);

            msg_RSDepthCameraInfo.header.seq = RSDepthCameraInfo_seqID++;
            msg_RSDepthCameraInfo.header.stamp = publish_timestamp;
            msg_RSDepthCameraInfo.header.frame_id = "0";
            msg_RSDepthCameraInfo.width = rs_depth_camerainfo.width;
            msg_RSDepthCameraInfo.height = rs_depth_camerainfo.height;
            msg_RSDepthCameraInfo.distortion_model = "plumb_bob";
            for(int i=0; i<9; i++) msg_RSDepthCameraInfo.K.at(i) = rs_depth_camerainfo.K[i];
            for(int i=0; i<12; i++) msg_RSDepthCameraInfo.P.at(i) = rs_depth_camerainfo.P[i];
            for(int i=0; i<9; i++) msg_RSDepthCameraInfo.R.at(i) = rs_depth_camerainfo.R[i];
            msg_RSDepthCameraInfo.D.resize(5);
            for(int i=0; i<5; i++) msg_RSDepthCameraInfo.D.at(i) = rs_depth_camerainfo.D[i];

            KIROSyncTransfer::getControllerROS()->getPubImageRSDepthCameraInfo().publish(msg_RSDepthCameraInfo);
            CHECK_KIROSYNCTRANSFERCONTROLLERROS_THREADLOOP;

            // ImageNV1 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            KIROSyncTransfer::getModel()->GetData_ImageNV1((char*&)(imgNV1.data), nv1_timestamp);
            timestamp_sec = (uint32_t)(nv1_timestamp / 1000000);
            timestamp_usec = (uint32_t)(nv1_timestamp % 1000000);
            CHECK_KIROSYNCTRANSFERCONTROLLERROS_THREADLOOP;

            cv::resize(imgNV1, imgResizeNV1, cv::Size(img_resize_width, img_resize_height), 0, 0, CV_INTER_CUBIC);
            CHECK_KIROSYNCTRANSFERCONTROLLERROS_THREADLOOP;

            msg_imgNV1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgResizeNV1).toImageMsg();
            msg_imgNV1->width = KIROSyncTransfer::getModel()->GetImageResizeWidth();
            msg_imgNV1->height = KIROSyncTransfer::getModel()->GetImageResizeHeight();
            msg_imgNV1->is_bigendian = false;
            msg_imgNV1->step = KIROSyncTransfer::getModel()->GetImageResizeWidth() * KIROSyncTransfer::getModel()->GetImageNVPixelBytes();
            msg_imgNV1->header.seq = imageNV1_SeqID++;
            msg_imgNV1->header.stamp = publish_timestamp;
            msg_imgNV1->header.frame_id = "0";

            KIROSyncTransfer::getControllerROS()->getPubImageNV1().publish(msg_imgNV1);
            CHECK_KIROSYNCTRANSFERCONTROLLERROS_THREADLOOP;

            // ImageThermal ///////////////////////////////////////////////////////////////////////////////////////////////////////
            KIROSyncTransfer::getModel()->GetData_ImageThermal((char*&)(imgThermal.data), thermal_timestamp);
            timestamp_sec = (uint32_t)(thermal_timestamp / 1000000);
            timestamp_usec = (uint32_t)(thermal_timestamp % 1000000);
            CHECK_KIROSYNCTRANSFERCONTROLLERROS_THREADLOOP;

            cv::resize(imgThermal, imgResizeThermal, cv::Size(img_resize_width, img_resize_height), 0, 0, CV_INTER_CUBIC);
            CHECK_KIROSYNCTRANSFERCONTROLLERROS_THREADLOOP;

            msg_imgThermal = cv_bridge::CvImage(std_msgs::Header(), "mono16", imgResizeThermal).toImageMsg();
            msg_imgThermal->width = KIROSyncTransfer::getModel()->GetImageResizeWidth();
            msg_imgThermal->height = KIROSyncTransfer::getModel()->GetImageResizeHeight();
            msg_imgThermal->is_bigendian = false;
            msg_imgThermal->step = KIROSyncTransfer::getModel()->GetImageResizeWidth() * KIROSyncTransfer::getModel()->GetImageThermalPixelBytes();
            msg_imgThermal->header.seq = imageThermal_SeqID++;
            msg_imgThermal->header.stamp = publish_timestamp;
            msg_imgThermal->header.frame_id = "0";

            KIROSyncTransfer::getControllerROS()->getPubImageThermal().publish(msg_imgThermal);
            CHECK_KIROSYNCTRANSFERCONTROLLERROS_THREADLOOP;

            // ImageGS ////////////////////////////////////////////////////////////////////////////////////////////////////////////
            /*
            KIROSyncTransfer::getModel()->GetData_ImageGS((char*&)(imgGS.data), gs_timestamp);
            timestamp_sec = (uint32_t)(gs_timestamp / 1000000);
            timestamp_usec = (uint32_t)(gs_timestamp % 1000000);
            CHECK_KIROSYNCTRANSFERCONTROLLERROS_THREADLOOP;

            cv::resize(imgGS, imgResizeGS, cv::Size(img_resize_width, img_resize_height), 0, 0, CV_INTER_CUBIC);
            CHECK_KIROSYNCTRANSFERCONTROLLERROS_THREADLOOP;

            msg_imgGS = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imgResizeGS).toImageMsg();
            msg_imgGS->width = KIROSyncTransfer::getModel()->GetImageResizeWidth();
            msg_imgGS->height = KIROSyncTransfer::getModel()->GetImageResizeHeight();
            msg_imgGS->is_bigendian = false;
            msg_imgGS->step = KIROSyncTransfer::getModel()->GetImageResizeWidth() * KIROSyncTransfer::getModel()->GetImageGSPixelBytes();
            msg_imgGS->header.seq = imageGS_SeqID++;
            msg_imgGS->header.stamp = publish_timestamp;
            msg_imgGS->header.frame_id = "0";

            KIROSyncTransfer::getControllerROS()->getPubImageGS().publish(msg_imgGS);
            CHECK_KIROSYNCTRANSFERCONTROLLERROS_THREADLOOP;
            */

            // LiDAR PointCloud ///////////////////////////////////////////////////////////////////////////////////////////////////
            KIROSyncTransfer::getModel()->GetData_PointCloud((ptcloud_t*)(ptcloud_data.get()), pointcloud_data_timestamp);
            timestamp_sec = (uint32_t)(pointcloud_data_timestamp / 1000000);
            timestamp_usec = (uint32_t)(pointcloud_data_timestamp % 1000000);
            CHECK_KIROSYNCTRANSFERCONTROLLERROS_THREADLOOP;

            msg_PointCloud.header.seq = PointCloud_seqID++;
            msg_PointCloud.header.stamp = publish_timestamp;
            msg_PointCloud.header.frame_id = "map";
            msg_PointCloud.fields.resize(4);
            msg_PointCloud.fields[0].name = "x";
            msg_PointCloud.fields[0].offset = 0;
            msg_PointCloud.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
            msg_PointCloud.fields[0].count = 1;
            msg_PointCloud.fields[1].name = "y";
            msg_PointCloud.fields[1].offset = 4;
            msg_PointCloud.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
            msg_PointCloud.fields[1].count = 1;
            msg_PointCloud.fields[2].name = "z";
            msg_PointCloud.fields[2].offset = 8;
            msg_PointCloud.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
            msg_PointCloud.fields[2].count = 1;
            msg_PointCloud.fields[3].name = "intensity";
            msg_PointCloud.fields[3].offset = 12;
            msg_PointCloud.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
            msg_PointCloud.fields[3].count = 1;
            msg_PointCloud.data.resize(std::max((size_t)1, (size_t)numPointCloud) * sizeof(ptcloud_t), 0);
            msg_PointCloud.point_step = sizeof(ptcloud_t);
            msg_PointCloud.row_step = msg_PointCloud.data.size();
            msg_PointCloud.height = 1;
            msg_PointCloud.width = msg_PointCloud.row_step / sizeof(ptcloud_t);
            msg_PointCloud.is_bigendian = false;
            msg_PointCloud.is_dense = true;

            uint8_t *ptr = msg_PointCloud.data.data();
            for(size_t i=0; i < numPointCloud; i++) {
                *(reinterpret_cast<float*>(ptr + 0)) = ptcloud_data.get()[i].at(0);
                *(reinterpret_cast<float*>(ptr + 4)) = ptcloud_data.get()[i].at(1);
                *(reinterpret_cast<float*>(ptr + 8)) = ptcloud_data.get()[i].at(2);
                *(reinterpret_cast<float*>(ptr + 12)) = ptcloud_data.get()[i].at(3);
                ptr += sizeof(ptcloud_t);
            }

            KIROSyncTransfer::getControllerROS()->getPubPointCloud().publish(msg_PointCloud);
            CHECK_KIROSYNCTRANSFERCONTROLLERROS_THREADLOOP;

            // Robot Odometry ///////////////////////////////////////////////////////////////////////////////////////////
            KIROSyncTransfer::getModel()->GetData_OdometryInfo(odometryinfo, odometryinfo_timestamp);
            CHECK_KIROSYNCTRANSFERCONTROLLERROS_THREADLOOP;

            msg_RobotOdometry.header.seq = odometryinfo.seq;
            msg_RobotOdometry.header.stamp = publish_timestamp;
            msg_RobotOdometry.header.frame_id = odometryinfo.frame_id;
            msg_RobotOdometry.child_frame_id = odometryinfo.child_frame_id;
            msg_RobotOdometry.pose.pose.position.x = odometryinfo.position_x;
            msg_RobotOdometry.pose.pose.position.y = odometryinfo.position_y;
            msg_RobotOdometry.pose.pose.position.z = odometryinfo.position_z;
            msg_RobotOdometry.pose.pose.orientation.x = odometryinfo.orientation_x;
            msg_RobotOdometry.twist.twist.linear.x = odometryinfo.linear_velocity_x;
            msg_RobotOdometry.twist.twist.linear.y = odometryinfo.linear_velocity_y;
            msg_RobotOdometry.twist.twist.angular.z = odometryinfo.angular_velocity_z;

            KIROSyncTransfer::getControllerROS()->getPubRobotOdometry().publish(msg_RobotOdometry);
            CHECK_KIROSYNCTRANSFERCONTROLLERROS_THREADLOOP;
        }

        CHECK_KIROSYNCTRANSFERCONTROLLERROS_THREADLOOP;
		//ros::spinOnce();
        rate.sleep();
        CHECK_KIROSYNCTRANSFERCONTROLLERROS_THREADLOOP;
    }

    if(ptcloud_data.get() != nullptr) ptcloud_data.reset(); ptcloud_data = nullptr;
    if(pKIROSyncTransferControllerROS != nullptr) pKIROSyncTransferControllerROS->threadProc_KIROSyncTransferControllerROS_RunFlag = false;

    return;
}





