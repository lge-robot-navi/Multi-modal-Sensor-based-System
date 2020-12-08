
#ifndef _SYNC_TRANSFER_CONTROLLER_ROS_H_983737272873783764826423432543523219412
#define _SYNC_TRANSFER_CONTROLLER_ROS_H_983737272873783764826423432543523219412

#include <thread>
#include <string>
#include <atomic>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


// external topic (subscribe from other node)


// internal topic (publish to other node)
#include <kiro_sync_transfer/ImageColor.h>
#include <kiro_sync_transfer/ImageDepth.h>
#include <kiro_sync_transfer/ImageIR.h>
#include <kiro_sync_transfer/ImageNV.h>
#include <kiro_sync_transfer/ImageThermal.h>
#include <kiro_sync_transfer/ImageGS.h>
#include <kiro_sync_transfer/CameraInfo.h>

// external service (from other node)

// internal service (to other node)
#include <kiro_sync_transfer/Initialize.h>
#include <kiro_sync_transfer/Finalize.h>
#include <kiro_sync_transfer/Run.h>
#include <kiro_sync_transfer/Stop.h>
#include <kiro_sync_transfer/SyncAllData.h>

#include "CommonStruct.h"


class KIROSyncTransferControllerROS
{
    public:
        KIROSyncTransferControllerROS();
        ~KIROSyncTransferControllerROS();

    public:
        int Initialize();
        int Finalize();
        
        int StartThreadLoop();
        int StopThreadLoop();
        bool isThreadRunning() { return threadProc_KIROSyncTransferControllerROS_RunFlag; }

    public:
        bool GetFlag_PublishRunning() { return _bPublishRunFlag.load(); }
        void SetFlag_PublishRunning(bool flag) { _bPublishRunFlag.store(flag); }

        int GetROSState() { return _ROSState; }

    ///// ROS Interface
    public:
        //ros::Publisher& getPubImageRaw() { return pub_ImageRaw; }
        //ros::Publisher& getPubImageDepth() { return pub_ImageDepth; }
        //ros::Publisher& getPubImageIR() { return pub_ImageIR; }

        image_transport::Publisher getPubImageRSColor() { return pub_ImageRSColor; }
        image_transport::Publisher getPubImageRSDepth() { return pub_ImageRSDepth; }
        image_transport::Publisher getPubImageRSAlignedDepth() { return pub_ImageRSAlignedDepth; }
        image_transport::Publisher getPubImageRSIR()    { return pub_ImageRSIR; }
        image_transport::Publisher getPubImageNV1()     { return pub_ImageNV1; }
        image_transport::Publisher getPubImageThermal() { return pub_ImageThermal; }
        image_transport::Publisher getPubImageGS()      { return pub_ImageGS; }
        
        ros::Publisher getPubImageRSColorCameraInfo()   { return pub_ImageRSColorCameraInfo; }
        ros::Publisher getPubImageRSIRCameraInfo()      { return pub_ImageRSIRCameraInfo; }
        ros::Publisher getPubImageRSDepthCameraInfo()   { return pub_ImageRSDepthCameraInfo; }

        ros::Publisher getPubPointCloud()               { return pub_PointCloud; }

        ros::Publisher getPubRobotOdometry()            { return pub_RobotOdometry; }

    public:
        bool Call_Initialize();
        bool Call_Finalize();
        bool Call_Run();
        bool Call_Stop();

    private:
	    void InitPublisher();
        void InitSubscriber();
        void InitServiceProvider();
        void InitServiceClient();

    private:
        ros::NodeHandle nh_Pub, nh_Sub;
        ros::NodeHandle nh_Service, nh_Client;
        image_transport::ImageTransport it_;

	    //ros::Publisher pub_ImageRaw, pub_ImageDepth, pub_ImageIR;
	    image_transport::Publisher pub_ImageRSColor, pub_ImageRSDepth, pub_ImageRSAlignedDepth, pub_ImageRSIR;
        image_transport::Publisher pub_ImageNV1, pub_ImageThermal; //pub_ImageNV2, 
        image_transport::Publisher pub_ImageGS;
        ros::Publisher pub_ImageRSColorCameraInfo, pub_ImageRSIRCameraInfo, pub_ImageRSDepthCameraInfo;
        ros::Publisher pub_PointCloud;
        ros::Publisher pub_RobotOdometry;

        ros::Subscriber sub_Odometry;

        ros::ServiceServer srv_Initialize, srv_Finalize, srv_Run, srv_Stop;
        ros::ServiceServer srv_SyncAllData;

    private:
        // publish handler (at connect)
        static void publishHandler_ImageRSColor(const image_transport::SingleSubscriberPublisher& pub);
        static void publishHandler_ImageRSDepth(const image_transport::SingleSubscriberPublisher& pub);
        static void publishHandler_ImageRSAlignedDepth(const image_transport::SingleSubscriberPublisher& pub);
        static void publishHandler_ImageRSIR(const image_transport::SingleSubscriberPublisher& pub);
        static void publishHandler_ImageNV1(const image_transport::SingleSubscriberPublisher& pub);
        static void publishHandler_ImageThermal(const image_transport::SingleSubscriberPublisher& pub);
        static void publishHandler_ImageGS(const image_transport::SingleSubscriberPublisher& pub);
        static void publishHandler_ImageRSColorCameraInfo(const ros::SingleSubscriberPublisher& pub);
        static void publishHandler_ImageRSIRCameraInfo(const ros::SingleSubscriberPublisher& pub);
        static void publishHandler_ImageRSDepthCameraInfo(const ros::SingleSubscriberPublisher& pub);
        static void publishHandler_PointCloud(const ros::SingleSubscriberPublisher& pub);
        static void publishHandler_RobotOdometry(const ros::SingleSubscriberPublisher& pub);

        // subscribe handler
        static void subscribeHandler_Odometry(const nav_msgs::Odometry &msg);

        // service handler
        static bool serviceHandler_Initialize(kiro_sync_transfer::Initialize::Request &req, kiro_sync_transfer::Initialize::Response &res);
        static bool serviceHandler_Finalize(kiro_sync_transfer::Finalize::Request &req, kiro_sync_transfer::Finalize::Response &res);
        static bool serviceHandler_Run(kiro_sync_transfer::Run::Request &req, kiro_sync_transfer::Run::Response &res);
        static bool serviceHandler_Stop(kiro_sync_transfer::Stop::Request &req, kiro_sync_transfer::Stop::Response &res);
        static bool serviceHandler_SyncAllData(kiro_sync_transfer::SyncAllData::Request &req, kiro_sync_transfer::SyncAllData::Response &res);

	private:
        mutable std::mutex _mtx;
        std::atomic<bool> _bPublishRunFlag;
        std::atomic<int> _ROSState;

		std::unique_ptr<std::thread> _thKIROSyncTransferControllerROS;
		std::atomic<bool> threadProc_KIROSyncTransferControllerROS_RunFlag;
		static void ThreadProc_KIROSyncTransferControllerROS(void *arg);

};

#endif 
