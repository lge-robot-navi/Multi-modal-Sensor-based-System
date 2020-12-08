
#include <ros/ros.h>
#include <signal.h>

#include "SyncTransfer.h"

int main(int argc, char **argv)
{
    static bool bRunningFlag = true;

    ros::init(argc, argv, "kiro_sync_transfer_node");

    KIROSyncTransfer::getInstance();

    // check operation mode 
    if(argc >= 2 && std::strncmp(argv[1], "-simmode", 8) == 0) {
        KIROSyncTransfer::getModel()->SetSimulationMode(true);        
        fprintf(stderr, "kiro_sync_transfer_node:: Set Simulation Mode\n");
    }

	// set signal handler
	signal(SIGINT, [](int){ bRunningFlag = false; });
	signal(SIGABRT, [](int){ bRunningFlag = false; });
	signal(SIGKILL, [](int){ bRunningFlag = false; });
	signal(SIGTERM, [](int){ bRunningFlag = false; });

    // auto-start
    if(bRunningFlag == true) {
        KIROSyncTransfer::getControllerROS()->Call_Initialize();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        KIROSyncTransfer::getControllerROS()->Call_Run();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    while(bRunningFlag == true && ros::ok())
    {
        if(KIROSyncTransfer::getModel().get() == nullptr ||
           KIROSyncTransfer::getControllerROS().get() == nullptr) break;

        ros::spinOnce();

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    
    fprintf(stderr, "kiro_sync_transfer_node:: Shutdown...\n");
    ros::shutdown();
    return 0;
}


