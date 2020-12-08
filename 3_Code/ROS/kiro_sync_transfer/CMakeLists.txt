cmake_minimum_required(VERSION 2.8.3)
project(kiro_sync_transfer)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	roscpp
	std_msgs
	image_transport
	sensor_msgs
	nav_msgs
	cv_bridge
	camera_info_manager
)

add_message_files(
	FILES
	ImageColor.msg
	ImageDepth.msg
	ImageIR.msg
	ImageNV.msg 
	ImageThermal.msg
    ImageGS.msg
    CameraInfo.msg
)

add_service_files(
	FILES
	Initialize.srv
	Finalize.srv
	Run.srv
	Stop.srv
	SyncAllData.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs
	sensor_msgs
	nav_msgs
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES {PROJECT_NAME}
	CATKIN_DEPENDS std_msgs image_transport sensor_msgs cv_bridge 
)

###########
## Build ##
###########

include_directories(include /usr/include/libxml2 ${catkin_INCLUDE_DIRS})
link_directories(./)

add_executable(kiro_sync_transfer_node
	src/SyncTransfer_Node.cpp
	src/SyncTransfer_Model.cpp
	src/SyncTransfer_Controller_ROS.cpp
	src/SyncTransfer_Controller_XMLRPC.cpp
)

target_link_libraries(kiro_sync_transfer_node ${catkin_LIBRARIES} 
	xml2 xmlrpc xmlrpc_xmlparse xmlrpc_xmltok xmlrpc_util
	xmlrpc_server xmlrpc_server_abyss xmlrpc_client
	opencv_core opencv_highgui 
	memsync
)

add_dependencies(kiro_sync_transfer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# check C++14 / C++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0x)
if(COMPILER_SUPPORTS_CXX17)
	set(CMAKE_CXX_FLAGS "-std=c++17 -g")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "-std=c++0x -g")
else()
	message(FATAL_ERROR "The compiler ${CATKIN_CXX_COMPILER} has no c++17/0x support")
endif()






