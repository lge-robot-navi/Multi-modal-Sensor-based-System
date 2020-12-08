
///// Debug Print Macro
#define _LINE_PRINT(x)	fprintf(stderr, "[%s:%d] %s: %s\n", __FILE__, __LINE__, __func__, x);


///// XMLRPC Config
#define	KIRO_CAMERA_DATASYNC_PORTNUMBER			    8001
#define	KIRO_CAMERA_DATASYNC_ADDRESS				"localhost"

#define KIRO_LIDAR_DATASYNC_PORTNUMBER              8005
#define KIRO_LIDAR_DATASYNC_ADDRESS                 "localhost"

#define KIRO_SYNC_TRANSFER_DATASYNC_PORTNUMBER      8009
#define KIRO_SYNC_TRANSFER_DATASYNC_ADDRESS         "localhost"

