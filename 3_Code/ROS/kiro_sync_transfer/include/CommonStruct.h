
#ifndef _COMMONSTRUCT_H_879314243242378921356347812546834217683421222
#define _COMMONSTRUCT_H_879314243242378921356347812546834217683421222

#include <string>
#include <array>

#define _LINE_PRINT(x)	fprintf(stderr, "[%s:%d] %s: %s\n", __FILE__, __LINE__, __func__, x);

#ifndef _CAMERAINFO_STRUCT_
#define _CAMERAINFO_STRUCT_
    struct CameraInfo
    {
        unsigned int height;
        unsigned int width;
        char distortion_model[9];
        float D[5];
        float K[9];
        float R[9];
        float P[12];
    };
#endif 

#ifndef _ODOMETRY_STRUCT_
#define _ODOMETRY_STRUCT_
    struct OdometryInfo
    {
        unsigned int seq;
        std::string frame_id;
        std::string child_frame_id;
        double position_x;
        double position_y;
        double position_z;
        double orientation_x;
        double orientation_y;
        double orientation_z;
        double orientation_w;
        double linear_velocity_x;
        double linear_velocity_y;
        double angular_velocity_z;
    };
#endif

#ifndef _PTCLOUD_T_
#define _PTCLOUD_T_
    typedef std::array<float, 4> ptcloud_t;
#endif 

#endif
