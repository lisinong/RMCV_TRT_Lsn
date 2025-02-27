#ifndef COMMON_H
#define COMMON_H
#include <iostream>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <sys/time.h>
#include <thread>
#include <termio.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <opencv2/core/eigen.hpp>

#define FMT_HEADER_ONLY 1
#include <fmt/format.h>
#include <fmt/color.h>
#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"
using namespace std;
using namespace cv;
using namespace Eigen;

// 读取视频调试
//  #define VIDEO

// 保存内录
//  #define SAVE_VIDEO

// 相机选择
//#define DAHENG
//#define MIDVISION
#define HK
// debug相关参数
#define Debug_yaml
#define Debug_image
//   #define Release
//   #define cam_show

#define RESET "\033[0m"
#define BLACK "\033[30m"   /* Black */
#define RED1 "\033[31m"    /* Red */
#define GREEN "\033[32m"   /* Green */
#define YELLOW "\033[33m"  /* Yellow */
#define BLUE1 "\033[34m"   /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m"    /* Cyan */
#define WHITE "\033[37m"   /* White */

namespace Horizon
{
    inline void log(string loggername, string logstring)
    {
        try
        {
            auto my_logger = spdlog::basic_logger_mt(loggername, "logs/basic.txt");
            my_logger->info(logstring);
            my_logger->flush();
        }
        catch (const std::exception &e)
        {
            std::cerr << e.what() << '\n';
        }
    }

    class Speed
    {
    public:
        Speed() : rate(0.0), direction(0) {}
        double rate;   // 速率
        int direction; // 方向，-1为逆时针，0为不转，1为顺时针转
    };
    // 返回当前时间
    static long now()
    {
        timeval tv;
        gettimeofday(&tv, NULL);
        return tv.tv_sec * 1000 + tv.tv_usec / 1000;
    }
    /*
     * @brief:  位姿的实现方式
     *
     */
    class GimbalPose
    {
    public:
        float pitch;
        float yaw;
        float roll;
        double timestamp;
        // 初始化函数
        GimbalPose(float pitch = 0.0, float yaw = 0.0, float roll = 0.0)
        {
            this->pitch = pitch;
            this->yaw = yaw;
            this->roll = roll;
        }
        // 左值
        GimbalPose operator=(const GimbalPose &gm)
        {
            this->pitch = gm.pitch;
            this->yaw = gm.yaw;
            this->roll = gm.roll;
            this->timestamp = gm.timestamp;
            return *this;
        }
        GimbalPose operator=(const float init_value)
        {
            this->pitch = init_value;
            this->yaw = init_value;
            this->roll = init_value;
            this->timestamp = now();
            return *this;
        }
        friend GimbalPose operator-(const GimbalPose &gm1, const GimbalPose gm2)
        {
            GimbalPose temp{};
            temp.pitch = gm1.pitch - gm2.pitch;
            temp.yaw = gm1.yaw - gm2.yaw;
            temp.roll = gm1.roll - gm2.roll;
            temp.timestamp = now();
            return temp;
        }
        friend GimbalPose operator+(const GimbalPose &gm1, const GimbalPose gm2)
        {
            GimbalPose temp{};
            temp.pitch = gm1.pitch + gm2.pitch;
            temp.yaw = gm1.yaw + gm2.yaw;
            temp.roll = gm1.roll + gm2.roll;
            temp.timestamp = now();
            return temp;
        }
        friend GimbalPose operator*(const GimbalPose &gm, const float k)
        {
            GimbalPose temp{};
            temp.pitch = gm.pitch * k;
            temp.yaw = gm.yaw * k;
            temp.roll = gm.roll * k;
            temp.timestamp = now();
            return temp;
        }
        friend GimbalPose operator*(const float k, const GimbalPose &gm)
        {
            GimbalPose temp{};
            temp.pitch = gm.pitch * k;
            temp.yaw = gm.yaw * k;
            temp.roll = gm.roll * k;
            temp.timestamp = now();
            return temp;
        }
        friend GimbalPose operator/(const GimbalPose &gm, const float k)
        {
            GimbalPose temp{};
            temp.pitch = gm.pitch / k;
            temp.yaw = gm.yaw / k;
            temp.roll = gm.roll / k;
            temp.timestamp = now();
            return temp;
        }
        friend std::ostream &operator<<(std::ostream &out, const GimbalPose &gm)
        {
            out << "[pitch : " << gm.pitch << ", yaw : " << gm.yaw << "]" << endl;
            return out;
        }
    };
}
#endif
