#ifndef THREAD_H
#define THREAD_H
#include "../DaHeng/DaHengCamera.h"
#include "TRTModule.hpp"
#include "../MidVision/MidCamera.h"
#include "predictor.h"
#include "Send_Receive.h"
#include <mutex>
#include <future>
#include "../HikVision/include/hikvision_camera.h"

using namespace Horizon;
enum class SerialState
{
    SEND,
    RECEIVE,
    WAIT
};

enum BufferSize
{
    IMGAE_BUFFER = 5
};

class Factory
{
public:
    Factory()
    {
        coord[0] = 0;
        coord[1] = 0;
        coord[2] = 0;

        rotation[0] = 0;
        rotation[1] = 0;
        rotation[2] = 0;
    }

public:
    cv::Mat img;

public:
    cv::Mat image_buffer_[BufferSize::IMGAE_BUFFER];
    double timer_buffer_[IMGAE_BUFFER];
    volatile unsigned int image_buffer_front_ = 0; // the produce index
    volatile unsigned int image_buffer_rear_ = 0;  // the comsum index

    void producer();

    void consumer();

    void Getdata();

private:
    Horizon::DataControler::Stm32Data TimeSynchronization(std::deque<Horizon::DataControler::Stm32Data> &stm32s, double src_time);
    Horizon::DataControler datacontroler;

    // 电控和视觉串口发数和收数，要发的数和要收的数
    Horizon::DataControler::VisionData visiondata; // 视觉向电控传数据
    Horizon::DataControler::Stm32Data stm32data;   // 电控向视觉发数据

    std::shared_ptr<PredictorPose> predic_pose_ = std::make_shared<PredictorPose>(); // 解算器

    Eigen::Vector3d coord; // 世界坐标
    Eigen::Vector3d rotation;

    volatile SerialState serial_state_; // 收发数的状态
    mutex serial_mutex_;                // 数据上🔓

    GimbalPose imu_data; // 电控发来的数据

    // 陀螺周期和发弹频率，仅在陀螺模式下使用
    std::tuple<long, long> T_f;
    // 定义收数位姿
    GimbalPose gimbal;
    // 定义发数的位姿
    GimbalPose gim;
    // enum DisplayMode {Open = 1,Close = 0};

    Horizon::DataControler::Stm32Data stm32data_temp;

    // CircularQueue<Horizon::DataControler::Stm32Data,1000> stm32_deque_;
    std::deque<Horizon::DataControler::Stm32Data> MCU_data_;
    int mcu_size_ = 5;

    bool is_armor_;
    int fd;
    vector <double>aver_fps;
};
#endif
