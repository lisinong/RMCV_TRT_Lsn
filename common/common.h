#ifndef COMMON
#define COMMON
#include <iostream>
#include <time.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <unistd.h>
#include <sys/time.h>
#include <thread>
#include <termio.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
using namespace std;
using namespace cv;
using namespace Eigen;


#define display_mode DisplayMode::Open
//#define display_mode DisplayMode::Close

#define X_BIAS 0.065
#define Y_BIAS 0.05
#define Z_BIAS 0.035

enum DisplayMode {Open = 1,Close = 0};

namespace Horizon{

    class Speed
    {
    public:
        Speed():rate(0.0),direction(0){}
        double        rate;// 速率
        int      direction;// 方向，-1为逆时针，0为不转，1为顺时针转
    };


    // 返回当前时间

    static long now()
    {
        timeval tv;
        gettimeofday(&tv, NULL);
        return tv.tv_sec*1000 + tv.tv_usec/1000;
    }


    /*
    * @brief:  位姿的实现方式 
    * 
    */
    class GimbalPose
    {
    public:
        float  pitch;
        float  yaw;
        float  roll;
        double timestamp;
        // 初始化函数
        GimbalPose(float pitch = 0.0, float yaw = 0.0, float roll = 0.0)
        {
            this->pitch     = pitch;
            this->yaw       = yaw;
            this->roll      = roll;
        }
        // 左值
        GimbalPose operator=(const GimbalPose& gm)
        {
            this->pitch     = gm.pitch;
            this->yaw       = gm.yaw;
            this->roll      = gm.roll;
            this->timestamp = gm.timestamp;
            return *this;
        }

        GimbalPose operator=(const float init_value)
        {
            this->pitch     = init_value;
            this->yaw       = init_value;
            this->roll      = init_value;
            this->timestamp = now();
            return *this;
        }

        friend GimbalPose operator-(const GimbalPose& gm1, const GimbalPose gm2)
        {
            GimbalPose temp{};
            temp.pitch     = gm1.pitch - gm2.pitch;
            temp.yaw       = gm1.yaw   - gm2.yaw;
            temp.roll      = gm1.roll  - gm2.roll;
            temp.timestamp = now();
            return temp;
        }

        friend GimbalPose operator+(const GimbalPose& gm1, const GimbalPose gm2)
        {
            GimbalPose temp{};
            temp.pitch     = gm1.pitch + gm2.pitch;
            temp.yaw       = gm1.yaw   + gm2.yaw;
            temp.roll      = gm1.roll  + gm2.roll;
            temp.timestamp = now();
            return temp;
        }

        friend GimbalPose operator*(const GimbalPose& gm, const float k)
        {
            GimbalPose temp{};
            temp.pitch     = gm.pitch * k;
            temp.yaw       = gm.yaw   * k;
            temp.roll      = gm.roll  * k;
            temp.timestamp = now();
            return temp;
        }

        friend GimbalPose operator*(const float k, const GimbalPose& gm)
        {
            GimbalPose temp{};
            temp.pitch     = gm.pitch * k;
            temp.yaw       = gm.yaw   * k;
            temp.roll      = gm.roll  * k;
            temp.timestamp = now();
            return temp ;
        }

        friend GimbalPose operator/(const GimbalPose& gm, const float k)
        {
            GimbalPose temp{};
            temp.pitch     = gm.pitch / k;
            temp.yaw       = gm.yaw   / k;
            temp.roll      = gm.roll  / k;
            temp.timestamp = now();
            return temp;
        }

        friend std::ostream& operator<<(std::ostream& out, const GimbalPose& gm)
        {
            out << "[pitch : " << gm.pitch << ", yaw : " << gm.yaw << "]" << endl;
            return out;
        }
    };


template<class T,int BUFFER>
class CircularQueue {
public:
    CircularQueue()
    {
        init();
        std::cout << "[enter queue]" << std::endl;
    }

    bool Enqueue(T &val);

    void init();

    int size();

private:
    // const static int BUFFER = 60;
    long long int iter_;
    int size_;
public:
    T queue_[BUFFER];
};

template <class T,int BUFFER>
bool CircularQueue<T,BUFFER>::Enqueue(T &val)
{
    queue_[iter_%BUFFER] = val;
    iter_++;
    size_ = iter_;
    if(iter_ >= BUFFER)
    {
        size_ = BUFFER;
    }
}

template <class T,int BUFFER>
void CircularQueue<T,BUFFER>::init()
{
    iter_ = 0;
    size_ = 0;
}

template <class T,int BUFFER>
int CircularQueue<T,BUFFER>::size()
{
    return size_;
}




    // 装甲板的类
    // class Armor
    // {
    // public:
    //     Armor():distance_(0),id_(0){};
    //     //Armor(float distance,int id):distance_(distance),id_(id){};
    // public:
    //     Point2f         point2d[4];// 装甲板的四个顶点的二维坐标
    //     Point3f three_equal_point_;// 
    //     Point2f          center2d_;// 二维中心点
    //     Point3f          center3d_;// 三维中心点
    //     float            distance_;// 装甲板的距离，相对于相机坐标系,单双目均可，主单目
    //     GimbalPose       cur_pose_;// 云台当前的位姿
    //     double        h_yaw_angle_;// 打击陀螺时判断的角度
    //     double               area_;// 装甲板的二维坐标
    //     //cv::Point3f  left_up_,right_up_,right_down_,left_down_;// 四个灯条的三维坐标，双目用
    //     int id_;                   // 装甲板的id
    //     long h_time_stamp_;        // 时间戳

    //     float angle_;              // 两灯条的夹角

    //     float width_;
    //     float height_;


    // public:
    // // 识别所用的Armor成员以及操作
    // public:
    //     ~Armor(){};
    // public:
    //     float pos[3];

    //     Point2f left_led_[2];         //左灯条
    //     Point2f right_led_[2];        //右灯条
    //     double tx = 0,ty = 0,tz = 0;
    //     bool is_findTarget = 0;
    //     enum ArmorType{SMALL,LARGE,RUNE};
    //     ArmorType armor_type_ = SMALL;//
        
    //     int id;                     // 当前装甲板数字

	// /***********antigyro_data*************/
	// double theta_x_;
	// double theta_y_;
	// double theta_z_;

    // };

    // inline float getDistance(const Point3f & p1,const Point3f & p2){
    //     return sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2)+pow(p1.z-p2.z,2));
    // }

    // inline float getDistance(const Point2f & p1,const Point2f & p2){
    //     return sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2));
    // }

}
#endif
