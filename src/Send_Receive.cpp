#include "Send_Receive.h"

namespace Horizon
{
#define DATA_LENGTH 16 // 接受的数据位数

    int OpenPort(const char *Portname)
    {
        int fd;
        fd = open(Portname, O_RDWR | O_NOCTTY | O_NONBLOCK);
        cout << fd << endl;
        if (-1 == fd)
        {
            printf("The port open error!\n");
            return -1;
        }
        else
        {
            fcntl(fd, F_SETFL, 0); // 读取串口的信息
        }
        return fd;
    }

    int configureSerial(int fd)
    {
        struct termios port_set;
        // 波特率
        cfsetispeed(&port_set, B115200);
        port_set.c_iflag &= ~ISTRIP; // 禁用输入数据的字符剥离。如果被设置，所有接收的字符将被剥离到7位。
        // port_set.c_iflag &= ~CRTSCTS;       禁用硬件流控制（RTS/CTS）
        // port_set.c_iflag |= CLOCAL|CREAD;   设置 CLOCAL 和 CREAD 标志。

        // CLOCAL 标志用于控制是否忽略调制解调器的状态线。如果设置了 CLOCAL，那么程序不会关心调制解调器的状态线，这对于使用虚拟串口或者没有物理调制解调器的情况很有用。
        // CREAD 标志用于控制是否能从串口读取数据。如果清除了 CREAD 标志，那么程序将无法从串口读取数据。

        port_set.c_cflag &= ~PARENB;                                   // 禁用奇偶校验
        port_set.c_cflag &= ~PARODD;                                   // 禁用奇校验，如果PARENB被设置的话
        port_set.c_cflag &= ~CSTOPB;                                   // 使用一位停止位，如果被设置，将使用两位
        port_set.c_cflag &= ~CSIZE;                                    // 清除字符大小掩码
        port_set.c_cflag |= CS8;                                       // 设置字符大小为8位
        port_set.c_iflag &= ~(IXON | IXOFF | IXANY);                   // 禁用软件流控制
        port_set.c_iflag &= ~(INLCR | IGNCR | ICRNL);                  // 禁用输入的 NL-CR, CR-NL 和 CR 转换
        port_set.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);           // 禁用规范模式，回显，和信号
        port_set.c_oflag &= ~OPOST;                                    // 禁用输出处理
        port_set.c_cc[VTIME] = 0;                                      // 设置读取的超时时间为0
        port_set.c_cc[VMIN] = 0;                                       // 设置读取的最小字符数为0
        port_set.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON); // 禁用中断，CR-NL 转换，输入奇偶校验，字符剥离，和软件流控制
        tcsetattr(fd, TCSANOW, &port_set);                             // 立即改变参数
        return (fd);
    }

    void DataControler::sentData(int fd, VisionData data)
    {
        unsigned char send_bytes[15] = {0};
        send_bytes[0] = 0xaa;
        send_bytes[14] = 0xbb;

        // data.time.f = 10;
        send_bytes[1] = data.pitch_data_.c[0];
        send_bytes[2] = data.pitch_data_.c[1];
        send_bytes[3] = data.pitch_data_.c[2];
        send_bytes[4] = data.pitch_data_.c[3];

        // data.yaw_data_.f = 10;

        send_bytes[5] = data.yaw_data_.c[0];
        send_bytes[6] = data.yaw_data_.c[1];
        send_bytes[7] = data.yaw_data_.c[2];
        send_bytes[8] = data.yaw_data_.c[3];

        state_ = 0;

        if (state_ == 0)
        { // zimiao  001

            SET_BIT((data.OnePointFive), 1);
            CLEAR_BIT((data.OnePointFive), 2);
            CLEAR_BIT((data.OnePointFive), 3);
        }
        else if (state_ == 1)
        { // dafu   110

            SET_BIT((data.OnePointFive), 3);
            SET_BIT((data.OnePointFive), 2);
            CLEAR_BIT((data.OnePointFive), 1);
        }
        else if (state_ == 2)
        { // xiaofu   100

            SET_BIT((data.OnePointFive), 3);
            CLEAR_BIT((data.OnePointFive), 2);
            CLEAR_BIT((data.OnePointFive), 1);
        }
        else
        { // fantuoluo   010

            CLEAR_BIT((data.OnePointFive), 3);
            SET_BIT((data.OnePointFive), 2);
            CLEAR_BIT((data.OnePointFive), 1);
        }
        if (data.is_have_armor)
        {
            SET_BIT((data.OnePointFive), 5);
        }
        else
        {
            CLEAR_BIT((data.OnePointFive), 5);
        }
        send_bytes[9] = data.OnePointFive;

        // data.time.f = 1;
        send_bytes[10] = data.time.c[0];
        send_bytes[11] = data.time.c[1];
        send_bytes[12] = data.time.c[2];
        send_bytes[13] = data.time.c[3];

        if (write(fd, send_bytes, sizeof(send_bytes)))
        {
            std::cout << "发送数据成功！！！" << std::endl;
        }
    }

    void DataControler::getData(int fd, Stm32Data &get_data)
    {
        int bytes = 0;
        ioctl(fd, FIONREAD, &bytes);
        if (bytes < DATA_LENGTH)
        {
            return;
        }
        unsigned char *rec_bytes = new unsigned char[bytes + 100]();
        bytes = read(fd, rec_bytes, bytes);

        int FirstIndex = -1;
        int LastIndex = -1;

        for (int i = 0; i < bytes; i++)
        {
            if (rec_bytes[i] == 0xaa)
            {
                // cout << "head top index" << endl;
                FirstIndex = i;
            }
            else if (rec_bytes[i] == 0xbb && FirstIndex != -1 && i - FirstIndex == DATA_LENGTH - 1)
            {
                // cout << "tail top index" << endl;
                LastIndex = i;
                break;
            }
            else
            {
            }
        }

        if (FirstIndex != -1 && LastIndex != -1)
        {

            // get_data.IsHave = true;
            get_data.pitch_data_.c[0] = rec_bytes[FirstIndex + 1];
            get_data.pitch_data_.c[1] = rec_bytes[FirstIndex + 2];
            get_data.pitch_data_.c[2] = rec_bytes[FirstIndex + 3];
            get_data.pitch_data_.c[3] = rec_bytes[FirstIndex + 4];

            // cout << "get_data_pitch: " << get_data.pitch_data_.f << endl;

            get_data.yaw_data_.c[0] = rec_bytes[FirstIndex + 5];
            get_data.yaw_data_.c[1] = rec_bytes[FirstIndex + 6];
            get_data.yaw_data_.c[2] = rec_bytes[FirstIndex + 7];
            get_data.yaw_data_.c[3] = rec_bytes[FirstIndex + 8];

            // cout << "get_data_yaw: " << get_data.yaw_data_.f << endl;

            get_data.OnePointFive = rec_bytes[FirstIndex + 9];

            if (getBit(get_data.OnePointFive, 1) == 0 && getBit(get_data.OnePointFive, 2) == 0 && getBit(get_data.OnePointFive, 3) == 1)
            {
                get_data.flag = 0;
            }
            else if (getBit(get_data.OnePointFive, 1) == 1 && getBit(get_data.OnePointFive, 2) == 1 && getBit(get_data.OnePointFive, 3) == 0)
            {
                get_data.flag = 1;
            }
            else if (getBit(get_data.OnePointFive, 1) == 1 && getBit(get_data.OnePointFive, 2) == 0 && getBit(get_data.OnePointFive, 3) == 0)
            {
                get_data.flag = 2;
            }
            else if (getBit(get_data.OnePointFive, 1) == 0 && getBit(get_data.OnePointFive, 2) == 1 && getBit(get_data.OnePointFive, 3) == 0)
            {
                get_data.flag = 3;
            }

            if (getBit(get_data.OnePointFive, 4) == 0)
            {
                cout << "\033[31m识别红色\033[0m" << endl;
                get_data.color_ = false;
            }
            else
            {
                cout << "\033[34m识别蓝色\033[0m" << endl;
                get_data.color_ = true;
            }
            if (getBit(get_data.OnePointFive, 5) == 0)
            {
                get_data.state_outpost = false;
            }
            else
            {
                get_data.state_outpost = true;
            }

            get_data.time.c[0] = rec_bytes[FirstIndex + 10];
            get_data.time.c[1] = rec_bytes[FirstIndex + 11];
            get_data.time.c[2] = rec_bytes[FirstIndex + 12];
            get_data.time.c[3] = rec_bytes[FirstIndex + 13];

            // 设置射速
            get_data.init_firing_rate = rec_bytes[FirstIndex + 14];

            cout << "接收完成" << get_data.time.f << endl;
            if (get_data.pitch_data_.f < (-180) || get_data.pitch_data_.f > (180))
            {
                cout << "Invalid pitch data: " << get_data.pitch_data_.f << endl;
                get_data.pitch_data_.f = 0;
                
            }

            if (get_data.yaw_data_.f < (-180) || get_data.yaw_data_.f > (180))
            {
                cout << "Invalid yaw data: " << get_data.yaw_data_.f << endl;
                get_data.yaw_data_.f = 0;
                
            }
        }
        else
        {
            cout << "接收失败!" << endl;
        }
        return;
    }

}
