#include "thread.h"
#include "Send_Receive.h"
#include "TRTModule.hpp"

/*图像🔓没写但海康的有*/

using namespace Horizon;
using namespace camera;
// 世界坐标系内坐标--->相机坐标系内坐标
inline Eigen::Vector3d pw_to_pc(const Eigen::Vector3d &pw, const Eigen::Matrix3d &R_CW)
{
	cv::FileStorage fs(debug_yaml, cv::FileStorage::READ);
	float X_BIAS, Y_BIAS, Z_BIAS;
	fs["X_BIAS"] >> X_BIAS;
	fs["Y_BIAS"] >> Y_BIAS;
	fs["Z_BIAS"] >> Z_BIAS;
	Eigen::Vector3d pw_t;
	pw_t = R_CW * pw;
	pw_t[0] = pw_t[0] - X_BIAS;
	pw_t[1] = pw_t[1] - Y_BIAS;
	pw_t[2] = pw_t[2] - Z_BIAS;

	pw_t[1] = -pw_t[1];
	fs.release();
	return pw_t;
}

// 相机坐标系内坐标--->图像坐标系内像素坐标
inline Eigen::Vector3d pc_to_pu(const Eigen::Vector3d &pc, const Eigen::Matrix3d &F)
{
	return F * pc / pc(2, 0);
}

namespace GxCamera
{
	int GX_exp_time = 7000;

	int GX_gain = 10;
	DaHengCamera *camera_ptr_ = nullptr;
	int GX_blance_r = 49;
	int GX_blance_g = 32;
	int GX_blance_b = 35;

	int GX_gamma = 1;

	void DaHengSetExpTime(int, void *)
	{
		camera_ptr_->SetExposureTime(GX_exp_time);
	}

	void DaHengSetGain(int, void *)
	{
		camera_ptr_->SetGain(3, GX_gain);
	}

}
namespace MidCamera
{
	int MV_exp_value = 13000;
	MVCamera *camera_ptr_ = nullptr;
	void MVSetExpTime(int, void *)
	{
		camera_ptr_->SetExpose(MV_exp_value);
	}
}
namespace HKcamera
{
	HikCamera *MVS_cap = nullptr;												// 创建一个相机对象
	const string camera_config_path = "../HikVision/config/camera_config.yaml"; // 相机配置文件路径
	const string intrinsic_para_path = "../param/camera_HK.yaml";				// 相机内参文件路径											// 记录相机初始化时间戳
	bool debug_flag = true;														// 是否开启相机调参
}
void Factory::producer()
{
#ifdef VIDEO
	cv::VideoCapture cap("/home/liqianqi/Horizon_InfantryVision-2023/src/thread/blue_buff.mp4");
	cv::Mat src;
	// cv::Mat image;
	//    GxCamera::camera_ptr_->GetMat(image);
	cap >> src;
	std::cout << src.size().width << "   " << src.size().height << std::endl;
	int frame_cnt = 0;
	const std::string &storage_location = "../record/";
	char now[64];
	std::time_t tt;
	struct tm *ttime;
	int width = 1280;
	int height = 1024;
	tt = time(nullptr);
	ttime = localtime(&tt);
	strftime(now, 64, "%Y-%m-%d_%H_%M_%S", ttime); // 以时间为名字
	std::string now_string(now);
	std::string path(std::string(storage_location + now_string).append(".avi"));
	auto writer = cv::VideoWriter(path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 25.0, cv::Size(1350, 1080)); // Avi format
	std::future<void> write_video;
	if (!writer.isOpened())
	{
		cerr << "Could not open the output video file for write\n";
		return;
	}
	if (!cap.isOpened())
	{
		return;
	}
	for (;;)
	{
		while (image_buffer_front_ - image_buffer_rear_ > IMGAE_BUFFER)
			;
		cap >> image_buffer_[image_buffer_front_ % IMGAE_BUFFER];
		src = image_buffer_[image_buffer_front_ % IMGAE_BUFFER];

		frame_cnt++;
		if (frame_cnt % 10 == 0)
		{
			frame_cnt = 0;
			// 异步读写加速,避免阻塞生产者
			write_video = std::async(std::launch::async, [&, src]()
									 { writer.write(src); });
		}
		if (src.empty())
			break;

		image_buffer_front_++;
	}
#endif
#ifdef HK
	auto t0 = std::chrono::steady_clock::now(); // 记录相机初始化时间戳
	while (true)
	{
		if (HKcamera::MVS_cap != nullptr)
		{
			while (image_buffer_front_ - image_buffer_rear_ > IMGAE_BUFFER)
			{
				// std::cout << image_buffer_front_ - image_buffer_rear_ << std::endl;
			};
			if (HKcamera::MVS_cap->ReadImg(image_buffer_[image_buffer_front_ % IMGAE_BUFFER])) // 相机取图
			{
				auto t2 = std::chrono::steady_clock::now();
				std::chrono::duration<double> time_run = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t0);
				// HKcamera::MVS_cap->undistProcess(image); // 相机畸变矫正示例(取消注释即可使用)
				timer_buffer_[image_buffer_front_ % IMGAE_BUFFER] = time_run.count();
				++image_buffer_front_;
			}
			else
			{
				delete HKcamera::MVS_cap;
				HKcamera::MVS_cap = nullptr;
			}

#ifdef SAVE_VIDEO
			frame_cnt++;
			cv::Mat src = image_buffer_[image_buffer_front_ % IMGAE_BUFFER];
			if (frame_cnt % 10 == 0)
			{
				frame_cnt = 0;
				// 异步读写加速,避免阻塞生产者
				write_video = std::async(std::launch::async, [&, src]()
										 { writer.write(src); });
			}
#endif
		}
		else
		{
			HKcamera::MVS_cap = new HikCamera;
			HKcamera::MVS_cap->Init(HKcamera::debug_flag, HKcamera::camera_config_path, HKcamera::intrinsic_para_path, t0); // 初始化相机，第一个参数为 动态调节相机参数模式
			HKcamera::MVS_cap->CamInfoShow();																				// 显示图像参数信息
			image_buffer_front_ = 0;
			image_buffer_rear_ = 0;
		}
	}
#endif
#ifdef DAHENG

#ifdef SAVE_VIDEO
	cv::Mat image;
	//    GxCamera::camera_ptr_->GetMat(image);
	int frame_cnt = 0;
	const std::string &storage_location = "/media/lsn/wheeltec/home/wheeltec/Videos";
	char now[64];
	std::time_t tt;
	struct tm *ttime;
	int width = 1280;
	int height = 1024;
	tt = time(nullptr);
	ttime = localtime(&tt);
	strftime(now, 64, "%Y-%m-%d_%H_%M_%S", ttime); // 以时间为名字
	std::string now_string(now);
	std::string path(std::string(storage_location + now_string).append(".avi"));
	auto writer = cv::VideoWriter(path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 25.0, cv::Size(width, height)); // Avi format
	std::future<void> write_video;
	if (!writer.isOpened())
	{
		cerr << "Could not open the output video file for write\n";
		return;
	}
#endif
	std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
	while (true)
	{
		if (GxCamera::camera_ptr_ != nullptr)
		{
			while (image_buffer_front_ - image_buffer_rear_ > IMGAE_BUFFER)
			{
				// std::cout << image_buffer_front_ - image_buffer_rear_ << std::endl;
			};
			if (GxCamera::camera_ptr_->GetMat(image_buffer_[image_buffer_front_ % IMGAE_BUFFER]))
			{
				std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
				std::chrono::duration<double> time_run = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t0);

				timer_buffer_[image_buffer_front_ % IMGAE_BUFFER] = time_run.count();
				++image_buffer_front_;
#ifdef SAVE_VIDEO
				frame_cnt++;
				cv::Mat src = image_buffer_[image_buffer_front_ % IMGAE_BUFFER];
				if (frame_cnt % 10 == 0)
				{
					frame_cnt = 0;
					// 异步读写加速,避免阻塞生产者
					write_video = std::async(std::launch::async, [&, src]()
											 { writer.write(src); });
				}
#endif
			}
			else
			{
				delete GxCamera::camera_ptr_;
				GxCamera::camera_ptr_ = nullptr;
			}
		}
		else
		{
			GxCamera::camera_ptr_ = new DaHengCamera;
			while (!GxCamera::camera_ptr_->StartDevice())
			{
			};
			GxCamera::camera_ptr_->SetResolution();
			while (!GxCamera::camera_ptr_->StreamOn())
			{
			};

			// 设置是否自动白平衡
			GxCamera::camera_ptr_->Set_BALANCE_AUTO(1);
			// 手动设置白平衡通道及系数，此之前需关闭自动白平衡
			GxCamera::camera_ptr_->SetExposureTime(GxCamera::GX_exp_time);
			GxCamera::camera_ptr_->SetGain(3, GxCamera::GX_gain);

			double GX_Gamma = 2.85;
			GxCamera::camera_ptr_->setGamma(GX_Gamma);
#ifdef cam_show
			cv::namedWindow("DaHengCameraDebug", cv::WINDOW_AUTOSIZE);
			cv::createTrackbar("DaHengExpTime", "DaHengCameraDebug", &GxCamera::GX_exp_time, 10000, GxCamera::DaHengSetExpTime);
			GxCamera::DaHengSetExpTime(0, nullptr);
			cv::createTrackbar("DaHengGain", "DaHengCameraDebug", &GxCamera::GX_gain, 10, GxCamera::DaHengSetGain);
			GxCamera::DaHengSetGain(0, nullptr);
#endif
			image_buffer_front_ = 0;
			image_buffer_rear_ = 0;
		}
	}
#endif

#ifdef MIDVISION

	std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
	while (true)
	{
		if (MidCamera::camera_ptr_ != nullptr)
		{
			while (image_buffer_front_ - image_buffer_rear_ > IMGAE_BUFFER)
			{
				// std::cout << "image_buffer_front_:" << image_buffer_front_ << std::endl;
				// std::cout << "image_buffer_rear_:" << image_buffer_rear_ << std::endl;
			};
			std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

			if (MidCamera::camera_ptr_->GetMat(image_buffer_[image_buffer_front_ % IMGAE_BUFFER]))
			{
				std::chrono::duration<double> time_run = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);

				MidCamera::camera_ptr_->SetExpose(MidCamera::MV_exp_value);

				timer_buffer_[image_buffer_front_ % IMGAE_BUFFER] = time_run.count();
				// cv::imshow("windowName", image_buffer_[image_buffer_front_ % IMGAE_BUFFER]);
				++image_buffer_front_;
			}
			else
			{
				delete MidCamera::camera_ptr_;
				MidCamera::camera_ptr_ = nullptr;
			}
			// image_mutex_.unlock();
		}
		else
		{
			MidCamera::camera_ptr_ = new MVCamera;
			MidCamera::camera_ptr_->SetExpose(MidCamera::MV_exp_value);
#ifdef cam_show
			cv::namedWindow("MVCameraDebug", cv::WINDOW_AUTOSIZE);
			cv::createTrackbar("MVExpTime", "MVCameraDebug", &MidCamera::MV_exp_value, 15000, MidCamera::MVSetExpTime);
			MidCamera::MVSetExpTime(0, nullptr);
#endif
			image_buffer_front_ = 0;
			image_buffer_rear_ = 0;
		}
	}
#endif
}

void Factory::consumer()
{
	TRTModule trtmodel("/home/lsn/RMCV_TRT_Lsn/model/2023-04-16-best.engine");
	while (true)
	{
		// 若满足这个条件，则让这个函数一只停在这里
		while (image_buffer_front_ <= image_buffer_rear_)
		{
		};
		std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now(); // time_of_process

		// 读取最新的图片
		image_buffer_rear_ = image_buffer_front_ - 1;
		// 直接获取引用
		cv::Mat &img = image_buffer_[image_buffer_rear_ % IMGAE_BUFFER];
		double src_time = timer_buffer_[image_buffer_rear_ % IMGAE_BUFFER];

		stm32data.flag = 0; // 状态位，0为自瞄，1为吊射，可自己实现
		serial_mutex_.lock();
		stm32data = TimeSynchronization(MCU_data_, src_time);

		imu_data.yaw = stm32data.yaw_data_.f;
		imu_data.pitch = stm32data.pitch_data_.f;
		serial_mutex_.unlock();
#ifdef Debug_yaml
		cv::FileStorage fs(debug_yaml, cv::FileStorage::READ);
		string debug_color;
		bool outpost_state;
		float fire_v0;

		fs["color"] >> debug_color;
		fs["outpost_state"] >> outpost_state;
		fs["fire_v0"] >> fire_v0;
		if (debug_color == "red")
		{
			trtmodel.color_id = COLOR::RED;
		}
		else if (debug_color == "blue")
		{
			trtmodel.color_id = COLOR::BLUE;
		}
		else
		{
			trtmodel.color_id = COLOR::GRAY;
		}
		trtmodel.State_outpost = outpost_state;
		predic_pose_->v0 = fire_v0;
		fs.release();
#endif
#ifdef Release
		predic_pose_->v0 = stm32data.init_firing_rate;
		if (stm32data.state_outpost)
		{
			trtmodel.State_outpost = true;
		}
		else
		{
			trtmodel.State_outpost = false;
		}

		if (stm32data.color_)
		{
			trtmodel.color_id = COLOR::BLUE;
		}
		else
		{
			trtmodel.color_id = COLOR::RED;
		}
#endif

		// 获取所有的装甲板信息
		cv::FileStorage trt(debug_yaml, cv::FileStorage::READ);
		float confidence, nms_threshold;
		trt["confidence"] >> confidence;
		trt["nms_threshold"] >> nms_threshold;
		auto armors = trtmodel(img, confidence, nms_threshold);
		trt.release();
		serial_mutex_.lock();
		if (armors.size() != 0)
		{
			predic_pose_->state_ = ARMOR_STATE_::TRACK;
		}
		if (predic_pose_->state_ != ARMOR_STATE_::LOSS)
		{
			gim = predic_pose_->run(imu_data, armors, src_time);
			coord = predic_pose_->last_pose_.first;
			rotation = predic_pose_->last_pose_.second;
			visiondata.is_have_armor = true;
		}
		else
		{
			visiondata.is_have_armor = false;
		}
		visiondata.yaw_data_.f = gim.yaw + 0.6;
		visiondata.pitch_data_.f = gim.pitch - 0.3;
		// 发送时间戳，记录为得到图像处理前的时间
		visiondata.time.f = src_time;
		datacontroler.sentData(fd, visiondata);
		serial_mutex_.unlock();
#ifdef Debug_image
		char test[100];
		sprintf(test, "x:%0.4f", coord[0]);
		cv::putText(img, test, cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 8);

		sprintf(test, "y:%0.4f", coord[1]);
		cv::putText(img, test, cv::Point(img.cols / 3, 80), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 8);

		sprintf(test, "z:%0.4f", coord[2]);
		cv::putText(img, test, cv::Point(2 * img.cols / 3, 120), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 8);

		// sprintf(test, "roll:%0.4f", rotation[0] * 180 / CV_PI);
		// cv::putText(img, test, cv::Point(10, 160), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 8);

		// sprintf(test, "pitch:%0.4f", rotation[1] * 180 / CV_PI);
		// cv::putText(img, test, cv::Point(img.cols / 3, 200), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 8);

		// sprintf(test, "yaw:%0.4f", rotation[2] * 180 / CV_PI);
		// cv::putText(img, test, cv::Point(2 * img.cols / 3, 240), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 8);

		sprintf(test, "get yaw:%0.4f ", stm32data.yaw_data_.f);
		cv::putText(img, test, cv::Point(10, 280), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 8);

		sprintf(test, "get pitch:%0.4f ", stm32data.pitch_data_.f);
		cv::putText(img, test, cv::Point(2 * img.cols / 3, 320), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 8);

		sprintf(test, "send yaw:%0.4f ", visiondata.yaw_data_.f);
		cv::putText(img, test, cv::Point(10, 360), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 8);

		sprintf(test, "send pitch:%0.4f ", visiondata.pitch_data_.f);
		cv::putText(img, test, cv::Point(2 * img.cols / 3, 380), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 8);

		// sprintf(test, "x speed:%0.4f ", predic_pose_->last_velocity_[0] * 100);
		// cv::putText(img, test, cv::Point(img.cols / 2, 460), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 8);
		// sprintf(test, "move is :%0.4f ", predic_pose_->move_);
		// cv::putText(img, test, cv::Point(img.cols / 2, 500), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 8);
		// if (predic_pose_->pnp_solve_->is_large_)
		// {
		// 	sprintf(test, " armor size:%s ", "large");
		// }
		// else
		// {
		// 	sprintf(test, " armor size:%s ", "small");
		// }
		// cv::putText(img, test, cv::Point(2 * img.cols / 3 - 1, 420), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 8);
		// if (visiondata.is_have_armor)
		// {
		// 	sprintf(test, " is have:%s ", "true");
		// }
		// else
		// {
		// 	sprintf(test, " is have:%s ", "false");
		// }
		// cv::putText(img, test, cv::Point(2 * img.cols / 3, 520), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1, 8);

		// Eigen::Vector3d pc = pw_to_pc(predic_pose_->predict_location_, predic_pose_->transform_vector_);
		// Eigen::Matrix3d F;
		// cv2eigen(predic_pose_->pnp_solve_->K_, F);
		// Eigen::Vector3d pu = pc_to_pu(pc, F);
		// cv::circle(img, {int(pu(0, 0)), int(predic_pose_->obj_pixe_.y)}, 5, cv::Scalar(0, 0, 255), 3);
		// if (predic_pose_->flag_switch)
		// {
		// 	cv::circle(img, {200, 200}, 5, cv::Scalar(0, 0, 255), 3);
		// }
		// else
		// {
		// }
		// if (predic_pose_->is_gyro)
		// {
		// 	cv::circle(img, {200, 500}, 5, cv::Scalar(255, 0, 0), 3);

		// 	Eigen::Vector3d pl = pw_to_pc(predic_pose_->left_switch, predic_pose_->transform_vector_);
		// 	Eigen::Matrix3d F1;
		// 	cv2eigen(predic_pose_->pnp_solve_->K_, F1);
		// 	Eigen::Vector3d pul = pc_to_pu(pl, F1);

		// 	cv::circle(img, {int(pul(0, 0)), int(predic_pose_->obj_pixe_.y - 100)}, 5, cv::Scalar(0, 255, 0), 3);
		// 	Eigen::Vector3d pr = pw_to_pc(predic_pose_->right_switch, predic_pose_->transform_vector_);
		// 	Eigen::Matrix3d Fr;
		// 	cv2eigen(predic_pose_->pnp_solve_->K_, Fr);
		// 	Eigen::Vector3d pur = pc_to_pu(pr, Fr);
		// 	cv::circle(img, {int(pur(0, 0)), int(predic_pose_->obj_pixe_.y - 100)}, 5, cv::Scalar(255, 0, 0), 3);
		// }
		// else
		// {
		// }

		std::string windowName = "show";
		cv::namedWindow(windowName, 0);
		cv::imshow(windowName, img);
		cv::waitKey(1);

#endif
		std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
		std::chrono::duration<double> time_run = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

		float FPS = 1 / (time_run.count());

		std::cout << "                 "
				  << "FPS: " << FPS << std::endl;

		// aver_fps.push_back(FPS);
		// if (aver_fps.size() == 99)
		// {
		// 	double sum = 0;
		// 	for (int i = 0; i < 100; i++)
		// 	{
		// 		sum += aver_fps[i];
		// 	}
		// 	std::cout << "                 "
		// 			  << "aver_fps: " << sum / 100 << std::endl;
		// 	aver_fps.clear();
		// }
	}
}

Horizon::DataControler::Stm32Data Factory::TimeSynchronization(std::deque<Horizon::DataControler::Stm32Data> &stm32s, double src_time)
{

	if ((int)stm32s.size() == 0)
	{
		Horizon::DataControler::Stm32Data init;
		return init;
	}
	int index = 0;

	vector<double> scale_time;
	scale_time.reserve(1000);

	for (int i = 0; i < (int)stm32s.size(); i++)
	{
		scale_time[i] = src_time - stm32s[i].time.f;
	}

	for (int i = 0; i < (int)stm32s.size(); i++)
	{
		if (std::abs(scale_time[i]) < std::abs(scale_time[index]))
		{
			index = i;
		}
	}   
       
	Horizon::DataControler::Stm32Data stm32 = stm32s[index];
	stm32data.pitch_data_.f = stm32s[index].pitch_data_.f;
	stm32data.yaw_data_.f = stm32s[index].yaw_data_.f;
	stm32data.time.f = stm32s[index].time.f;
	return stm32;
}

void Factory::Getdata()
{
	fd = OpenPort("/dev/ttyUSB0");
	configureSerial(fd);
	while (1)
	{
		serial_mutex_.lock();
		datacontroler.getData(fd, stm32data_temp);
		// 锁定问题

		if (MCU_data_.size() < mcu_size_)
		{
			MCU_data_.push_back(stm32data_temp);
		}
		else
		{
			MCU_data_.pop_front();
			MCU_data_.push_back(stm32data_temp);
		}
		serial_mutex_.unlock();
	}
}
