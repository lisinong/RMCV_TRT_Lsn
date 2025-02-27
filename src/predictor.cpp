
#include "../include/predictor.h"

#define SIN_POINT_NUM 400
float SavePoint[SIN_POINT_NUM];	   // 保存点位
float SecSavePoint[SIN_POINT_NUM]; // 保存点位
int Times = 0;

void drawCurveData(cv::Point3f point)
{

	using namespace cv;
	Mat poly_background_src_ = cv::Mat::zeros(640, 512, CV_8UC3);
	poly_background_src_.setTo(cv::Scalar(0, 255, 0));

	SavePoint[Times % SIN_POINT_NUM] = point.x * 100;

	char test[100];
	sprintf(test, "vx:%0.4f", point.x * 100);
	cv::putText(poly_background_src_, test, cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 1, 8);

	if (point.x * 100 > 30)
	{
		sprintf(test, " vx large:%s ", "true");
	}
	else
	{
		sprintf(test, " vx large:%s ", "false");
	}
	cv::putText(poly_background_src_, test, cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 0, 0), 1, 8);

	// 找最大值的方法只用循环一次就可以
	float maxData = 0;
	for (int i = 0; i <= Times && i < SIN_POINT_NUM; i++)
	{
		if (fabs(SavePoint[i]) > maxData)
		{
			maxData = fabs(SavePoint[i]);
		}
	}
	// 计算倍率
	float bei = 10;
	if (10 * maxData > poly_background_src_.rows / 2)
	{
		bei = (poly_background_src_.rows / 2 - 10) / maxData;
	}
	if (10 * maxData < 150 && maxData != 0)
	{
		bei = 150.0 / maxData;
	}
	std::vector<cv::Point> points;
	// 存入当前记录位置的后续单位,靠前的位置
	int t = 0;
	for (int i = (Times + 1) % SIN_POINT_NUM; i <= Times && i < SIN_POINT_NUM; i++)
	{
		points.push_back(Point((float)poly_background_src_.cols / SIN_POINT_NUM * t, poly_background_src_.rows / 2 + bei * SavePoint[i]));
		t++;
	}
	// 绘制折线
	// cv::polylines(poly_background_src_, points, false, cv::Scalar(255, 0, 0), 1, 8, 0);
	// 存入之前的元素
	for (int i = 0; i <= Times % SIN_POINT_NUM; i++)
	{
		points.push_back(Point((float)poly_background_src_.cols / SIN_POINT_NUM * t, poly_background_src_.rows / 2 + bei * SavePoint[i]));
		t++;
	}

	cv::polylines(poly_background_src_, points, false, cv::Scalar(0, 0, 255), 3, 8, 0);

	// cv::polylines(poly_background_src_, points, false, cv::Scalar(255, 0, 0), 3, 8, 0);

	string windowName = "波形图-预测前";
	namedWindow(windowName, 0);
	imshow(windowName, poly_background_src_);
	Times++;
}

PnpSolver::PnpSolver(const string yaml)
{
	cv::FileStorage fs(cam_yaml, cv::FileStorage::READ);
	fs["M1"] >> K_;
	fs["D1"] >> distCoeffs_;
	fs.release();
}

/**
 * @brief 将旋转矩阵转化为欧拉角
 * @param R 旋转矩阵
 * @return 欧拉角
 */
Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix3d &R)
{
	double sy = sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
	bool singular = sy < 1e-6;
	double x, y, z;
	if (!singular)
	{
		x = atan2(R(2, 1), R(2, 2));
		y = atan2(-R(2, 0), sy);
		z = atan2(R(1, 0), R(0, 0));
	}
	else
	{
		x = atan2(-R(1, 2), R(1, 1));
		y = atan2(-R(2, 0), sy);
		z = 0;
	}
	return {z, y, x};
}

Eigen::Vector3d get_euler_angle(cv::Mat rotation_vector)
{
	double theta = cv::norm(rotation_vector, cv::NORM_L2);

	double w = std::cos(theta / 2);
	double x = std::sin(theta / 2) * rotation_vector.ptr<double>(0)[0] / theta;
	double y = std::sin(theta / 2) * rotation_vector.ptr<double>(1)[0] / theta;
	double z = std::sin(theta / 2) * rotation_vector.ptr<double>(2)[0] / theta;

	double ysqr = y * y;
	// pitch (x-axis rotation)
	double t0 = 2.0 * (w * x + y * z);
	double t1 = 1.0 - 2.0 * (x * x + ysqr);
	double pitch = std::atan2(t0, t1);

	// yaw (y-axis rotation)
	double t2 = 2.0 * (w * y - z * x);
	if (t2 > 1.0)
	{
		t2 = 1.0;
	}
	if (t2 < -1.0)
	{
		t2 = -1.0;
	}
	double yaw = std::asin(t2);

	// roll (z-axis rotation)
	double t3 = 2.0 * (w * z + x * y);
	double t4 = 1.0 - 2.0 * (ysqr + z * z);
	double roll = std::atan2(t3, t4);

	return {roll, yaw, pitch};
}

/**
 * @brief:  位姿解算器
 *
 * @author: liqianqi
 *
 * @param:  obj: 装甲板信息，主要用四点
 *
 * @return: 装甲板在相机系的位置和姿态
 */
std::pair<Eigen::Vector3d, Eigen::Vector3d> PnpSolver::poseCalculation(Object &obj)
{

	std::cout << "[pose_solver] poseCalculation" << std::endl;
	std::vector<cv::Point3f> point_in_world; // 装甲板世界坐标系
	float width_height_ratio = std::sqrt(std::pow(obj.pts[3].x - obj.pts[0].x, 2) + std::pow(obj.pts[3].y - obj.pts[0].y, 2)) / std::sqrt(std::pow(obj.pts[1].x - obj.pts[0].x, 2) + std::pow(obj.pts[1].y - obj.pts[0].y, 2));
	std::cout << "=======================" << std::endl;
	std::vector<cv::Point2f> point_in_pixe; // 像素坐标系
	for (auto pt : obj.pts)
	{
		point_in_pixe.emplace_back(pt);
	}
	std::cout << "pixe_[x,y]  " << point_in_pixe[0].x << " " << point_in_pixe[0].y << std::endl;
	std::cout << "pixe_[x,y]  " << point_in_pixe[1].x << " " << point_in_pixe[1].y << std::endl;
	std::cout << "pixe_[x,y]  " << point_in_pixe[2].x << " " << point_in_pixe[2].y << std::endl;
	std::cout << "pixe_[x,y]  " << point_in_pixe[3].x << " " << point_in_pixe[3].y << std::endl;
	cv::FileStorage fs(debug_yaml, cv::FileStorage::READ);
	if (width_height_ratio < 2.7)
	{
		std::cout << "[notice] the small armor" << std::endl;
		is_large_ = false;
		float kRealSmallArmorWidth, kRealSmallArmorHeight;
		fs["small_armor"]["width"] >> kRealSmallArmorWidth;
		fs["small_armor"]["height"] >> kRealSmallArmorHeight;
		float fHalfX = kRealSmallArmorWidth * 0.5f;					   // 将装甲板的宽的一半作为原点的x
		float fHalfY = kRealSmallArmorHeight * 0.5f;				   // 将装甲板的宽的一半作为原点的y
		point_in_world.emplace_back(cv::Point3f(-fHalfX, -fHalfY, 0)); // 左下
		point_in_world.emplace_back(cv::Point3f(-fHalfX, fHalfY, 0));  // 左上
		point_in_world.emplace_back(cv::Point3f(fHalfX, fHalfY, 0));   // 右上
		point_in_world.emplace_back(cv::Point3f(fHalfX, -fHalfY, 0));  // 右下
	}
	else
	{
		std::cout << "[notice] the large armor" << std::endl;
		is_large_ = true;
		float kRealLargeArmorWidth, kRealLargeArmorHeight;
		fs["large_armor"]["width"] >> kRealLargeArmorWidth;
		fs["large_armor"]["height"] >> kRealLargeArmorHeight;
		float fHalfX = kRealLargeArmorWidth * 0.5f;	 // 将装甲板的宽的一半作为原点的x
		float fHalfY = kRealLargeArmorHeight * 0.5f; // 将装甲板的宽的一半作为原点的y
		point_in_world.emplace_back(cv::Point3f(-fHalfX, -fHalfY, 0));
		point_in_world.emplace_back(cv::Point3f(-fHalfX, fHalfY, 0));
		point_in_world.emplace_back(cv::Point3f(fHalfX, fHalfY, 0));
		point_in_world.emplace_back(cv::Point3f(fHalfX, -fHalfY, 0));
	}
	fs.release();

	if (point_in_world.size() != 4 && point_in_pixe.size() != 4)
	{
		std::cout << "[world] size " << point_in_world.size() << std::endl;
		std::cout << "[pixe] size " << point_in_pixe.size() << std::endl;
	}

	cv::Mat rvecs = cv::Mat::zeros(3, 1, CV_64FC1);
	cv::Mat tvecs = cv::Mat::zeros(3, 1, CV_64FC1);

	// 世界坐标系到相机坐标系的变换
	// tvecs 表示从相机系到世界系的平移向量并在相机系下的坐标
	// rvecs 表示从相机系到世界系的旋转向量，需要做进一步的转换
	// 默认迭代法: SOLVEPNP_ITERATIVE，最常规的用法，精度较高，速度适中，一次解算一次耗时不到1ms
	cv::solvePnP(point_in_world, point_in_pixe, K_, distCoeffs_, rvecs, tvecs, cv::SOLVEPNP_ITERATIVE);

	cv::Mat rotM = cv::Mat::zeros(3, 3, CV_64FC1); // 解算出来的旋转矩阵

	cv::Rodrigues(rvecs, rotM);

	rotate_world_cam_ = rotM;

	Eigen::Matrix3d rotM_eigen;

	cv2eigen(rotM, rotM_eigen);
	// 将旋转矩阵分解为三个轴的欧拉角（roll、pitch、yaw）
	// Eigen::Vector3d euler_angles = rotationMatrixToEulerAngles(rotM_eigen);
	Eigen::Vector3d euler_angles = get_euler_angle(rvecs);

	// 这需要具体测量
	// double roll = euler_angles.at<double>(0);  // X-world-axis 与 X-cam-axis 在yoz上的夹角
	// double pitch = euler_angles.at<double>(1); // Y-world-axis 与 Y-cam-axis 在xoz上的夹角
	// double yaw = euler_angles.at<double>(2);   // Z-world-axis 与 Z-cam-axis 在xoy上的夹角

	double roll = euler_angles[0];	// X-world-axis 与 X-cam-axis 在yoz上的夹角   roll
	double yaw = euler_angles[1];	// Y-world-axis 与 Y-cam-axis 在xoz上的夹角   yaw
	double pitch = euler_angles[2]; // Z-world-axis 与 Z-cam-axis 在xoy上的夹角   pitch

	Eigen::Vector3d coord;
	coord << tvecs.ptr<double>(0)[0] / 100, -tvecs.ptr<double>(0)[1] / 100, tvecs.ptr<double>(0)[2] / 100;

	Eigen::Vector3d rotation;
	rotation << roll, pitch, yaw;

	std::cout << "euler_[roll] " << roll * 180 / CV_PI << "  "
			  << "euler_[pitch] " << pitch * 180 / CV_PI << "  "
			  << "euler_[yaw] " << yaw * 180 / CV_PI << std::endl;

	std::pair<Eigen::Vector3d, Eigen::Vector3d> pose;
	pose.first = coord;
	pose.second = rotation;

	std::cout << "[tx]: " << tvecs.ptr<double>(0)[0] / 100 << "  "
			  << "[ty]: " << -tvecs.ptr<double>(0)[1] / 100 << "  [tz]: " << tvecs.ptr<double>(0)[2] / 100 << std::endl;

	// 为什么要传相机系的旋转角，因为当无法上车调试时，可以将就着调
	std::cout << "camera pose finished!!!" << std::endl;
	std::cout << "=======================" << std::endl;
	return pose;
}

/**
 * @brief:  相机系到云台系的姿态
 *
 * @author: liqianqi
 *
 * @param:  cam_coord是相机系的坐标, rotate_world_cam是从世界系转相机系的旋转矩阵
 *
 * @return: 装甲板在云台系的位置和姿态
 */
std::pair<Eigen::Vector3d, Eigen::Vector3d> PredictorPose::cam2ptz(std::pair<Eigen::Vector3d, Eigen::Vector3d> &cam_coord, cv::Mat &rotate_world_cam)
{
	cv::FileStorage fs(debug_yaml, cv::FileStorage::READ);
	float X_BIAS, Y_BIAS, Z_BIAS;
	fs["X_BIAS"] >> X_BIAS;
	fs["Y_BIAS"] >> Y_BIAS;
	fs["Z_BIAS"] >> Z_BIAS;
	cam_coord.first[0] = cam_coord.first[0] + X_BIAS;
	cam_coord.first[1] = cam_coord.first[1] + Y_BIAS;
	cam_coord.first[2] = cam_coord.first[2] + Z_BIAS;
	fs.release();
	Eigen::Matrix3d pitch_rotation_matrix_t;
	pitch_rotation_matrix_t
		<< 1,
		0, 0,
		0, std::cos((imu_data_.pitch * CV_PI) / 180), std::sin((imu_data_.pitch * CV_PI) / 180),
		0, -std::sin((imu_data_.pitch * CV_PI / 180)), std::cos((imu_data_.pitch * CV_PI) / 180);

	Eigen::Matrix3d yaw_rotation_matrix_t;
	yaw_rotation_matrix_t
		<< std::cos((imu_data_.yaw * CV_PI) / 180),
		0, std::sin((imu_data_.yaw * CV_PI) / 180),
		0, 1, 0,
		-std::sin((imu_data_.yaw * CV_PI) / 180), 0, std::cos((imu_data_.yaw * CV_PI) / 180);

	/**
	 * 写两种矩阵的原因是因为位置和姿态所用坐标系不同
	 * 用欧拉较或泰特布莱恩角表示旋转，顺序十分重要，一般是X-Y-Z，但RM这种小角度，绕定轴动轴都一样顺序什么样结果都一样
	 * 可以动手试试
	 */
	Eigen::Vector3d ptz_coord = yaw_rotation_matrix_t * pitch_rotation_matrix_t * cam_coord.first;
	Eigen::Matrix3d transform_vector;
	transform_vector = yaw_rotation_matrix_t * pitch_rotation_matrix_t;
	transform_vector_ = transform_vector.inverse();

	Eigen::Vector3d rotation;
	rotation << cam_coord.second[0], cam_coord.second[1], cam_coord.second[2];

	std::pair<Eigen::Vector3d, Eigen::Vector3d> pose;
	pose.first = ptz_coord;
	pose.second = rotation;

	return pose;
}

/**
 * @brief:  预测主程序
 *
 * @author: liqianqi
 *
 * @param:  imu_data是IMU姿态解算的结果, objects是本帧所有的装甲板，time是图像的时间戳
 *
 * @return: 经过预测后云台上台和偏航的角度
 */
GimbalPose PredictorPose::run(GimbalPose &imu_data, std::vector<Object> &objects, double time)
{
	imu_data_ = imu_data;
	current_time_ = time;

	// Eigen::Matrix3d pitch_rotation_matrix_t;
	// pitch_rotation_matrix_t
	// 	<< 1,
	// 	0, 0,
	// 	0, std::cos((imu_data_.pitch * CV_PI) / 180), std::sin((imu_data_.pitch * CV_PI) / 180),
	// 	0, -std::sin((imu_data_.pitch * CV_PI / 180)), std::cos((imu_data_.pitch * CV_PI) / 180);

	// Eigen::Matrix3d yaw_rotation_matrix_t;
	// yaw_rotation_matrix_t
	// 	<< std::cos((imu_data_.yaw * CV_PI) / 180),
	// 	0, std::sin((imu_data_.yaw * CV_PI) / 180),
	// 	0, 1, 0,
	// 	-std::sin((imu_data_.yaw * CV_PI) / 180), 0, std::cos((imu_data_.yaw * CV_PI) / 180);

	// Eigen::Matrix3d transform_vector;
	// transform_vector = yaw_rotation_matrix_t * pitch_rotation_matrix_t;
	// transform_vector_ = transform_vector.inverse();
	// 如果这一帧没有装甲板，先看看init_是true还是false
	// 是true

	if (!objects.size())
	{
		loss_cnt_++;
		if (loss_cnt_ > 200)
		{
			loss_cnt_ = 200;
		}
		if (loss_cnt_ > 100 || last_state_ == ARMOR_STATE_::LOSS)
		{
			state_ = ARMOR_STATE_::LOSS;
			last_state_ = state_;
			// 返回最后一帧的云台角，避免剧烈晃动
			// return
			init_ = true;
			// velocities_.clear();
			ekf.P = Eigen::Matrix<double, 6, 6>::Identity();
			return last_eular_;
		}
		state_ = ARMOR_STATE_::TRACK;
		last_state_ = state_;
		// 根据上一帧位置和速度在做预测，100次  !!! return
		Eigen::Vector3d current_pose;
		current_pose[0] = last_location_[0] + last_velocity_[0] * (current_time_ - last_time_);
		current_pose[1] = last_location_[1] + last_velocity_[1] * (current_time_ - last_time_);
		current_pose[2] = last_location_[2] + last_velocity_[2] * (current_time_ - last_time_);
		float fly_t = bullteFlyTime(current_pose);

		Eigen::Vector3d predict_pose;
		predict_pose[0] = current_pose[0] + last_velocity_[0] * fly_t;
		predict_pose[1] = current_pose[1] + last_velocity_[1] * fly_t;
		predict_pose[2] = current_pose[2] + last_velocity_[2] * fly_t;

		bullteFlyTime(current_pose);
		last_time_ = current_time_;
		last_location_ = current_pose;
		predict_location_ = predict_pose;
		GimbalPose gm = gm_ptz;
		last_eular_ = gm;
		return gm;
	}

	loss_cnt_ = 0;
	if (init_)
	{
		std::cout << "init finished!!!" << std::endl;
		init();
		state_ = ARMOR_STATE_::INIT;
		last_state_ = state_;
		// 选择一个装甲板
		float distances[objects.size()];
		for (int i = 0; i < objects.size(); i++)
		{
			cv::Point2f center = (objects[i].pts[0] + objects[i].pts[2]) / 2;
			distances[i] = (center.x - 512) + (center.y - 640);
		}

		int index = 0;
		for (int i = 1; i < objects.size(); i++)
		{
			if (distances[i] < distances[index])
				index = i;
		}
		Object obj = objects[index];
		cout << "[ obj x:" << obj.pts[0].x << obj.pts[0].y << "]" << endl;
		std::pair<Eigen::Vector3d, Eigen::Vector3d> world_cam_pose;
		world_cam_pose = pnp_solve_->poseCalculation(obj);

		std::pair<Eigen::Vector3d, Eigen::Vector3d> cam_ptz_pose;
		cam_ptz_pose = cam2ptz(world_cam_pose, pnp_solve_->rotate_world_cam_);

		std::cout << "[world x_init]: " << cam_ptz_pose.first[0] << "  "
				  << "[world y_init]: " << cam_ptz_pose.first[1] << "  [world z_init]: " << cam_ptz_pose.first[2] << std::endl;

		last_pose_ = cam_ptz_pose; // 记录这一时刻，为下一时刻预测做准备
		init_ = false;
		last_location_ = cam_ptz_pose.first;
		// !!! return
		//float fly_t = bullteFlyTime(cam_ptz_pose.first);
		last_time_ = current_time_;
		GimbalPose gm = gm_ptz;
		velocities_.clear();
		last_eular_ = gm;
		return gm;
	}

	/**
	 * TRCK
	 */
	std::cout << "continue track armor!!!" << std::endl;

	// 选择一个装甲板
	state_ = ARMOR_STATE_::TRACK;
	Object obj = ArmorSelect(objects);

	// float distance_;
	// distance_ = std::sqrt(obj.coord[0] * obj.coord[0] + obj.coord[2] * obj.coord[2]);
	// float yaw_now = std::asin(obj.coord[0] / distance_) * 180 / CV_PI;
	// float distance_last;
	// distance_last = std::sqrt(last_obj.coord[0] * last_obj.coord[0] + last_obj.coord[2] * last_obj.coord[2]);
	// float yaw_last = std::asin(last_obj.coord[0] / distance_last) * 180 / CV_PI;
	// if (isSwitch(obj.coord, last_obj.coord))
	// {
	// 	cnt_switch++;
	// 	flag_switch = true;
	// 	Eigen::Vector3d now_record = obj.coord;
	// 	velocities_.clear();
	// 	if (cnt_switch < 6 && is_gyro == false)
	// 	{
	// 		obj.coord = last_obj.coord + last_velocity_ * (current_time_ - last_time_);
	// 		obj.pyr = last_obj.pyr;
	// 	}
	// 	if (std::abs(current_time_ - last_switch_time_) > 1.0)
	// 	{
	// 		is_gyro_cnt_ = 0;
	// 		last_switch_time_ = current_time_;
	// 	}
	// 	if ((std::abs(current_time_ - last_switch_time_)) > 0.15 && (current_time_ - last_switch_time_) < 1.0)
	// 	{
	// 		is_gyro_cnt_++;
	// 	}
	// 	if (is_gyro_cnt_ > 4)
	// 	{
	// 		is_gyro = true;
	// 		if (yaw_now > yaw_last)
	// 		{
	// 			right_switch = obj.coord;
	// 			left_switch = last_obj.coord;
	// 		}
	// 		else
	// 		{
	// 			right_switch = last_obj.coord;
	// 			left_switch = obj.coord;
	// 		}
	// 	}
	// 	else
	// 	{
	// 		is_gyro = false;
	// 	}
	// 	last_switch_time_ = current_time_;
	// if(current_time_ - last_switch_time_ > 3)
	// {
	// 	last_switch_time_ = current_time_;
	// 	time_buff_.clear();
	// }

	// if((current_time_ - last_switch_time_) > 0.25 && (current_time_ - last_switch_time_) < 0.8)
	// {
	// 	if (time_buff_.size() < 6)
	// 	{
	// 		time_buff_.emplace_back(current_time_);
	// 	}
	// 	else
	// 	{
	// 		time_buff_.pop_front();
	// 		time_buff_.emplace_back(current_time_);
	// 	}
	// 	last_switch_time_ = current_time_;
	// }

	// if(time_buff_.size() > 5)
	// {
	// 	is_gyro_ = true;
	// }else
	// {
	// 	is_gyro_ = false;
	// }
	//}

	// std::cout << "time_buff_ size is: " << time_buff.size() << std::endl;
	// if (!flag_switch)
	// {
	// 	cnt_switch = 0;
	// }
	// if (std::abs(current_time_ - last_switch_time_) > 1.0)
	// {
	// 	is_gyro = false;
	// 	is_gyro_cnt_ = 0;
	// }
	// time_diff = current_time_ - last_switch_time_;

	std::pair<Eigen::Vector3d, Eigen::Vector3d> cam_ptz_pose;
	cam_ptz_pose.first = obj.coord;
	cam_ptz_pose.second = obj.pyr;
	// cam2ptz(cam_ptz_pose, pnp_solve_->rotate_world_cam_);
	// cam_ptz_pose = world_cam_pose;

	std::cout << "[world x]: " << cam_ptz_pose.first[0] << "  "
			  << "[world y]: " << cam_ptz_pose.first[1] << "  [world z]: " << cam_ptz_pose.first[2] << std::endl;

	if (std::sqrt(std::pow(cam_ptz_pose.first[0] - last_location_[0], 2) + std::pow(cam_ptz_pose.first[1] - last_location_[1], 2) + std::pow(cam_ptz_pose.first[2] - last_location_[2], 2)) >= 0.8)
	{
		init_ = true;
		return last_eular_;
	}

	//========================EKF========================//

	std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();

	Predict predictfunc;
	Measure measure;

	if (isnan(ekf.P.sum()))
	{
		ekf.P = Eigen::Matrix<double, 6, 6>::Identity();
	}

	Eigen::Matrix<double, 6, 1> Xh;
	Xh << last_location_[0], last_velocity_[0], last_location_[1], last_velocity_[1], last_location_[2], last_velocity_[2];
	ekf.init(Xh);
	Eigen::Matrix<double, 6, 1> Xr;
	Xr << cam_ptz_pose.first(0, 0), 0, cam_ptz_pose.first(1, 0), 0, cam_ptz_pose.first(2, 0), 0;
	Eigen::Matrix<double, 3, 1> Yr;
	measure(Xr.data(), Yr.data()); // 转化成pitch,yaw,distance
	predictfunc.delta_t = (current_time_ - last_time_);
	ekf.predict(predictfunc);								  // 更新预测器，此时预测器里的是预测值
	Eigen::Matrix<double, 6, 1> Xe = ekf.update(measure, Yr); // 更新滤波器，输入真实的球面坐标 Yr

	if (!isnan(Xe[0]) && !isnan(Xe[2]) && !isnan(Xe[4]))
	{
		// cam_ptz_pose.first[0] = Xe[0];
		// cam_ptz_pose.first[1] = Xe[2];
		// cam_ptz_pose.first[2] = Xe[4];
	}
	else
	{
		ekf.P = Eigen::Matrix<double, 6, 6>::Identity();
	}

	std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
	std::chrono::duration<double> time_run = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t0);

	std::cout << "[EKF time is : " << time_run.count() * 1000 << " ]" << std::endl;

	//========================EKF========================//

	std::cout << "world coord solve finished" << std::endl;

	float fly_t = bullteFlyTime(cam_ptz_pose.first);

	std::cout << "the first fly time geted" << std::endl;

	Eigen::Vector4d current_state; // 现在的速度: 直接计算的速度，未经过拟合
	current_state[0] = cam_ptz_pose.first[0];
	current_state[1] = cam_ptz_pose.first[1];
	current_state[2] = cam_ptz_pose.first[2];
	current_state[3] = current_time_;

	Eigen::Vector3d now_v;
	if (velocities_.size() < velocities_deque_size_)
	{
		velocities_.push_back(current_state);
	}
	else
	{
		velocities_.pop_front();
		velocities_.push_back(current_state);
	}
	now_v = CeresVelocity(velocities_);

	if (isnan(now_v[0]) || isnan(now_v[1]) || isnan(now_v[2]))
	{
		now_v = last_velocity_;
	}
	// if( now_v[0]*last_velocity_[0] < 0 && v_count < 50)
	// {
	// 	v_count++;
	// 	last_velocity_ = now_v;
	// }else
	// {
	// 	v_count = 0;
	// }

	//now_v[0] = 0;
	now_v[1] = 0;
	now_v[2] = 0;
	cv::Point3f point;
	point.x = now_v[0];
	  drawCurveData(point);

	Eigen::Vector3d predict_location;

	std::cout << "[ vx: " << now_v[0] << " vy: " << now_v[1] << " vz: " << now_v[2] << " ]" << std::endl;

	predict_location[0] = cam_ptz_pose.first[0] + (now_v[0] * (fly_t));
	predict_location[1] = cam_ptz_pose.first[1] + (now_v[1] * (fly_t));
	predict_location[2] = cam_ptz_pose.first[2] + (now_v[2] * (fly_t));

	is_gyro = false;
	if (is_gyro)
	{
		float distance_;
		distance_ = std::sqrt(predict_location[0] * predict_location[0] + predict_location[2] * predict_location[2]);
		float yaw_pre = std::asin(predict_location[0] / distance_) * 180 / CV_PI;

		float distance_right;
		distance_right = std::sqrt(right_switch[0] * right_switch[0] + right_switch[2] * right_switch[2]);
		float yaw_right = std::asin(right_switch[0] / distance_right) * 180 / CV_PI;

		float distance_left;
		distance_left = std::sqrt(left_switch[0] * left_switch[0] + left_switch[2] * left_switch[2]);
		float yaw_left = std::asin(left_switch[0] / distance_left) * 180 / CV_PI;

		if (yaw_pre > yaw_right)
		{
			predict_location = left_switch;
		}
		else if (yaw_pre < yaw_left)
		{
			predict_location = right_switch;
		}
	}
	std::cout << "predict expression " << (now_v[0] * (fly_t)) << " cm " << std::endl;
	move_ = now_v[0] * (fly_t) * 100;

	bullteFlyTime(predict_location);

	last_location_ = cam_ptz_pose.first;
	last_time_ = current_time_;
	last_velocity_ = now_v;
	last_state_ = state_;
	last_pose_ = cam_ptz_pose;
	predict_location_ = predict_location;
	last_obj = obj;

	GimbalPose gm = gm_ptz;
	gm.yaw = gm.yaw;
	gm.pitch = gm.pitch;
	last_eular_ = gm;
	return gm;
}

/**
 * @brief  判断弹道的飞行时间
 *
 * @param  三维坐标
 *
 * @return 飞行的时间
 */
float PredictorPose::bullteFlyTime(Eigen::Vector3d coord)
{
	cv::Point3f p1;

	p1.x = coord[0];
	p1.y = coord[1];
	p1.z = coord[2];

	float g = 9.8;

	// x1,y1,z1;
	// 先算yaw的值的
	float distance1;
	distance1 = std::sqrt(p1.x * p1.x + p1.z * p1.z);
	gm_ptz.yaw = std::asin(p1.x / distance1) * 180 / CV_PI;

	if (p1.z < 0 && p1.x >= 0)
	{
		gm_ptz.yaw = 2 * (90 - gm_ptz.yaw) + gm_ptz.yaw;
	}
	else if (p1.z < 0 && p1.x < 0)
	{
		gm_ptz.yaw = 2 * (-90 - gm_ptz.yaw) + gm_ptz.yaw;
	}

	// pitch值
	float a = -0.5 * g * (std::pow(distance1, 2) / std::pow(v0, 2));
	float b = distance1;
	float c = a - p1.y;

	float Discriminant = std::pow(b, 2) - 4 * a * c; // 判别式
	if (Discriminant < 0)
		return -1;
	float tan_angle1 = (-b + std::pow(Discriminant, 0.5)) / (2 * a); //*180/CV_PI;
	float tan_angle2 = (-b - std::pow(Discriminant, 0.5)) / (2 * a);

	float angle1 = std::atan(tan_angle1) * 180 / CV_PI; // 角度制
	float angle2 = std::atan(tan_angle2) * 180 / CV_PI; // 角度制

	if (tan_angle1 >= -3 && tan_angle1 <= 3)
	{
		// cout << "angle1     " << angle1 << endl;
		gm_ptz.pitch = angle1;
	}
	else if (tan_angle2 >= -3 && tan_angle2 <= 3)
	{
		gm_ptz.pitch = angle2;
	}

	float PI_pitch = (gm_ptz.pitch / 180) * CV_PI; // 弧度制

	return distance1 / (v0 * std::cos(PI_pitch));
}

Object PredictorPose::ArmorSelect(std::vector<Object> &objects)
{

	for (int i = 0; i < objects.size(); i++)
	{
		std::pair<Eigen::Vector3d, Eigen::Vector3d> world_cam_pose;
		world_cam_pose = pnp_solve_->poseCalculation(objects[i]);

		std::pair<Eigen::Vector3d, Eigen::Vector3d> cam_ptz_pose;
		cam_ptz_pose = cam2ptz(world_cam_pose, pnp_solve_->rotate_world_cam_);

		objects[i].coord = cam_ptz_pose.first;
		objects[i].pyr = cam_ptz_pose.second;
	}

	float distances[objects.size()];
	for (int i = 0; i < objects.size(); i++)
	{
		distances[i] = std::sqrt(std::pow(objects[i].coord[0], 2) + std::pow(objects[i].coord[1], 2) + std::pow(objects[i].coord[2], 2));
	}

	float last_pose = std::sqrt(std::pow(last_pose_.first[0], 2) + std::pow(last_pose_.first[1], 2) + std::pow(last_pose_.first[2], 2));

	float distances_residual[objects.size()];

	for (int i = 1; i < objects.size(); i++)
	{
		distances_residual[i] = std::abs(distances[i] - last_pose);
	}

	int index = 0;
	for (int i = 1; i < objects.size(); i++)
	{
		if (distances_residual[i] < distances_residual[index])
			index = i;
	}

	return objects[index];
}

Eigen::Vector3d PredictorPose::CeresVelocity(std::deque<Eigen::Vector4d> velocities) // 最小二乘法拟合速度
{
	int N = velocities.size();
	if (velocities.size() < 4)
	{
		return last_velocity_;
	}

	double avg_x = 0;
	double avg_x2 = 0;
	double avg_f = 0;
	double avg_xf = 0;

	double time_first = velocities.front()[3];

	for (int i = 0; i < N; i++)
	{
		avg_x += velocities[i][3] - time_first;
		avg_x2 += std::pow(velocities[i][3] - time_first, 2);
		avg_f += velocities[i][0];
		avg_xf += (velocities[i][3] - time_first) * velocities[i][0];
	}

	double vx = (avg_xf - N * (avg_x / N) * (avg_f / N)) / (avg_x2 - N * std::pow(avg_x / N, 2));

	avg_x = 0;
	avg_x2 = 0;
	avg_f = 0;
	avg_xf = 0;
	for (int i = 0; i < N; i++)
	{
		avg_x += velocities[i][3] - time_first;
		avg_x2 += std::pow(velocities[i][3] - time_first, 2);
		avg_f += velocities[i][1];
		avg_xf += (velocities[i][3] - time_first) * velocities[i][1];
	}
	double vy = (avg_xf - N * (avg_x / N) * (avg_f / N)) / (avg_x2 - N * std::pow(avg_x / N, 2));

	double avg_x_ = 0;
	double avg_x2_ = 0;
	double avg_f_ = 0;
	double avg_xf_ = 0;
	for (int i = 0; i < N; i++)
	{
		avg_x_ += velocities[i][3] - time_first;
		avg_x2_ += std::pow(velocities[i][3] - time_first, 2);
		avg_f_ += velocities[i][2];
		avg_xf_ += (velocities[i][3] - time_first) * velocities[i][2];
	}
	double vz = (avg_xf_ - N * (avg_x_ / N) * (avg_f_ / N)) / (avg_x2_ - N * std::pow(avg_x_ / N, 2));

	// Vector3d ave_v_;
	// ave_v_[0] = vx;
	// ave_v_[1] = vy;
	// ave_v_[2] = vz;

	// if (ave_v.size() != 0)
	// {
	// double sum_vx, sum_vy, sum_vz;
	// for (int u = 0; u < ave_v.size(); u++)
	// {
	// 	sum_vx += ave_v[u][0];
	// 	sum_vy += ave_v[u][1];
	// 	sum_vz += ave_v[u][2];
	// }
	// double aver_vx = sum_vx / ave_v.size();
	// double aver_vy = sum_vy / ave_v.size();
	// double aver_vz = sum_vz / ave_v.size();

	if (vx * last_velocity_[0] < 0)
	{
		vx = last_velocity_[0];
		v_count++;
	}
	else
	{
		v_count = 0;
	}

	return {vx, vy, vz};
}
