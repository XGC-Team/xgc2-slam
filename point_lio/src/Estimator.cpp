/**
 * @file Estimator.cpp
 * @brief Point-LIO状态估计器实现文件
 *
 * 本文件实现了Point-LIO (Point-based LiDAR-Inertial Odometry)的核心状态估计功能，
 * 主要包括：
 * 1. 基于ESEKF (Error State Extended Kalman Filter)的状态估计
 * 2. 支持两种状态表示：输入状态(input state)和输出状态(output state)
 * 3. 点到平面的距离残差计算和雅可比矩阵推导
 * 4. IMU测量模型和观测方程
 * 5. 外参标定支持
 *
 * 状态估计流程：
 * - 预测阶段：使用IMU数据进行状态预测
 * - 更新阶段：使用LiDAR点云匹配结果进行状态更新
 * - 支持IMU饱和检测和处理
 */

// #include <../include/IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp>
#include "Estimator.h"

// ================== 全局变量定义 ==================

// 点云相关变量
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));          // 存储平面法向量的点云
std::vector<int> time_seq;                                            // 时间序列
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI(10000, 1));   // 降采样后的LiDAR坐标系点云
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI(10000, 1));  // 降采样后的世界坐标系点云
std::vector<V3D> pbody_list;                                          // LiDAR坐标系下的点列表
std::vector<PointVector> Nearest_Points;                              // 最近邻点集合
std::shared_ptr<IVoxType> ivox_ = nullptr;                            // iVox增量式体素地图，用于快速最近邻搜索
std::vector<float> pointSearchSqDis(NUM_MATCH_POINTS);               // 最近邻点的平方距离
bool   point_selected_surf[100000] = {0};                            // 点是否被选为有效平面点的标志
std::vector<M3D> crossmat_list;                                       // 反对称矩阵列表，用于叉乘运算

// 统计变量
int effct_feat_num = 0;                                               // 有效特征点数量
int k = 0;                                                            // 当前处理的时间段索引
int idx = -1;                                                         // 当前处理的点索引

// 卡尔曼滤波器实例
esekfom::esekf<state_input, 24, input_ikfom> kf_input;               // 使用IMU输入的ESEKF滤波器(24维状态)
esekfom::esekf<state_output, 30, input_ikfom> kf_output;             // 使用IMU输出的ESEKF滤波器(30维状态)
input_ikfom input_in;                                                 // IMU输入数据

// IMU数据
V3D angvel_avr, acc_avr, acc_avr_norm;                               // 角速度均值、加速度均值、归一化加速度均值
int feats_down_size = 0;                                              // 降采样后的特征点数量

// 外参：LiDAR相对于IMU的位姿
V3D Lidar_T_wrt_IMU(Zero3d);                                         // LiDAR相对于IMU的平移向量
M3D Lidar_R_wrt_IMU(Eye3d);                                          // LiDAR相对于IMU的旋转矩阵
double G_m_s2 = 9.81;                                                // 重力加速度常量

/**
 * @brief 构建输入状态(input state)的过程噪声协方差矩阵
 *
 * 该函数为基于IMU输入的ESEKF滤波器构建24x24的过程噪声协方差矩阵Q。
 * 过程噪声反映了系统模型的不确定性。
 *
 * 状态向量组成（24维）：
 * - 位置(3) + 旋转(3) + 速度(3) + 角速度偏置(3) + 加速度偏置(3) + 重力(3) + 外参旋转(3) + 外参平移(3)
 *
 * @return Eigen::Matrix<double, 24, 24> 过程噪声协方差矩阵
 */
Eigen::Matrix<double, 24, 24> process_noise_cov_input()
{
	Eigen::Matrix<double, 24, 24> cov;
	cov.setZero();
	// 陀螺仪噪声协方差 (索引3-5: 旋转误差状态)
	cov.block<3, 3>(3, 3).diagonal() << gyr_cov_input, gyr_cov_input, gyr_cov_input;
	// 加速度计噪声协方差 (索引12-14: 速度误差状态)
	cov.block<3, 3>(12, 12).diagonal() << acc_cov_input, acc_cov_input, acc_cov_input;
	// 陀螺仪偏置随机游走协方差 (索引15-17: 陀螺仪偏置)
	cov.block<3, 3>(15, 15).diagonal() << b_gyr_cov, b_gyr_cov, b_gyr_cov;
	// 加速度计偏置随机游走协方差 (索引18-20: 加速度计偏置)
	cov.block<3, 3>(18, 18).diagonal() << b_acc_cov, b_acc_cov, b_acc_cov;
	// MTK::get_cov<process_noise_input>::type cov = MTK::get_cov<process_noise_input>::type::Zero();
	// MTK::setDiagonal<process_noise_input, vect3, 0>(cov, &process_noise_input::ng, gyr_cov_input);// 0.03
	// MTK::setDiagonal<process_noise_input, vect3, 3>(cov, &process_noise_input::na, acc_cov_input); // *dt 0.01 0.01 * dt * dt 0.05
	// MTK::setDiagonal<process_noise_input, vect3, 6>(cov, &process_noise_input::nbg, b_gyr_cov); // *dt 0.00001 0.00001 * dt *dt 0.3 //0.001 0.0001 0.01
	// MTK::setDiagonal<process_noise_input, vect3, 9>(cov, &process_noise_input::nba, b_acc_cov);   //0.001 0.05 0.0001/out 0.01
	return cov;
}

/**
 * @brief 构建输出状态(output state)的过程噪声协方差矩阵
 *
 * 该函数为基于IMU输出的ESEKF滤波器构建30x30的过程噪声协方差矩阵Q。
 * 与输入状态不同，输出状态将角速度和加速度作为状态的一部分进行估计。
 *
 * 状态向量组成（30维）：
 * - 位置(3) + 旋转(3) + 速度(3) + 角速度(3) + 加速度(3) + 重力(3) + 角速度偏置(3) + 加速度偏置(3) + 外参旋转(3) + 外参平移(3)
 *
 * @return Eigen::Matrix<double, 30, 30> 过程噪声协方差矩阵
 */
Eigen::Matrix<double, 30, 30> process_noise_cov_output()
{
	Eigen::Matrix<double, 30, 30> cov;
	cov.setZero();
	// 速度噪声协方差 (索引12-14: 速度状态)
	cov.block<3, 3>(12, 12).diagonal() << vel_cov, vel_cov, vel_cov;
	// 角速度噪声协方差 (索引15-17: 角速度状态)
	cov.block<3, 3>(15, 15).diagonal() << gyr_cov_output, gyr_cov_output, gyr_cov_output;
	// 加速度噪声协方差 (索引18-20: 加速度状态)
	cov.block<3, 3>(18, 18).diagonal() << acc_cov_output, acc_cov_output, acc_cov_output;
	// 陀螺仪偏置随机游走协方差 (索引24-26: 陀螺仪偏置)
	cov.block<3, 3>(24, 24).diagonal() << b_gyr_cov, b_gyr_cov, b_gyr_cov;
	// 加速度计偏置随机游走协方差 (索引27-29: 加速度计偏置)
	cov.block<3, 3>(27, 27).diagonal() << b_acc_cov, b_acc_cov, b_acc_cov;
	return cov;
}

/**
 * @brief 计算输入状态(input state)的状态转移方程 f(x, u)
 *
 * 该函数实现了ESEKF预测阶段的状态转移方程，描述系统状态如何随时间演化。
 * 对于输入状态模型，IMU测量值（角速度和加速度）作为输入u。
 *
 * 状态转移方程：
 * - 位置导数 = 速度
 * - 旋转导数 = 去偏置后的角速度
 * - 速度导数 = 世界坐标系下的加速度 + 重力
 *
 * @param s 当前状态
 * @param in IMU输入数据（角速度和加速度测量值）
 * @return Eigen::Matrix<double, 24, 1> 状态导数向量
 */
Eigen::Matrix<double, 24, 1> get_f_input(state_input &s, const input_ikfom &in)
{
	Eigen::Matrix<double, 24, 1> res = Eigen::Matrix<double, 24, 1>::Zero();
	vect3 omega;
	// 计算去偏置后的角速度：omega = gyro_measurement - bias
	in.gyro.boxminus(omega, s.bg);
	// 将加速度从IMU坐标系转换到世界坐标系，并去除偏置
	vect3 a_inertial = s.rot * (in.acc-s.ba); // .normalized()
	for(int i = 0; i < 3; i++ ){
		res(i) = s.vel[i];                        // 位置的导数 = 速度
		res(i + 3) = omega[i];                    // 旋转的导数 = 角速度
		res(i + 12) = a_inertial[i] + s.gravity[i]; // 速度的导数 = 加速度 + 重力
	}
	return res;
}

/**
 * @brief 计算输出状态(output state)的状态转移方程 f(x, u)
 *
 * 该函数实现了输出状态模型的状态转移方程。
 * 与输入状态不同，输出状态将角速度和加速度作为状态的一部分，而非输入。
 *
 * 状态转移方程：
 * - 位置导数 = 速度
 * - 旋转导数 = 状态中的角速度
 * - 速度导数 = 世界坐标系下的加速度 + 重力
 *
 * @param s 当前状态（包含角速度和加速度）
 * @param in IMU输入数据（此模型中未直接使用）
 * @return Eigen::Matrix<double, 30, 1> 状态导数向量
 */
Eigen::Matrix<double, 30, 1> get_f_output(state_output &s, const input_ikfom &in)
{
	Eigen::Matrix<double, 30, 1> res = Eigen::Matrix<double, 30, 1>::Zero();
	// 将状态中的加速度从IMU坐标系转换到世界坐标系
	vect3 a_inertial = s.rot * s.acc; // .normalized()
	for(int i = 0; i < 3; i++ ){
		res(i) = s.vel[i];                        // 位置的导数 = 速度
		res(i + 3) = s.omg[i];                    // 旋转的导数 = 状态中的角速度
		res(i + 12) = a_inertial[i] + s.gravity[i]; // 速度的导数 = 加速度 + 重力
	}
	return res;
}

/**
 * @brief 计算输入状态模型的雅可比矩阵 df/dx（状态转移方程对状态的偏导数）
 *
 * 该函数计算状态转移方程f相对于状态x的雅可比矩阵，用于ESEKF的线性化。
 * 这是EKF预测步骤中协方差更新所需的关键矩阵。
 *
 * 主要非零块：
 * - df_pos/dvel: 位置对速度的导数（单位矩阵）
 * - df_vel/drot: 速度对旋转的导数（考虑加速度的旋转变换）
 * - df_vel/dba: 速度对加速度偏置的导数
 * - df_vel/dgravity: 速度对重力的导数
 * - df_rot/dbg: 旋转对陀螺仪偏置的导数
 *
 * @param s 当前状态
 * @param in IMU输入数据
 * @return Eigen::Matrix<double, 24, 24> 雅可比矩阵
 */
Eigen::Matrix<double, 24, 24> df_dx_input(state_input &s, const input_ikfom &in)
{
	Eigen::Matrix<double, 24, 24> cov = Eigen::Matrix<double, 24, 24>::Zero();
	// df_pos/dvel: 位置对速度的导数 = I（单位矩阵）
	cov.template block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
	vect3 acc_;
	// 计算去偏置后的加速度
	in.acc.boxminus(acc_, s.ba);
	vect3 omega;
	in.gyro.boxminus(omega, s.bg);
	// df_vel/drot: 速度对旋转的导数，使用反对称矩阵（叉乘）
	cov.template block<3, 3>(12, 3) = -s.rot*MTK::hat(acc_); // .normalized().toRotationMatrix()
	// df_vel/dba: 速度对加速度偏置的导数 = -R（负的旋转矩阵）
	cov.template block<3, 3>(12, 18) = -s.rot; //.normalized().toRotationMatrix();
	// Eigen::Matrix<state_ikfom::scalar, 2, 1> vec = Eigen::Matrix<state_ikfom::scalar, 2, 1>::Zero();
	// Eigen::Matrix<state_ikfom::scalar, 3, 2> grav_matrix;
	// s.S2_Mx(grav_matrix, vec, 21);
	// df_vel/dgravity: 速度对重力的导数 = I（单位矩阵）
	cov.template block<3, 3>(12, 21) = Eigen::Matrix3d::Identity(); // grav_matrix;
	// df_rot/dbg: 旋转对陀螺仪偏置的导数 = -I（负单位矩阵）
	cov.template block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity();
	return cov;
}

/**
 * @brief 计算输出状态模型的雅可比矩阵 df/dx（状态转移方程对状态的偏导数）
 *
 * 该函数计算输出状态模型的雅可比矩阵。
 * 与输入状态不同，由于角速度和加速度是状态的一部分，雅可比矩阵的结构有所不同。
 *
 * 主要非零块：
 * - df_pos/dvel: 位置对速度的导数（单位矩阵）
 * - df_vel/drot: 速度对旋转的导数
 * - df_vel/dacc: 速度对加速度状态的导数
 * - df_vel/dgravity: 速度对重力的导数
 * - df_rot/domg: 旋转对角速度状态的导数
 *
 * @param s 当前状态
 * @param in IMU输入数据（此模型中未直接使用）
 * @return Eigen::Matrix<double, 30, 30> 雅可比矩阵
 */
Eigen::Matrix<double, 30, 30> df_dx_output(state_output &s, const input_ikfom &in)
{
	Eigen::Matrix<double, 30, 30> cov = Eigen::Matrix<double, 30, 30>::Zero();
	// df_pos/dvel: 位置对速度的导数 = I（单位矩阵）
	cov.template block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
	// df_vel/drot: 速度对旋转的导数，使用反对称矩阵
	cov.template block<3, 3>(12, 3) = -s.rot*MTK::hat(s.acc); // .normalized().toRotationMatrix()
	// df_vel/dacc: 速度对加速度状态的导数 = R（旋转矩阵）
	cov.template block<3, 3>(12, 18) = s.rot; //.normalized().toRotationMatrix();
	// Eigen::Matrix<state_ikfom::scalar, 2, 1> vec = Eigen::Matrix<state_ikfom::scalar, 2, 1>::Zero();
	// Eigen::Matrix<state_ikfom::scalar, 3, 2> grav_matrix;
	// s.S2_Mx(grav_matrix, vec, 21);
	// df_vel/dgravity: 速度对重力的导数 = I（单位矩阵）
	cov.template block<3, 3>(12, 21) = Eigen::Matrix3d::Identity(); // grav_matrix;
	// df_rot/domg: 旋转对角速度状态的导数 = I（单位矩阵）
	cov.template block<3, 3>(3, 15) = Eigen::Matrix3d::Identity();
	return cov;
}

/**
 * @brief 输入状态模型的观测方程（LiDAR点到平面距离残差）
 *
 * 该函数是ESEKF更新阶段的核心，计算LiDAR点云的观测残差及其雅可比矩阵。
 * 采用点到平面的距离作为观测模型，通过最小化点到局部地图平面的距离来更新状态估计。
 *
 * 算法流程：
 * 1. 遍历当前时间段内的所有点云
 * 2. 将点从LiDAR坐标系转换到世界坐标系
 * 3. 在局部地图中搜索最近邻点
 * 4. 拟合平面并计算点到平面距离
 * 5. 根据距离阈值筛选有效点
 * 6. 计算观测雅可比矩阵 dh/dx 和残差 z
 *
 * @param s 当前状态估计
 * @param cov_p 位置协方差（用于自适应权重，当前未使用）
 * @param cov_R 旋转协方差（用于自适应权重，当前未使用）
 * @param ekfom_data EKF观测数据结构，包含雅可比矩阵h_x和残差z
 */
void h_model_input(state_input &s, Eigen::Matrix3d cov_p, Eigen::Matrix3d cov_R, esekfom::dyn_share_modified<double> &ekfom_data)
{
	bool match_in_map = false;
	VF(4) pabcd;  // 平面参数 [a, b, c, d]，平面方程为 ax + by + cz + d = 0
	pabcd.setZero();
	normvec->resize(time_seq[k]);  // 调整法向量点云大小
	int effect_num_k = 0;  // 有效特征点计数器

	// ====== 第一阶段：点云匹配和平面拟合 ======
	for (int j = 0; j < time_seq[k]; j++)
	{
		PointType &point_body_j  = feats_down_body->points[idx+j+1];   // LiDAR坐标系下的点
		PointType &point_world_j = feats_down_world->points[idx+j+1];  // 世界坐标系下的点
		// 将点从LiDAR坐标系转换到世界坐标系
		pointBodyToWorld(&point_body_j, &point_world_j);
		V3D p_body = pbody_list[idx+j+1];
		double p_norm = p_body.norm();  // 点到LiDAR原点的距离
		V3D p_world;
		p_world << point_world_j.x, point_world_j.y, point_world_j.z;
		{
			auto &points_near = Nearest_Points[idx+j+1];
			// 在iVox地图中搜索最近邻点
            ivox_->GetClosestPoint(point_world_j, points_near, NUM_MATCH_POINTS); //
			// 检查最近邻点数量是否足够
			if ((points_near.size() < NUM_MATCH_POINTS)) // || pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5) // 5)
			{
				point_selected_surf[idx+j+1] = false;  // 点数不足，标记为无效
			}
			else
			{
				point_selected_surf[idx+j+1] = false;
				// 使用最近邻点拟合平面
				if (esti_plane(pabcd, points_near, plane_thr)) //(planeValid)
				{
					// 计算点到平面的距离：d = |ax + by + cz + d|
					float pd2 = fabs(pabcd(0) * point_world_j.x + pabcd(1) * point_world_j.y + pabcd(2) * point_world_j.z + pabcd(3));
					// V3D norm_vec;
					// M3D Rpf, pf;
					// pf = crossmat_list[idx+j+1];
					// // pf << SKEW_SYM_MATRX(p_body);
					// Rpf = s.rot * pf;
					// norm_vec << pabcd(0), pabcd(1), pabcd(2);
					// double noise_state = norm_vec.transpose() * (cov_p+Rpf*cov_R*Rpf.transpose())  * norm_vec + sqrt(p_norm) * 0.001;
					// // if (p_norm > match_s * pd2 * pd2)
					// double epsilon = pd2 / sqrt(noise_state);
					// // cout << "check epsilon:" << epsilon << endl;
					// double weight = 1.0; // epsilon / sqrt(epsilon * epsilon+1);
					// if (epsilon > 1.0)
					// {
					// 	weight = sqrt(2 * epsilon - 1) / epsilon;
					// 	pabcd(0) = weight * pabcd(0);
					// 	pabcd(1) = weight * pabcd(1);
					// 	pabcd(2) = weight * pabcd(2);
					// 	pabcd(3) = weight * pabcd(3);
					// }
					// 点到平面距离筛选：只有当点的范数足够大且距离较小时才认为是有效匹配
					// 这个条件避免了退化情况（点太近或匹配质量差）
					if (p_norm > match_s * pd2 * pd2)
					{
						point_selected_surf[idx+j+1] = true;  // 标记为有效点
						// 保存平面参数（法向量和距离）
						normvec->points[j].x = pabcd(0);  // 平面法向量 x 分量
						normvec->points[j].y = pabcd(1);  // 平面法向量 y 分量
						normvec->points[j].z = pabcd(2);  // 平面法向量 z 分量
						normvec->points[j].intensity = pabcd(3);  // 平面到原点的距离
						effect_num_k ++;  // 增加有效点计数
					}
				}
			}
		}
	}
	// 如果没有有效的匹配点，标记此次观测无效
	if (effect_num_k == 0)
	{
		ekfom_data.valid = false;
		return;
	}
	// ====== 第二阶段：构建观测雅可比矩阵和残差向量 ======
	ekfom_data.M_Noise = laser_point_cov;  // 设置激光点云测量噪声协方差
	ekfom_data.h_x.resize(effect_num_k, 12);  // 雅可比矩阵：effect_num_k x 12
	ekfom_data.h_x = Eigen::MatrixXd::Zero(effect_num_k, 12);
	ekfom_data.z.resize(effect_num_k);  // 观测残差向量
	int m = 0;  // 有效点索引

	for (int j = 0; j < time_seq[k]; j++)
	{
		// ekfom_data.converge = false;
		if(point_selected_surf[idx+j+1])
		{
			V3D norm_vec(normvec->points[j].x, normvec->points[j].y, normvec->points[j].z);

			// 根据是否估计外参，计算不同的雅可比矩阵
			if (extrinsic_est_en)
			{
				// 外参估计模式：需要计算对外参（旋转和平移）的雅可比
				V3D p_body = pbody_list[idx+j+1];
				M3D p_crossmat, p_imu_crossmat;
				p_crossmat << SKEW_SYM_MATRX(p_body);  // LiDAR点的反对称矩阵
				V3D point_imu = s.offset_R_L_I * p_body + s.offset_T_L_I;  // 转换到IMU坐标系
				p_imu_crossmat << SKEW_SYM_MATRX(point_imu);  // IMU坐标系点的反对称矩阵
				V3D C(s.rot.transpose() * norm_vec);  // 将法向量转换到IMU坐标系
				V3D A(p_imu_crossmat * C);  // 对旋转的雅可比
				V3D B(p_crossmat * s.offset_R_L_I.transpose() * C);  // 对外参的雅可比
				// 雅可比矩阵：[dh/dpos(3), dh/drot(3), dh/dext_R(3), dh/dext_T(3)]
				ekfom_data.h_x.block<1, 12>(m, 0) << norm_vec(0), norm_vec(1), norm_vec(2), VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
			}
			else
			{
				// 固定外参模式：外参雅可比为0
				M3D point_crossmat = crossmat_list[idx+j+1];
				V3D C(s.rot.transpose() * norm_vec); // conjugate().normalized()
				V3D A(point_crossmat * C);  // 对旋转的雅可比
				// 雅可比矩阵：[dh/dpos(3), dh/drot(3), 0(6)]
				ekfom_data.h_x.block<1, 12>(m, 0) << norm_vec(0), norm_vec(1), norm_vec(2), VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
			}
			// 计算观测残差：点到平面的有符号距离
			// residual = -(n^T * p + d)，其中n是法向量，p是点坐标，d是平面常数
			ekfom_data.z(m) = -norm_vec(0) * feats_down_world->points[idx+j+1].x -norm_vec(1) * feats_down_world->points[idx+j+1].y -norm_vec(2) * feats_down_world->points[idx+j+1].z-normvec->points[j].intensity;

			m++;
		}
	}
	effct_feat_num += effect_num_k;  // 累加总的有效特征点数
}

/**
 * @brief 输出状态模型的观测方程（LiDAR点到平面距离残差）
 *
 * 该函数与h_model_input类似，但用于输出状态模型。
 * 功能和算法流程与h_model_input完全相同，唯一区别在于使用的状态结构不同。
 *
 * @param s 当前状态估计（输出状态）
 * @param cov_p 位置协方差
 * @param cov_R 旋转协方差
 * @param ekfom_data EKF观测数据结构
 */
void h_model_output(state_output &s, Eigen::Matrix3d cov_p, Eigen::Matrix3d cov_R, esekfom::dyn_share_modified<double> &ekfom_data)
{
	bool match_in_map = false;
	VF(4) pabcd;
	pabcd.setZero();
	normvec->resize(time_seq[k]);
	int effect_num_k = 0;
	for (int j = 0; j < time_seq[k]; j++)
	{
		PointType &point_body_j  = feats_down_body->points[idx+j+1];
		PointType &point_world_j = feats_down_world->points[idx+j+1];
		pointBodyToWorld(&point_body_j, &point_world_j); 
		V3D p_body = pbody_list[idx+j+1];
		double p_norm = p_body.norm();
		V3D p_world;
		p_world << point_world_j.x, point_world_j.y, point_world_j.z;
		{
			auto &points_near = Nearest_Points[idx+j+1];
			
            ivox_->GetClosestPoint(point_world_j, points_near, NUM_MATCH_POINTS); // 
			
			if ((points_near.size() < NUM_MATCH_POINTS)) // || pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5)
			{
				point_selected_surf[idx+j+1] = false;
			}
			else
			{
				point_selected_surf[idx+j+1] = false;
				if (esti_plane(pabcd, points_near, plane_thr)) //(planeValid)
				{
					float pd2 = fabs(pabcd(0) * point_world_j.x + pabcd(1) * point_world_j.y + pabcd(2) * point_world_j.z + pabcd(3));
					// V3D norm_vec;
					// M3D Rpf, pf;
					// pf = crossmat_list[idx+j+1];
					// // pf << SKEW_SYM_MATRX(p_body);
					// Rpf = s.rot * pf;
					// norm_vec << pabcd(0), pabcd(1), pabcd(2);
					// double noise_state = norm_vec.transpose() * (cov_p+Rpf*cov_R*Rpf.transpose())  * norm_vec + sqrt(p_norm) * 0.001;
					// // if (p_norm > match_s * pd2 * pd2)
					// double epsilon = pd2 / sqrt(noise_state);
					// double weight = 1.0; // epsilon / sqrt(epsilon * epsilon+1);
					// if (epsilon > 1.0) 
					// {
					// 	weight = sqrt(2 * epsilon - 1) / epsilon;
					// 	pabcd(0) = weight * pabcd(0);
					// 	pabcd(1) = weight * pabcd(1);
					// 	pabcd(2) = weight * pabcd(2);
					// 	pabcd(3) = weight * pabcd(3);
					// }
					if (p_norm > match_s * pd2 * pd2)
					{
						// point_selected_surf[i] = true;
						point_selected_surf[idx+j+1] = true;
						normvec->points[j].x = pabcd(0);
						normvec->points[j].y = pabcd(1);
						normvec->points[j].z = pabcd(2);
						normvec->points[j].intensity = pabcd(3);
						effect_num_k ++;
					}
				}  
			}
		}
	}
	if (effect_num_k == 0) 
	{
		ekfom_data.valid = false;
		return;
	}
	ekfom_data.M_Noise = laser_point_cov;
	ekfom_data.h_x.resize(effect_num_k, 12);
	ekfom_data.h_x = Eigen::MatrixXd::Zero(effect_num_k, 12);
	ekfom_data.z.resize(effect_num_k);
	int m = 0;
	for (int j = 0; j < time_seq[k]; j++)
	{
		// ekfom_data.converge = false;
		if(point_selected_surf[idx+j+1])
		{
			V3D norm_vec(normvec->points[j].x, normvec->points[j].y, normvec->points[j].z);
			if (extrinsic_est_en)
			{
				V3D p_body = pbody_list[idx+j+1];
				M3D p_crossmat, p_imu_crossmat;
				p_crossmat << SKEW_SYM_MATRX(p_body);
				V3D point_imu = s.offset_R_L_I * p_body + s.offset_T_L_I;
				p_imu_crossmat << SKEW_SYM_MATRX(point_imu);
				V3D C(s.rot.transpose() * norm_vec);
				V3D A(p_imu_crossmat * C);
				V3D B(p_crossmat * s.offset_R_L_I.transpose() * C);
				ekfom_data.h_x.block<1, 12>(m, 0) << norm_vec(0), norm_vec(1), norm_vec(2), VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
			}
			else
			{   
				M3D point_crossmat = crossmat_list[idx+j+1];
				V3D C(s.rot.transpose() * norm_vec); // conjugate().normalized()
				V3D A(point_crossmat * C);
				ekfom_data.h_x.block<1, 12>(m, 0) << norm_vec(0), norm_vec(1), norm_vec(2), VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
			}
			ekfom_data.z(m) = -norm_vec(0) * feats_down_world->points[idx+j+1].x -norm_vec(1) * feats_down_world->points[idx+j+1].y -norm_vec(2) * feats_down_world->points[idx+j+1].z-normvec->points[j].intensity;
			
			m++;
		}
	}
	effct_feat_num += effect_num_k;
}

/**
 * @brief IMU观测模型（用于输出状态模型的IMU约束更新）
 *
 * 该函数将IMU测量值作为观测量，用于约束和校正状态中的角速度和加速度。
 * 这是Point-LIO特有的设计，通过将IMU测量作为观测而非输入，提供额外的约束。
 *
 * 观测模型：
 * - 角速度观测残差 = 测量值 - 估计的角速度 - 陀螺仪偏置
 * - 加速度观测残差 = 归一化的测量值 - 估计的加速度 - 加速度计偏置
 *
 * 同时进行IMU饱和检测，对饱和的轴将残差置零以避免错误更新。
 *
 * @param s 当前状态估计（包含角速度和加速度状态）
 * @param ekfom_data EKF观测数据结构
 */
void h_model_IMU_output(state_output &s, esekfom::dyn_share_modified<double> &ekfom_data)
{
    // 初始化饱和检测标志为false
    std::memset(ekfom_data.satu_check, false, 6);
	// 角速度观测残差：测量均值 - 状态中的角速度 - 陀螺仪偏置
	ekfom_data.z_IMU.block<3,1>(0, 0) = angvel_avr - s.omg - s.bg;
	// 加速度观测残差：归一化的测量均值 - 状态中的加速度 - 加速度计偏置
	ekfom_data.z_IMU.block<3,1>(3, 0) = acc_avr * G_m_s2 / acc_norm - s.acc - s.ba;
	// 设置IMU测量噪声协方差
    ekfom_data.R_IMU << imu_meas_omg_cov, imu_meas_omg_cov, imu_meas_omg_cov, imu_meas_acc_cov, imu_meas_acc_cov, imu_meas_acc_cov;

	// IMU饱和检测和处理
	if(check_satu)
	{
		// 检测陀螺仪X轴是否饱和
		if(fabs(angvel_avr(0)) >= 0.99 * satu_gyro)
		{
			ekfom_data.satu_check[0] = true;  // 标记为饱和
			ekfom_data.z_IMU(0) = 0.0;        // 饱和时残差置零，不用于更新
		}

		// 检测陀螺仪Y轴是否饱和
		if(fabs(angvel_avr(1)) >= 0.99 * satu_gyro)
		{
			ekfom_data.satu_check[1] = true;
			ekfom_data.z_IMU(1) = 0.0;
		}

		// 检测陀螺仪Z轴是否饱和
		if(fabs(angvel_avr(2)) >= 0.99 * satu_gyro)
		{
			ekfom_data.satu_check[2] = true;
			ekfom_data.z_IMU(2) = 0.0;
		}

		// 检测加速度计X轴是否饱和
		if(fabs(acc_avr(0)) >= 0.99 * satu_acc)
		{
			ekfom_data.satu_check[3] = true;
			ekfom_data.z_IMU(3) = 0.0;
		}

		// 检测加速度计Y轴是否饱和
		if(fabs(acc_avr(1)) >= 0.99 * satu_acc)
		{
			ekfom_data.satu_check[4] = true;
			ekfom_data.z_IMU(4) = 0.0;
		}

		// 检测加速度计Z轴是否饱和
		if(fabs(acc_avr(2)) >= 0.99 * satu_acc)
		{
			ekfom_data.satu_check[5] = true;
			ekfom_data.z_IMU(5) = 0.0;
		}
	}
}

/**
 * @brief 将点从LiDAR坐标系转换到世界坐标系
 *
 * 该函数执行坐标系变换链：LiDAR -> IMU -> World
 * 变换公式：p_world = R_world_imu * (R_lidar_imu * p_lidar + T_lidar_imu) + T_world_imu
 *
 * 根据不同的配置选择不同的滤波器和外参：
 * 1. 是否进行外参在线估计（extrinsic_est_en）
 * 2. 使用输入状态还是输出状态模型（use_imu_as_input）
 *
 * @param pi 输入点（LiDAR坐标系）
 * @param po 输出点（世界坐标系）
 */
void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    // 提取LiDAR坐标系下的点坐标
    V3D p_body(pi->x, pi->y, pi->z);

    V3D p_global;
	if (extrinsic_est_en)
	{
		// 使用在线估计的外参
		if (!use_imu_as_input)
		{
			// 使用输出状态滤波器的外参估计
			// p_world = R_wi * (R_li * p_l + T_li) + T_wi
			p_global = kf_output.x_.rot * (kf_output.x_.offset_R_L_I * p_body + kf_output.x_.offset_T_L_I) + kf_output.x_.pos;
		}
		else
		{
			// 使用输入状态滤波器的外参估计
			p_global = kf_input.x_.rot * (kf_input.x_.offset_R_L_I * p_body + kf_input.x_.offset_T_L_I) + kf_input.x_.pos;
		}
	}
	else
	{
		// 使用固定的外参（从配置文件读取）
		if (!use_imu_as_input)
		{
			// 使用输出状态滤波器的位姿 + 固定外参
			p_global = kf_output.x_.rot * (Lidar_R_wrt_IMU * p_body + Lidar_T_wrt_IMU) + kf_output.x_.pos; // .normalized()
		}
		else
		{
			// 使用输入状态滤波器的位姿 + 固定外参
			p_global = kf_input.x_.rot * (Lidar_R_wrt_IMU * p_body + Lidar_T_wrt_IMU) + kf_input.x_.pos; // .normalized()
		}
	}

    // 将结果赋值给输出点
    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;  // 保留强度信息
}