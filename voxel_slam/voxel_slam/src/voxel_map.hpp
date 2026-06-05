#ifndef VOXEL_MAP2_HPP
#define VOXEL_MAP2_HPP

#include "tools.hpp"
#include "preintegration.hpp"
#include <thread>
#include <Eigen/Eigenvalues>
#include <unordered_set>
#include <mutex>

#include <ros/ros.h>
#include <fstream>

/**
 * @brief 点变量结构体
 * 存储点的位置和协方差矩阵
 */
struct pointVar
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d pnt;  // 点的3D坐标
  Eigen::Matrix3d var;  // 点的协方差矩阵（3x3）
};

using PVec = vector<pointVar>;  // 点变量向量类型别名
using PVecPtr = shared_ptr<vector<pointVar>>;  // 点变量向量智能指针类型别名

/**
 * @brief 对点变量向量进行体素降采样
 * @param pvec 输入的点变量向量
 * @param voxel_size 体素大小
 * @param pl_keep 输出的降采样后的点云
 */
void down_sampling_pvec(PVec &pvec, double voxel_size, pcl::PointCloud<PointType> &pl_keep)
{
  // 使用哈希映射存储体素及其对应的点和计数
  unordered_map<VOXEL_LOC, pair<pointVar, int>> feat_map;
  float loc_xyz[3];

  // 遍历所有点，将其分配到对应的体素中
  for(pointVar &pv: pvec)
  {
    // 计算点所在的体素索引
    for(int j=0; j<3; j++)
    {
      loc_xyz[j] = pv.pnt[j] / voxel_size;
      if(loc_xyz[j] < 0)
        loc_xyz[j] -= 1.0;  // 处理负坐标的情况
    }

    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    auto iter = feat_map.find(position);
    if(iter == feat_map.end())
    {
      // 新体素，直接添加
      feat_map[position] = make_pair(pv, 1);
    }
    else
    {
      // 已存在的体素，计算平均值
      pair<pointVar, int> &pp = iter->second;
      pp.first.pnt = (pp.first.pnt * pp.second + pv.pnt) / (pp.second + 1);
      pp.first.var = (pp.first.var * pp.second + pv.var) / (pp.second + 1);
      pp.second += 1;  // 增加点计数
    }
  }

  // 从体素映射构建降采样后的点云
  pcl::PointCloud<PointType>().swap(pl_keep);
  pl_keep.reserve(feat_map.size());
  PointType ap;
  for(auto iter=feat_map.begin(); iter!=feat_map.end(); ++iter)
  {
    pointVar &pv = iter->second.first;
    ap.x = pv.pnt[0]; ap.y = pv.pnt[1]; ap.z = pv.pnt[2];
    // 将协方差矩阵的对角元素存储到法向量字段中
    ap.normal_x = pv.var(0, 0);
    ap.normal_y = pv.var(1, 1);
    ap.normal_z = pv.var(2, 2);
    pl_keep.push_back(ap);
  }

}

/**
 * @brief 平面结构体
 * 存储平面的几何参数和协方差信息
 */
struct Plane
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d center = Eigen::Vector3d::Zero();  // 平面中心点
  Eigen::Vector3d normal = Eigen::Vector3d::Zero();  // 平面法向量
  Eigen::Matrix<double, 6, 6> plane_var;             // 平面参数的协方差矩阵（6x6）
  float radius = 0;                                  // 平面的有效半径
  bool is_plane = false;                             // 是否为有效平面的标志

  Plane()
  {
    plane_var.setZero();
  }

};

// 全局参数定义
Eigen::Vector4d min_point;                 // 最小点数阈值
double min_eigen_value;                    // 最小特征值阈值
int max_layer = 2;                         // 八叉树最大层数
int max_points = 100;                      // 每个体素最大点数
double voxel_size = 1.0;                   // 体素大小
int min_ba_point = 20;                     // BA优化的最小点数
vector<double> plane_eigen_value_thre;     // 平面特征值阈值向量

/**
 * @brief 计算点的Bf变换和协方差
 * @param pv 输入点变量
 * @param bcov 输出的9x9协方差矩阵
 * @param vec 3D向量
 */
void Bf_var(const pointVar &pv, Eigen::Matrix<double, 9, 9> &bcov, const Eigen::Vector3d &vec)
{
  Eigen::Matrix<double, 6, 3> Bi;
  // 构建Bi矩阵，用于计算二次型协方差传播
  Bi << 2*vec(0),        0,        0,
          vec(1),   vec(0),        0,
          vec(2),        0,   vec(0),
               0, 2*vec(1),        0,
               0,   vec(2),   vec(1),
               0,        0, 2*vec(2);
  // 计算协方差传播
  Eigen::Matrix<double, 6, 3> Biup = Bi * pv.var;
  bcov.block<6, 6>(0, 0) = Biup * Bi.transpose();
  bcov.block<6, 3>(0, 6) = Biup;
  bcov.block<3, 6>(6, 0) = Biup.transpose();
  bcov.block<3, 3>(6, 6) = pv.var;
}

/**
 * @brief LiDAR BA因子类
 * 用于优化过程中的LiDAR束调整（Bundle Adjustment）
 */
class LidarFactor
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  vector<PointCluster> sig_vecs;                  // 固定点簇向量
  vector<vector<PointCluster>> plvec_voxels;      // 体素中的点簇向量（窗口内的多帧）
  vector<double> coeffs;                          // 系数向量
  PLV(3) eig_values;                              // 特征值向量
  PLM(3) eig_vectors;                             // 特征向量矩阵
  vector<PointCluster> pcr_adds;                  // 附加点簇
  int win_size;                                   // 滑动窗口大小

  /**
   * @brief 构造函数
   * @param _w 窗口大小
   */
  LidarFactor(int _w): win_size(_w){}

  /**
   * @brief 向因子中添加体素数据
   * @param vec_orig 原始点簇向量
   * @param fix 固定点簇
   * @param coe 系数
   * @param eig_value 特征值
   * @param eig_vector 特征向量
   * @param pcr_add 附加点簇
   */
  void push_voxel(vector<PointCluster> &vec_orig, PointCluster &fix, double coe, Eigen::Vector3d &eig_value, Eigen::Matrix3d &eig_vector, PointCluster &pcr_add)
  {
    plvec_voxels.push_back(vec_orig);
    sig_vecs.push_back(fix);
    coeffs.push_back(coe);
    eig_values.push_back(eig_value);
    eig_vectors.push_back(eig_vector);
    pcr_adds.push_back(pcr_add);
  }

  /**
   * @brief 计算Hessian矩阵、雅可比向量和残差
   * @param xs IMU状态向量
   * @param head 起始索引
   * @param end 结束索引
   * @param Hess 输出的Hessian矩阵
   * @param JacT 输出的雅可比转置向量
   * @param residual 输出的残差值
   */
  void acc_evaluate2(const vector<IMUST> &xs, int head, int end, Eigen::MatrixXd &Hess, Eigen::VectorXd &JacT, double &residual)
  {
    // 初始化输出矩阵和向量
    Hess.setZero(); JacT.setZero(); residual = 0;
    vector<PointCluster> sig_tran(win_size);
    const int kk = 0;  // 使用第0个特征值（最小特征值）

    // 预先分配临时变量
    PLV(3) viRiTuk(win_size);
    PLM(3) viRiTukukT(win_size);

    vector<Eigen::Matrix<double, 3, 6>, Eigen::aligned_allocator<Eigen::Matrix<double, 3, 6>>> Auk(win_size);
    Eigen::Matrix3d umumT;

    // 遍历所有体素
    for(int a=head; a<end; a++)
    {
      vector<PointCluster> &sig_orig = plvec_voxels[a];
      double coe = coeffs[a];  // 当前体素的系数

      // 获取预计算的特征值和特征向量
      Eigen::Vector3d lmbd = eig_values[a];
      Eigen::Matrix3d U = eig_vectors[a];
      int NN = pcr_adds[a].N;
      Eigen::Vector3d vBar = pcr_adds[a].v / NN;  // 点簇的平均位置
      
      Eigen::Vector3d u[3] = {U.col(0), U.col(1), U.col(2)};
      Eigen::Vector3d &uk = u[kk];
      Eigen::Matrix3d ukukT = uk * uk.transpose();
      umumT.setZero();
      for(int i=0; i<3; i++)
        if(i != kk)
          umumT += 2.0/(lmbd[kk] - lmbd[i]) * u[i] * u[i].transpose();

      for(int i=0; i<win_size; i++)
      // for(int i=1; i<win_size; i++)
      if(sig_orig[i].N != 0)
      {
        Eigen::Matrix3d Pi = sig_orig[i].P;
        Eigen::Vector3d vi = sig_orig[i].v;
        Eigen::Matrix3d Ri = xs[i].R;
        double ni = sig_orig[i].N;

        Eigen::Matrix3d vihat; vihat << SKEW_SYM_MATRX(vi);
        Eigen::Vector3d RiTuk = Ri.transpose() * uk;
        Eigen::Matrix3d RiTukhat; RiTukhat << SKEW_SYM_MATRX(RiTuk);

        Eigen::Vector3d PiRiTuk = Pi * RiTuk;
        viRiTuk[i] = vihat * RiTuk;
        viRiTukukT[i] = viRiTuk[i] * uk.transpose();
        
        Eigen::Vector3d ti_v = xs[i].p - vBar;
        double ukTti_v = uk.dot(ti_v);

        Eigen::Matrix3d combo1 = hat(PiRiTuk) + vihat * ukTti_v;
        Eigen::Vector3d combo2 = Ri*vi + ni*ti_v;
        Auk[i].block<3, 3>(0, 0) = (Ri*Pi + ti_v*vi.transpose()) * RiTukhat - Ri*combo1;
        Auk[i].block<3, 3>(0, 3) = combo2 * uk.transpose() + combo2.dot(uk) * I33;
        Auk[i] /= NN;

        const Eigen::Matrix<double, 6, 1> &jjt = Auk[i].transpose() * uk;
        JacT.block<6, 1>(6*i, 0) += coe * jjt;

        const Eigen::Matrix3d &HRt = 2.0/NN * (1.0-ni/NN) * viRiTukukT[i];
        Eigen::Matrix<double, 6, 6> Hb = Auk[i].transpose() * umumT * Auk[i];
        Hb.block<3, 3>(0, 0) += 2.0/NN * (combo1 - RiTukhat*Pi) * RiTukhat - 2.0/NN/NN * viRiTuk[i] * viRiTuk[i].transpose() - 0.5*hat(jjt.block<3, 1>(0, 0));
        Hb.block<3, 3>(0, 3) += HRt;
        Hb.block<3, 3>(3, 0) += HRt.transpose();
        Hb.block<3, 3>(3, 3) += 2.0/NN * (ni - ni*ni/NN) * ukukT;

        Hess.block<6, 6>(6*i, 6*i) += coe * Hb;
      }

      for(int i=0; i<win_size-1; i++)
      // for(int i=1; i<win_size-1; i++)
      if(sig_orig[i].N != 0)
      {
        double ni = sig_orig[i].N;
        for(int j=i+1; j<win_size; j++)
        if(sig_orig[j].N != 0)
        {
          double nj = sig_orig[j].N;
          Eigen::Matrix<double, 6, 6> Hb = Auk[i].transpose() * umumT * Auk[j];
          Hb.block<3, 3>(0, 0) += -2.0/NN/NN * viRiTuk[i] * viRiTuk[j].transpose();
          Hb.block<3, 3>(0, 3) += -2.0*nj/NN/NN * viRiTukukT[i];
          Hb.block<3, 3>(3, 0) += -2.0*ni/NN/NN * viRiTukukT[j].transpose();
          Hb.block<3, 3>(3, 3) += -2.0*ni*nj/NN/NN * ukukT;

          Hess.block<6, 6>(6*i, 6*j) += coe * Hb;
        }
      }
      
      residual += coe * lmbd[kk];
    }

    for(int i=1; i<win_size; i++)
      for(int j=0; j<i; j++)
        Hess.block<6, 6>(6*i, 6*j) = Hess.block<6, 6>(6*j, 6*i).transpose();
    
  }

  /**
   * @brief 仅计算残差（不计算Hessian和雅可比）
   * @param xs IMU状态向量
   * @param head 起始索引
   * @param end 结束索引
   * @param residual 输出的残差值
   */
  void evaluate_only_residual(const vector<IMUST> &xs, int head, int end, double &residual)
  {
    residual = 0;
    int kk = 0;  // 使用第0个特征值（最小特征值）

    PointCluster pcr;

    for(int a=head; a<end; a++)
    {
      const vector<PointCluster> &sig_orig = plvec_voxels[a];
      PointCluster sig = sig_vecs[a];

      for(int i=0; i<win_size; i++)
      if(sig_orig[i].N != 0)
      {
        pcr.transform(sig_orig[i], xs[i]);
        sig += pcr;
      }

      Eigen::Vector3d vBar = sig.v / sig.N;
      // Eigen::Matrix3d cmt = sig.P/sig.N - vBar * vBar.transpose();
      // Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(sig.P - sig.v * vBar.transpose());
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(sig.P/sig.N - vBar * vBar.transpose());
      Eigen::Vector3d lmbd = saes.eigenvalues();

      // centers[a] = vBar;
      eig_values[a] = saes.eigenvalues();
      eig_vectors[a] = saes.eigenvectors();
      pcr_adds[a] = sig;
      // Ns[a] = sig.N;

      residual += coeffs[a] * lmbd[kk];
    }
    
  }

  /**
   * @brief 清空所有数据
   */
  void clear()
  {
    sig_vecs.clear(); plvec_voxels.clear();
    eig_values.clear(); eig_vectors.clear();
    pcr_adds.clear(); coeffs.clear();
  }

  ~LidarFactor(){}

};

/**
 * @brief LiDAR BA优化器类
 * 使用Levenberg-Marquardt算法进行LiDAR束调整优化
 */
class Lidar_BA_Optimizer
{
public:
  int win_size, jac_leng, thd_num = 2;  // 窗口大小、雅可比长度、线程数

  /**
   * @brief 多线程计算Hessian和雅可比
   * @param x_stats IMU状态向量
   * @param voxhess LiDAR因子
   * @param Hess 输出的Hessian矩阵
   * @param JacT 输出的雅可比转置向量
   * @return 总残差值
   */
  double divide_thread(vector<IMUST> &x_stats, LidarFactor &voxhess, Eigen::MatrixXd &Hess, Eigen::VectorXd &JacT)
  {
    // int thd_num = 4;
    double residual = 0;
    Hess.setZero(); JacT.setZero();
    PLM(-1) hessians(thd_num); 
    PLV(-1) jacobins(thd_num);

    for(int i=0; i<thd_num; i++)
    {
      hessians[i].resize(jac_leng, jac_leng);
      jacobins[i].resize(jac_leng);
    }

    int tthd_num = thd_num;
    vector<double> resis(tthd_num, 0);
    int g_size = voxhess.plvec_voxels.size();
    if(g_size < tthd_num) tthd_num = 1;

    vector<thread*> mthreads(tthd_num);
    double part = 1.0 * g_size / tthd_num;
    // for(int i=0; i<tthd_num; i++)
    for(int i=1; i<tthd_num; i++)
      mthreads[i] = new thread(&LidarFactor::acc_evaluate2, &voxhess, x_stats, part*i, part*(i+1), ref(hessians[i]), ref(jacobins[i]), ref(resis[i]));

    for(int i=0; i<tthd_num; i++)
    {
      if(i != 0) mthreads[i]->join();
      else
        voxhess.acc_evaluate2(x_stats, 0, part, hessians[0], jacobins[0], resis[0]);
      Hess += hessians[i];
      JacT += jacobins[i];
      residual += resis[i];
      delete mthreads[i];
    }

    return residual;
  }

  /**
   * @brief 仅计算残差（多线程）
   * @param x_stats IMU状态向量
   * @param voxhess LiDAR因子
   * @return 总残差值
   */
  double only_residual(vector<IMUST> &x_stats, LidarFactor &voxhess)
  {
    double residual1 = 0;
    vector<double> residuals(thd_num, 0);
    int g_size = voxhess.plvec_voxels.size();
    if(g_size < thd_num)
    {
      printf("Too Less Voxel"); exit(0);
    }
    vector<thread*> mthreads(thd_num, nullptr);
    double part = 1.0 * g_size / thd_num;
    for(int i=1; i<thd_num; i++)
      mthreads[i] = new thread(&LidarFactor::evaluate_only_residual, &voxhess, x_stats, part*i, part*(i+1), ref(residuals[i]));

    for(int i=0; i<thd_num; i++)
    {
      if(i != 0) 
        mthreads[i]->join();
      else
        voxhess.evaluate_only_residual(x_stats, part*i, part*(i+1), residuals[i]);
      residual1 += residuals[i];
      delete mthreads[i];
    }

    return residual1;
  }

  /**
   * @brief 阻尼迭代优化（Levenberg-Marquardt）
   * @param x_stats IMU状态向量（输入输出）
   * @param voxhess LiDAR因子
   * @param hess 输出的Hessian矩阵指针
   * @param resis 残差向量（输出）
   * @param max_iter 最大迭代次数
   * @param is_display 是否显示优化过程
   * @return 是否收敛
   */
  bool damping_iter(vector<IMUST> &x_stats, LidarFactor &voxhess, Eigen::MatrixXd* hess, vector<double> &resis, int max_iter = 3, bool is_display = false)
  {
    win_size = voxhess.win_size;
    jac_leng = win_size * 6;

    // LM算法参数
    double u = 0.01, v = 2;
    Eigen::MatrixXd D(jac_leng, jac_leng), Hess(jac_leng, jac_leng);
    Eigen::VectorXd JacT(jac_leng), dxi(jac_leng);
    hess->resize(jac_leng, jac_leng);

    D.setIdentity();
    double residual1, residual2, q;
    bool is_calc_hess = true;
    vector<IMUST> x_stats_temp = x_stats;

    bool is_converge = true;

    // double tt1 = ros::Time::now().toSec();
    // for(int i=0; i<10; i++)
    for(int i=0; i<max_iter; i++)
    {
      if(is_calc_hess)
      {
        residual1 = divide_thread(x_stats, voxhess, Hess, JacT);
        *hess = Hess;
      }

      if(i == 0)
        resis.push_back(residual1);

      Hess.topRows(6).setZero();
      Hess.leftCols(6).setZero();
      Hess.block<6, 6>(0, 0).setIdentity();
      JacT.head(6).setZero();
      
      D.diagonal() = Hess.diagonal();
      dxi = (Hess + u*D).ldlt().solve(-JacT);

      for(int j=0; j<win_size; j++)
      {
        x_stats_temp[j].R = x_stats[j].R * Exp(dxi.block<3, 1>(6*j, 0));
        x_stats_temp[j].p = x_stats[j].p + dxi.block<3, 1>(6*j+3, 0);
      }
      double q1 = 0.5*dxi.dot(u*D*dxi-JacT);

      residual2 = only_residual(x_stats_temp, voxhess);

      q = (residual1-residual2);
      if(is_display)
        printf("iter%d: (%lf %lf) u: %lf v: %.1lf q: %.2lf %lf %lf\n", i, residual1, residual2, u, v, q/q1, q1, q);

      if(q > 0)
      {
        x_stats = x_stats_temp;
        double one_three = 1.0 / 3;

        q = q / q1;
        v = 2;
        q = 1 - pow(2*q-1, 3);
        u *= (q<one_three ? one_three:q);
        is_calc_hess = true;
      }
      else
      {
        u = u * v;
        v = 2 * v;
        is_calc_hess = false;
        is_converge = false;
      }

      if(fabs((residual1-residual2)/residual1)<1e-6)
        break;
    }
    resis.push_back(residual2);
    return is_converge;
  }

};

double imu_coef = 1e-4;  // IMU因子权重系数
// double imu_coef = 1e-8;
#define DVEL 6  // 速度维度

/**
 * @brief LiDAR-惯性BA优化器类
 * 结合LiDAR和IMU进行联合束调整优化
 */
class LI_BA_Optimizer
{
public:
  int win_size, jac_leng, imu_leng;

  void hess_plus(Eigen::MatrixXd &Hess, Eigen::VectorXd &JacT, Eigen::MatrixXd &hs, Eigen::VectorXd &js)
  {
    for(int i=0; i<win_size; i++)
    {
      JacT.block<DVEL, 1>(i*DIM, 0) += js.block<DVEL, 1>(i*DVEL, 0);
      for(int j=0; j<win_size; j++)
        Hess.block<DVEL, DVEL>(i*DIM, j*DIM) += hs.block<DVEL, DVEL>(i*DVEL, j*DVEL);
    }
  }

  double divide_thread(vector<IMUST> &x_stats, LidarFactor &voxhess, deque<IMU_PRE*> &imus_factor, Eigen::MatrixXd &Hess, Eigen::VectorXd &JacT)
  {
    int thd_num = 5;
    double residual = 0;
    Hess.setZero(); JacT.setZero();
    PLM(-1) hessians(thd_num); 
    PLV(-1) jacobins(thd_num);
    vector<double> resis(thd_num, 0);

    for(int i=0; i<thd_num; i++)
    {
      hessians[i].resize(jac_leng, jac_leng);
      jacobins[i].resize(jac_leng);
    }

    int tthd_num = thd_num;
    int g_size = voxhess.plvec_voxels.size();
    if(g_size < tthd_num) tthd_num = 1;
    double part = 1.0 * g_size / tthd_num;

    vector<thread*> mthreads(tthd_num);
    // for(int i=0; i<tthd_num; i++)
    for(int i=1; i<tthd_num; i++)
      mthreads[i] = new thread(&LidarFactor::acc_evaluate2, &voxhess, x_stats, part*i, part * (i+1), ref(hessians[i]), ref(jacobins[i]), ref(resis[i]));

    Eigen::MatrixXd jtj(2*DIM, 2*DIM);
    Eigen::VectorXd gg(2*DIM);

    for(int i=0; i<win_size-1; i++)
    {
      jtj.setZero(); gg.setZero();
      residual += imus_factor[i]->give_evaluate(x_stats[i], x_stats[i+1], jtj, gg, true);
      Hess.block<DIM*2, DIM*2>(i*DIM, i*DIM) += jtj;
      JacT.block<DIM*2, 1>(i*DIM, 0) += gg;
    }

    Eigen::Matrix<double, DIM, DIM> joc;
    Eigen::Matrix<double, DIM, 1> rr;
    joc.setIdentity(); rr.setZero();

    Hess *= imu_coef;
    JacT *= imu_coef;
    residual *= (imu_coef * 0.5);

    // printf("resi: %lf\n", residual);

    for(int i=0; i<tthd_num; i++)
    {
      // mthreads[i]->join();
      if(i != 0) mthreads[i]->join();
      else
        voxhess.acc_evaluate2(x_stats, 0, part, hessians[0], jacobins[0], resis[0]);
      hess_plus(Hess, JacT, hessians[i], jacobins[i]);
      residual += resis[i];
      delete mthreads[i];
    }

    return residual;
  }

  double only_residual(vector<IMUST> &x_stats, LidarFactor &voxhess, deque<IMU_PRE*> &imus_factor)
  {
    double residual1 = 0, residual2 = 0;
    Eigen::MatrixXd jtj(2*DIM, 2*DIM);
    Eigen::VectorXd gg(2*DIM);

    int thd_num = 5;
    vector<double> residuals(thd_num, 0);
    int g_size = voxhess.plvec_voxels.size();
    if(g_size < thd_num)
    {
      // printf("Too Less Voxel"); exit(0);
      thd_num = 1;
    }
    vector<thread*> mthreads(thd_num, nullptr);
    double part = 1.0 * g_size / thd_num;
    for(int i=1; i<thd_num; i++)
      mthreads[i] = new thread(&LidarFactor::evaluate_only_residual, &voxhess, x_stats, part*i, part*(i+1), ref(residuals[i]));

    for(int i=0; i<win_size-1; i++)
      residual1 += imus_factor[i]->give_evaluate(x_stats[i], x_stats[i+1], jtj, gg, false);
    residual1 *= (imu_coef * 0.5);

    for(int i=0; i<thd_num; i++)
    {
      if(i != 0) 
      {
        mthreads[i]->join(); delete mthreads[i];
      }
      else
        voxhess.evaluate_only_residual(x_stats, part*i, part*(i+1), residuals[i]);
      residual2 += residuals[i];
    }

    return (residual1 + residual2);
  }

  void damping_iter(vector<IMUST> &x_stats, LidarFactor &voxhess, deque<IMU_PRE*> &imus_factor, Eigen::MatrixXd* hess)
  {
    win_size = voxhess.win_size;
    jac_leng = win_size * 6;
    imu_leng = win_size * DIM;
    double u = 0.01, v = 2;
    Eigen::MatrixXd D(imu_leng, imu_leng), Hess(imu_leng, imu_leng);
    Eigen::VectorXd JacT(imu_leng), dxi(imu_leng);
    hess->resize(imu_leng, imu_leng);

    D.setIdentity();
    double residual1, residual2, q;
    bool is_calc_hess = true;
    vector<IMUST> x_stats_temp = x_stats;

    double hesstime = 0;
    double resitime = 0;
  
    // for(int i=0; i<10; i++)
    for(int i=0; i<3; i++)
    {
      if(is_calc_hess)
      {
        double tm = ros::Time::now().toSec();
        residual1 = divide_thread(x_stats, voxhess, imus_factor, Hess, JacT);
        hesstime += ros::Time::now().toSec() - tm;
        *hess = Hess;
      }
      
      Hess.topRows(DIM).setZero();
      Hess.leftCols(DIM).setZero();
      Hess.block<DIM, DIM>(0, 0).setIdentity();
      JacT.head(DIM).setZero();

      D.diagonal() = Hess.diagonal();
      dxi = (Hess + u*D).ldlt().solve(-JacT);

      for(int j=0; j<win_size; j++)
      {
        x_stats_temp[j].R = x_stats[j].R * Exp(dxi.block<3, 1>(DIM*j, 0));
        x_stats_temp[j].p = x_stats[j].p + dxi.block<3, 1>(DIM*j+3, 0);
        x_stats_temp[j].v = x_stats[j].v + dxi.block<3, 1>(DIM*j+6, 0);
        x_stats_temp[j].bg = x_stats[j].bg + dxi.block<3, 1>(DIM*j+9, 0);
        x_stats_temp[j].ba = x_stats[j].ba + dxi.block<3, 1>(DIM*j+12, 0);
      }

      for(int j=0; j<win_size-1; j++)
        imus_factor[j]->update_state(dxi.block<DIM, 1>(DIM*j, 0));

      double q1 = 0.5 * dxi.dot(u*D*dxi-JacT);

      double tl1 = ros::Time::now().toSec();
      residual2 = only_residual(x_stats_temp, voxhess, imus_factor);
      double tl2 = ros::Time::now().toSec();
      // printf("onlyresi: %lf\n", tl2-tl1);
      resitime += tl2 - tl1;

      q = (residual1-residual2);
      // printf("iter%d: (%lf %lf) u: %lf v: %.1lf q: %.2lf %lf %lf\n", i, residual1, residual2, u, v, q/q1, q1, q);

      if(q > 0)
      {
        x_stats = x_stats_temp;
        double one_three = 1.0 / 3;

        q = q / q1;
        v = 2;
        q = 1 - pow(2*q-1, 3);
        u *= (q<one_three ? one_three:q);
        is_calc_hess = true;
      }
      else
      {
        u = u * v;
        v = 2 * v;
        is_calc_hess = false;

        for(int j=0; j<win_size-1; j++)
        {
          imus_factor[j]->dbg = imus_factor[j]->dbg_buf;
          imus_factor[j]->dba = imus_factor[j]->dba_buf;
        }

      }

      if(fabs((residual1-residual2)/residual1)<1e-6)
        break;
    }

    // printf("ba: %lf %lf %zu\n", hesstime, resitime, voxhess.plvec_voxels.size());

  }

};

/**
 * @brief 带重力优化的LiDAR-惯性BA优化器类
 * 在LiDAR-IMU联合优化的基础上增加重力方向的优化
 */
class LI_BA_OptimizerGravity
{
public:
  int win_size, jac_leng, imu_leng;  // 窗口大小、雅可比长度、IMU状态长度

  void hess_plus(Eigen::MatrixXd &Hess, Eigen::VectorXd &JacT, Eigen::MatrixXd &hs, Eigen::VectorXd &js)
  {
    for(int i=0; i<win_size; i++)
    {
      JacT.block<DVEL, 1>(i*DIM, 0) += js.block<DVEL, 1>(i*DVEL, 0);
      for(int j=0; j<win_size; j++)
        Hess.block<DVEL, DVEL>(i*DIM, j*DIM) += hs.block<DVEL, DVEL>(i*DVEL, j*DVEL);
    }
  }

  double divide_thread(vector<IMUST> &x_stats, LidarFactor &voxhess, deque<IMU_PRE*> &imus_factor, Eigen::MatrixXd &Hess, Eigen::VectorXd &JacT)
  {
    int thd_num = 5;
    double residual = 0;
    Hess.setZero(); JacT.setZero();
    PLM(-1) hessians(thd_num); 
    PLV(-1) jacobins(thd_num);
    vector<double> resis(thd_num, 0);

    for(int i=0; i<thd_num; i++)
    {
      hessians[i].resize(jac_leng, jac_leng);
      jacobins[i].resize(jac_leng);
    }

    int tthd_num = thd_num;
    int g_size = voxhess.plvec_voxels.size();
    if(g_size < tthd_num) tthd_num = 1;
    double part = 1.0 * g_size / tthd_num;

    vector<thread*> mthreads(tthd_num);
    // for(int i=0; i<tthd_num; i++)
    for(int i=1; i<tthd_num; i++)
      mthreads[i] = new thread(&LidarFactor::acc_evaluate2, &voxhess, x_stats, part*i, part * (i+1), ref(hessians[i]), ref(jacobins[i]), ref(resis[i]));

    Eigen::MatrixXd jtj(2*DIM+3, 2*DIM+3);
    Eigen::VectorXd gg(2*DIM+3);

    for(int i=0; i<win_size-1; i++)
    {
      jtj.setZero(); gg.setZero();
      residual += imus_factor[i]->give_evaluate_g(x_stats[i], x_stats[i+1], jtj, gg, true);
      Hess.block<DIM*2, DIM*2>(i*DIM, i*DIM) += jtj.block<2*DIM, 2*DIM>(0, 0);
      Hess.block<DIM*2, 3>(i*DIM, imu_leng-3) += jtj.block<2*DIM, 3>(0, 2*DIM);
      Hess.block<3, DIM*2>(imu_leng-3, i*DIM) += jtj.block<3, 2*DIM>(2*DIM,0);
      Hess.block<3, 3>(imu_leng-3, imu_leng-3) += jtj.block<3, 3>(2*DIM, 2*DIM);

      JacT.block<DIM*2, 1>(i*DIM, 0) += gg.head(2*DIM);
      JacT.tail(3) += gg.tail(3);
    }

    Eigen::Matrix<double, DIM, DIM> joc;
    Eigen::Matrix<double, DIM, 1> rr;
    joc.setIdentity(); rr.setZero();

    Hess *= imu_coef;
    JacT *= imu_coef;
    residual *= (imu_coef * 0.5);

    // printf("resi: %lf\n", residual);

    for(int i=0; i<tthd_num; i++)
    {
      // mthreads[i]->join();
      if(i != 0) mthreads[i]->join();
      else
        voxhess.acc_evaluate2(x_stats, 0, part, hessians[0], jacobins[0], resis[0]);
      hess_plus(Hess, JacT, hessians[i], jacobins[i]);
      residual += resis[i];
      delete mthreads[i];
    }

    return residual;
  }

  double only_residual(vector<IMUST> &x_stats, LidarFactor &voxhess, deque<IMU_PRE*> &imus_factor)
  {
    double residual1 = 0, residual2 = 0;
    Eigen::MatrixXd jtj(2*DIM, 2*DIM);
    Eigen::VectorXd gg(2*DIM);

    int thd_num = 5;
    vector<double> residuals(thd_num, 0);
    int g_size = voxhess.plvec_voxels.size();
    if(g_size < thd_num)
    {
      // printf("Too Less Voxel"); exit(0);
      thd_num = 1;
    }
    vector<thread*> mthreads(thd_num, nullptr);
    double part = 1.0 * g_size / thd_num;
    for(int i=1; i<thd_num; i++)
      mthreads[i] = new thread(&LidarFactor::evaluate_only_residual, &voxhess, x_stats, part*i, part*(i+1), ref(residuals[i]));

    for(int i=0; i<win_size-1; i++)
      residual1 += imus_factor[i]->give_evaluate_g(x_stats[i], x_stats[i+1], jtj, gg, false);
    residual1 *= (imu_coef * 0.5);

    for(int i=0; i<thd_num; i++)
    {
      if(i != 0) 
      {
        mthreads[i]->join(); delete mthreads[i];
      }
      else
        voxhess.evaluate_only_residual(x_stats, part*i, part*(i+1), residuals[i]);
      residual2 += residuals[i];
    }

    return (residual1 + residual2);
  }

  void damping_iter(vector<IMUST> &x_stats, LidarFactor &voxhess, deque<IMU_PRE*> &imus_factor, vector<double> &resis, Eigen::MatrixXd* hess, int max_iter = 2)
  {
    win_size = voxhess.win_size;
    jac_leng = win_size * 6;
    imu_leng = win_size * DIM + 3;
    double u = 0.01, v = 2;
    Eigen::MatrixXd D(imu_leng, imu_leng), Hess(imu_leng, imu_leng);
    Eigen::VectorXd JacT(imu_leng), dxi(imu_leng);

    D.setIdentity();
    double residual1, residual2, q;
    bool is_calc_hess = true;
    vector<IMUST> x_stats_temp = x_stats;
    
    for(int i=0; i<max_iter; i++)
    {
      if(is_calc_hess)
      {
        residual1 = divide_thread(x_stats, voxhess, imus_factor, Hess, JacT);
        *hess = Hess;
      }

      if(i == 0)
        resis.push_back(residual1);

      Hess.topRows(6).setZero();
      Hess.leftCols(6).setZero();
      Hess.block<6, 6>(0, 0).setIdentity();
      JacT.head(6).setZero();

      // Hess.rightCols(3).setZero();
      // Hess.bottomRows(3).setZero();
      // Hess.block<3, 3>(imu_leng-3, imu_leng-3).setIdentity();
      // JacT.tail(3).setZero();

      D.diagonal() = Hess.diagonal();
      dxi = (Hess + u*D).ldlt().solve(-JacT);

      x_stats_temp[0].g += dxi.tail(3);
      for(int j=0; j<win_size; j++)
      {
        x_stats_temp[j].R = x_stats[j].R * Exp(dxi.block<3, 1>(DIM*j, 0));
        x_stats_temp[j].p = x_stats[j].p + dxi.block<3, 1>(DIM*j+3, 0);
        x_stats_temp[j].v = x_stats[j].v + dxi.block<3, 1>(DIM*j+6, 0);
        x_stats_temp[j].bg = x_stats[j].bg + dxi.block<3, 1>(DIM*j+9, 0);
        x_stats_temp[j].ba = x_stats[j].ba + dxi.block<3, 1>(DIM*j+12, 0);
        x_stats_temp[j].g = x_stats_temp[0].g;
      }

      for(int j=0; j<win_size-1; j++)
        imus_factor[j]->update_state(dxi.block<DIM, 1>(DIM*j, 0));
      
      double q1 = 0.5 * dxi.dot(u*D*dxi-JacT);
      residual2 = only_residual(x_stats_temp, voxhess, imus_factor);
      q = (residual1-residual2);
      // printf("iter%d: (%lf %lf) u: %lf v: %.1lf q: %.2lf %lf %lf\n", i, residual1, residual2, u, v, q/q1, q1, q);
      
      if(q > 0)
      {
        x_stats = x_stats_temp;
        double one_three = 1.0 / 3;

        q = q / q1;
        v = 2;
        q = 1 - pow(2*q-1, 3);
        u *= (q<one_three ? one_three:q);
        is_calc_hess = true;
      }
      else
      {
        u = u * v;
        v = 2 * v;
        is_calc_hess = false;

        for(int j=0; j<win_size-1; j++)
        {
          imus_factor[j]->dbg = imus_factor[j]->dbg_buf;
          imus_factor[j]->dba = imus_factor[j]->dba_buf;
        }
      }

      if(fabs((residual1-residual2)/residual1)<1e-6)
        break;

    }
    resis.push_back(residual2);
    
  }

};

/**
 * @brief 关键帧结构体
 * 10帧扫描合并为一个关键帧
 */
struct Keyframe
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  IMUST x0;                                // IMU状态
  pcl::PointCloud<PointType>::Ptr plptr;   // 点云数据指针
  int exist;                               // 是否存在的标志
  int id, mp;                              // 关键帧ID和映射索引
  float jour;                              // 时间戳或序号

  Keyframe(IMUST &_x0): x0(_x0), exist(0)
  {
    plptr.reset(new pcl::PointCloud<PointType>());
  }

  /**
   * @brief 生成变换后的点云
   * @param pl_send 输出的点云
   * @param rot 旋转矩阵（默认为单位矩阵）
   * @param tra 平移向量（默认为零向量）
   */
  void generate(pcl::PointCloud<PointType> &pl_send, Eigen::Matrix3d rot = Eigen::Matrix3d::Identity(), Eigen::Vector3d tra = Eigen::Vector3d(0, 0, 0))
  {
    Eigen::Vector3d v3;
    for(PointType ap: plptr->points)
    {
      v3 << ap.x, ap.y, ap.z;
      v3 = rot * v3 + tra;  // 应用旋转和平移
      ap.x = v3[0]; ap.y = v3[1]; ap.z = v3[2];
      pl_send.push_back(ap);
    }
  }

};

/**
 * @brief 滑动窗口类
 * 每个体素节点中的滑动窗口，用于存储多帧点云数据
 */
class SlideWindow
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  vector<PVec> points;                // 点变量向量（多帧）
  vector<PointCluster> pcrs_local;    // 局部点簇向量（多帧）

  /**
   * @brief 构造函数
   * @param wdsize 窗口大小
   */
  SlideWindow(int wdsize)
  {
    pcrs_local.resize(wdsize);
    points.resize(wdsize);
    for(int i=0; i<wdsize; i++)
      points[i].reserve(20);  // 预分配空间
  }

  /**
   * @brief 调整窗口大小
   * @param wdsize 新的窗口大小
   */
  void resize(int wdsize)
  {
    if(points.size() != wdsize)
    {
      points.resize(wdsize);
      pcrs_local.resize(wdsize);
    }
  }

  /**
   * @brief 清空所有数据
   */
  void clear()
  {
    int wdsize = points.size();
    for(int i=0; i<wdsize; i++)
    {
      points[i].clear();
      pcrs_local[i].clear();
    }
  }

};

/**
 * @brief 八叉树地图类
 * 用于里程计和局部建图，可根据项目需求重写
 */
int* mp;  // 映射指针
class OctoTree
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SlideWindow* sw = nullptr;              // 滑动窗口指针
  PointCluster pcr_add;                   // 累加的点簇
  Eigen::Matrix<double, 9, 9> cov_add;    // 累加的协方差矩阵

  PointCluster pcr_fix;                   // 固定的点簇
  PVec point_fix;                         // 固定的点变量向量

  int layer, octo_state, wdsize;          // 层级、八叉树状态、窗口大小
  OctoTree* leaves[8];                    // 8个子节点指针
  double voxel_center[3];                 // 体素中心坐标
  double jour = 0;                        // 时间戳
  float quater_length;                    // 四分之一体素边长

  Plane plane;                            // 平面参数
  bool isexist = false;                   // 是否存在的标志

  Eigen::Vector3d eig_value;              // 特征值
  Eigen::Matrix3d eig_vector;             // 特征向量

  int last_num = 0, opt_state = -1;       // 上次点数、优化状态
  mutex mVox;                             // 互斥锁

  /**
   * @brief 构造函数
   * @param _l 层级
   * @param _w 窗口大小
   */
  OctoTree(int _l, int _w) : layer(_l), wdsize(_w), octo_state(0)
  {
    for(int i=0; i<8; i++) leaves[i] = nullptr;
    cov_add.setZero();
  }

  /**
   * @brief 向体素中添加点
   * @param ord 顺序索引
   * @param pv 点变量
   * @param pw 世界坐标系下的点位置
   * @param sws 滑动窗口向量
   */
  inline void push(int ord, const pointVar &pv, const Eigen::Vector3d &pw, vector<SlideWindow*> &sws)
  {
    mVox.lock();  // 加锁保护
    // 如果滑动窗口不存在，则创建或从缓存中获取
    if(sw == nullptr)
    {
      if(sws.size() != 0)
      {
        sw = sws.back();
        sws.pop_back();
        sw->resize(wdsize);
      }
      else
        sw = new SlideWindow(wdsize);
    }
    if(!isexist) isexist = true;

    int mord = mp[ord];
    if(layer < max_layer)
      sw->points[mord].push_back(pv);  // 添加点到滑动窗口
    sw->pcrs_local[mord].push(pv.pnt);  // 添加点到局部点簇
    pcr_add.push(pw);  // 添加世界坐标点
    // 计算并累加协方差
    Eigen::Matrix<double, 9, 9> Bi;
    Bf_var(pv, Bi, pw);
    cov_add += Bi;
    mVox.unlock();  // 解锁
  }

  inline void push_fix(pointVar &pv)
  {
    if(layer < max_layer)
      point_fix.push_back(pv);
    pcr_fix.push(pv.pnt);
    pcr_add.push(pv.pnt);
    Eigen::Matrix<double, 9, 9> Bi;
    Bf_var(pv, Bi, pv.pnt);
    cov_add += Bi;
  }

  inline void push_fix_novar(pointVar &pv)
  {
    if(layer < max_layer)
      point_fix.push_back(pv);
    pcr_fix.push(pv.pnt);
    pcr_add.push(pv.pnt);
  }

  /**
   * @brief 判断是否为平面
   * @param eig_values 特征值向量
   * @return 是否为平面
   */
  inline bool plane_judge(Eigen::Vector3d &eig_values)
  {
    // 通过特征值比值判断平面性
    return (eig_values[0] < min_eigen_value && (eig_values[0]/eig_values[2])<plane_eigen_value_thre[layer]);
  }

  /**
   * @brief 分配点到体素或子节点
   * @param ord 顺序索引
   * @param pv 点变量
   * @param pw 世界坐标系下的点位置
   * @param sws 滑动窗口向量
   */
  void allocate(int ord, const pointVar &pv, const Eigen::Vector3d &pw, vector<SlideWindow*> &sws)
  {
    if(octo_state == 0)
    {
      // 叶节点，直接添加点
      push(ord, pv, pw, sws);
    }
    else
    {
      // 分支节点，确定点所在的子节点
      int xyz[3] = {0, 0, 0};
      for(int k=0; k<3; k++)
        if(pw[k] > voxel_center[k]) xyz[k] = 1;
      int leafnum = 4*xyz[0] + 2*xyz[1] + xyz[2];  // 计算子节点索引

      // 如果子节点不存在，则创建
      if(leaves[leafnum] == nullptr)
      {
        leaves[leafnum] = new OctoTree(layer+1, wdsize);
        // 计算子节点的中心位置
        leaves[leafnum]->voxel_center[0] = voxel_center[0] + (2*xyz[0]-1)*quater_length;
        leaves[leafnum]->voxel_center[1] = voxel_center[1] + (2*xyz[1]-1)*quater_length;
        leaves[leafnum]->voxel_center[2] = voxel_center[2] + (2*xyz[2]-1)*quater_length;
        leaves[leafnum]->quater_length = quater_length / 2;
      }

      // 递归分配到子节点
      leaves[leafnum]->allocate(ord, pv, pw, sws);
    }

  }

  void allocate_fix(pointVar &pv)
  {
    if(octo_state == 0)
    {
      push_fix_novar(pv);
    }
    else if(layer < max_layer)
    {
      int xyz[3] = {0, 0, 0};
      for(int k=0; k<3; k++)
        if(pv.pnt[k] > voxel_center[k]) xyz[k] = 1;
      int leafnum = 4*xyz[0] + 2*xyz[1] + xyz[2];

      if(leaves[leafnum] == nullptr)
      {
        leaves[leafnum] = new OctoTree(layer+1, wdsize);
        leaves[leafnum]->voxel_center[0] = voxel_center[0] + (2*xyz[0]-1)*quater_length;
        leaves[leafnum]->voxel_center[1] = voxel_center[1] + (2*xyz[1]-1)*quater_length;
        leaves[leafnum]->voxel_center[2] = voxel_center[2] + (2*xyz[2]-1)*quater_length;
        leaves[leafnum]->quater_length = quater_length / 2;
      }

      leaves[leafnum]->allocate_fix(pv);
    }
  }

  void fix_divide(vector<SlideWindow*> &sws)
  {
    for(pointVar &pv: point_fix)
    {
      int xyz[3] = {0, 0, 0};
      for(int k=0; k<3; k++)
        if(pv.pnt[k] > voxel_center[k]) xyz[k] = 1;
      int leafnum = 4*xyz[0] + 2*xyz[1] + xyz[2];
      if(leaves[leafnum] == nullptr)
      {
        leaves[leafnum] = new OctoTree(layer+1, wdsize);
        leaves[leafnum]->voxel_center[0] = voxel_center[0] + (2*xyz[0]-1)*quater_length;
        leaves[leafnum]->voxel_center[1] = voxel_center[1] + (2*xyz[1]-1)*quater_length;
        leaves[leafnum]->voxel_center[2] = voxel_center[2] + (2*xyz[2]-1)*quater_length;
        leaves[leafnum]->quater_length = quater_length / 2;
      }

      leaves[leafnum]->push_fix(pv);
    }

  }

  void subdivide(int si, IMUST &xx, vector<SlideWindow*> &sws)
  {
    for(pointVar &pv: sw->points[mp[si]])
    {
      Eigen::Vector3d pw = xx.R * pv.pnt + xx.p;
      int xyz[3] = {0, 0, 0};
      for(int k=0; k<3; k++)
        if(pw[k] > voxel_center[k]) xyz[k] = 1;
      int leafnum = 4*xyz[0] + 2*xyz[1] + xyz[2];
      if(leaves[leafnum] == nullptr)
      {
        leaves[leafnum] = new OctoTree(layer+1, wdsize);
        leaves[leafnum]->voxel_center[0] = voxel_center[0] + (2*xyz[0]-1)*quater_length;
        leaves[leafnum]->voxel_center[1] = voxel_center[1] + (2*xyz[1]-1)*quater_length;
        leaves[leafnum]->voxel_center[2] = voxel_center[2] + (2*xyz[2]-1)*quater_length;
        leaves[leafnum]->quater_length = quater_length / 2;
      }

      leaves[leafnum]->push(si, pv, pw, sws);
    }
  }

  void plane_update()
  {
    plane.center = pcr_add.v / pcr_add.N;
    int l = 0;
    Eigen::Vector3d u[3] = {eig_vector.col(0), eig_vector.col(1), eig_vector.col(2)};
    double nv = 1.0 / pcr_add.N;

    Eigen::Matrix<double, 3, 9> u_c; u_c.setZero();
    for(int k=0; k<3; k++)
    if(k != l)
    {
      Eigen::Matrix3d ukl = u[k] * u[l].transpose();
      Eigen::Matrix<double, 1, 9> fkl;
      fkl.head(6) << ukl(0, 0), ukl(1, 0)+ukl(0, 1), ukl(2, 0)+ukl(0, 2), 
                     ukl(1, 1), ukl(1, 2)+ukl(2, 1),           ukl(2, 2);
      fkl.tail(3) = -(u[k].dot(plane.center) * u[l] + u[l].dot(plane.center) * u[k]);
      
      u_c += nv / (eig_value[l]-eig_value[k]) * u[k] * fkl;
    }

    Eigen::Matrix<double, 3, 9> Jc = u_c * cov_add;
    plane.plane_var.block<3, 3>(0, 0) = Jc * u_c.transpose();
    Eigen::Matrix3d Jc_N = nv * Jc.block<3, 3>(0, 6);
    plane.plane_var.block<3, 3>(0, 3) = Jc_N;
    plane.plane_var.block<3, 3>(3, 0) = Jc_N.transpose();
    plane.plane_var.block<3, 3>(3, 3) = nv * nv * cov_add.block<3, 3>(6, 6);
    plane.normal = u[0];
    plane.radius = eig_value[2];
  }

  /**
   * @brief 重新切分体素
   * @param win_count 窗口计数
   * @param x_buf IMU状态缓冲区
   * @param sws 滑动窗口向量
   */
  void recut(int win_count, vector<IMUST> &x_buf, vector<SlideWindow*> &sws)
  {
    if(octo_state == 0)
    {
      if(layer >= 0)
      {
        opt_state = -1;
        // 点数不足，标记为非平面
        if(pcr_add.N <= min_point[layer])
        {
          plane.is_plane = false; return;
        }
        if(!isexist || sw == nullptr) return;

        // 计算特征值和特征向量
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(pcr_add.cov());
        eig_value  = saes.eigenvalues();
        eig_vector = saes.eigenvectors();
        plane.is_plane = plane_judge(eig_value);

        if(plane.is_plane)
        {
          // 是平面，不需要进一步分割
          return;
        }
        else if(layer >= max_layer)
          // 已达到最大层级，不再分割
          return;
      }
      
      if(pcr_fix.N != 0)
      {
        fix_divide(sws);
        // point_fix.clear();
        PVec().swap(point_fix);
      }

      for(int i=0; i<win_count; i++)
        subdivide(i, x_buf[i], sws);

      sw->clear();
      sws.push_back(sw);
      sw = nullptr;
      octo_state = 1;
    }

    for(int i=0; i<8; i++)
      if(leaves[i] != nullptr)
        leaves[i]->recut(win_count, x_buf, sws);

  }

  void margi(int win_count, int mgsize, vector<IMUST> &x_buf, const LidarFactor &vox_opt)
  {
    if(octo_state == 0 && layer>=0)
    {
      if(!isexist || sw == nullptr) return;
      mVox.lock();
      vector<PointCluster> pcrs_world(wdsize);
      // pcr_add = pcr_fix;
      // for(int i=0; i<win_count; i++)
      // if(sw->pcrs_local[mp[i]].N != 0)
      // {
      //   pcrs_world[i].transform(sw->pcrs_local[mp[i]], x_buf[i]);
      //   pcr_add += pcrs_world[i];
      // }

      if(opt_state >= int(vox_opt.pcr_adds.size()))
      {
        printf("Error: opt_state: %d %zu\n", opt_state, vox_opt.pcr_adds.size());
        exit(0);
      }

      if(opt_state >= 0)
      {
        pcr_add = vox_opt.pcr_adds[opt_state];
        eig_value  = vox_opt.eig_values[opt_state];
        eig_vector = vox_opt.eig_vectors[opt_state];
        opt_state = -1;
        
        for(int i=0; i<mgsize; i++)
        if(sw->pcrs_local[mp[i]].N != 0)
        {
          pcrs_world[i].transform(sw->pcrs_local[mp[i]], x_buf[i]);
        }
      }
      else
      {
        pcr_add = pcr_fix;
        for(int i=0; i<win_count; i++)
        if(sw->pcrs_local[mp[i]].N != 0)
        {
          pcrs_world[i].transform(sw->pcrs_local[mp[i]], x_buf[i]);
          pcr_add += pcrs_world[i];
        }

        if(plane.is_plane)
        {
          Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(pcr_add.cov());
          eig_value = saes.eigenvalues();
          eig_vector = saes.eigenvectors();
        }
        
      }

      if(pcr_fix.N < max_points && plane.is_plane)
      if(pcr_add.N - last_num >= 5 || last_num <= 10)
      {
        plane_update();
        last_num = pcr_add.N;
      }

      if(pcr_fix.N < max_points)
      {
        for(int i=0; i<mgsize; i++)
        if(pcrs_world[i].N != 0)
        {
          pcr_fix += pcrs_world[i];
          for(pointVar pv: sw->points[mp[i]])
          {
            pv.pnt = x_buf[i].R * pv.pnt + x_buf[i].p;
            point_fix.push_back(pv);
          }
        }

      }
      else
      {
        for(int i=0; i<mgsize; i++)
          if(pcrs_world[i].N != 0)
            pcr_add -= pcrs_world[i];
        
        if(point_fix.size() != 0)
          PVec().swap(point_fix);
      }

      for(int i=0; i<mgsize; i++)
      if(sw->pcrs_local[mp[i]].N != 0)
      {
        sw->pcrs_local[mp[i]].clear();
        sw->points[mp[i]].clear();
      }
      
      if(pcr_fix.N >= pcr_add.N)
        isexist = false;
      else
        isexist = true;
      
      mVox.unlock();
    }
    else
    {
      isexist = false;
      for(int i=0; i<8; i++)
      if(leaves[i] != nullptr)
      {
        leaves[i]->margi(win_count, mgsize, x_buf, vox_opt);
        isexist = isexist || leaves[i]->isexist;
      }
    }

  }

  /**
   * @brief 提取LiDAR因子用于优化
   * @param vox_opt LiDAR因子对象
   */
  void tras_opt(LidarFactor &vox_opt)
  {
    if(octo_state == 0)
    {
      // 叶节点且为有效平面
      if(layer >= 0 && isexist && plane.is_plane && sw!=nullptr)
      {
        // 特征值比值检查，确保平面质量
        if(eig_value[0]/eig_value[1] > 0.12) return;

        double coe = 1;
        vector<PointCluster> pcrs(wdsize);
        for(int i=0; i<wdsize; i++)
          pcrs[i] = sw->pcrs_local[mp[i]];
        opt_state = vox_opt.plvec_voxels.size();
        vox_opt.push_voxel(pcrs, pcr_fix, coe, eig_value, eig_vector, pcr_add);
      }

    }
    else
    {
      // 分支节点，递归处理所有子节点
      for(int i=0; i<8; i++)
        if(leaves[i] != nullptr)
          leaves[i]->tras_opt(vox_opt);
    }


  }

  /**
   * @brief 匹配点与平面
   * @param wld 世界坐标系下的点
   * @param pla 输出的匹配平面指针
   * @param max_prob 最大概率（未使用）
   * @param var_wld 点的协方差矩阵
   * @param sigma_d 输出的标准差
   * @param oc 输出的八叉树节点指针
   * @return 是否匹配成功（1表示成功，0表示失败）
   */
  int match(Eigen::Vector3d &wld, Plane* &pla, double &max_prob, Eigen::Matrix3d &var_wld, double &sigma_d, OctoTree* &oc)
  {
    int flag = 0;
    if(octo_state == 0)
    {
      // 叶节点，检查是否为平面
      if(plane.is_plane)
      {
        // 计算点到平面的距离
        float dis_to_plane = fabs(plane.normal.dot(wld - plane.center));
        float dis_to_center = (plane.center - wld).squaredNorm();
        float range_dis = (dis_to_center - dis_to_plane * dis_to_plane);
        // 检查点是否在平面的有效范围内
        if(range_dis <= 3*3*plane.radius)
        {
          // 计算不确定性
          Eigen::Matrix<double, 1, 6> J_nq;
          J_nq.block<1, 3>(0, 0) = wld - plane.center;
          J_nq.block<1, 3>(0, 3) = -plane.normal;
          double sigma_l = J_nq * plane.plane_var * J_nq.transpose();
          sigma_l += plane.normal.transpose() * var_wld * plane.normal;
          // 3-sigma检验
          if(dis_to_plane < 3 * sqrt(sigma_l))
          {
            // 匹配成功
            oc = this;
            sigma_d = sigma_l;
            pla = &plane;
            flag = 1;
          }
        }
      }
    }
    else
    {
      // 分支节点，确定点所在的子节点并递归匹配
      int xyz[3] = {0, 0, 0};
      for(int k=0; k<3; k++)
        if(wld[k] > voxel_center[k]) xyz[k] = 1;
      int leafnum = 4*xyz[0] + 2*xyz[1] + xyz[2];

      if(leaves[leafnum] != nullptr)
        flag = leaves[leafnum]->match(wld, pla, max_prob, var_wld, sigma_d, oc);
    }

    return flag;
  }

  void tras_ptr(vector<OctoTree*> &octos_release)
  {
    if(octo_state == 1)
    {
      for(int i=0; i<8; i++)
      if(leaves[i] != nullptr)
      {
        octos_release.push_back(leaves[i]);
        leaves[i]->tras_ptr(octos_release);
      }
    }
  }

  // ~OctoTree()
  // {
  //   for(int i=0; i<8; i++)
  //   if(leaves[i] != nullptr)
  //   {
  //     delete leaves[i];
  //     leaves[i] = nullptr;
  //   }
  // }

  // Extract the point cloud map for debug
  void tras_display(int win_count, pcl::PointCloud<PointType> &pl_fixd, pcl::PointCloud<PointType> &pl_wind, vector<IMUST> &x_buf)
  {
    if(octo_state == 0)
    {
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(pcr_add.cov());
      Eigen::Matrix3d eig_vectors = saes.eigenvectors();
      Eigen::Vector3d eig_values  = saes.eigenvalues();

      PointType ap; 
      // ap.intensity = ins;

      if(plane.is_plane)
      {
        // if(pcr_add.N-pcr_fix.N < min_ba_point) return;
        // if(eig_value[0]/eig_value[1] > 0.1)
        //   return;

        // for(pointVar &pv: point_fix)
        // {
        //   Eigen::Vector3d pvec = pv.pnt;
        //   ap.x = pvec[0];
        //   ap.y = pvec[1];
        //   ap.z = pvec[2];
        //   ap.normal_x = sqrt(eig_values[0]);
        //   ap.normal_y = sqrt(eig_values[2] / eig_values[0]);
        //   ap.normal_z = pcr_add.N;
        //   ap.curvature = pcr_add.N - pcr_fix.N;
        //   pl_fixd.push_back(ap);
        // }

        for(int i=0; i<win_count; i++)
        for(pointVar &pv: sw->points[mp[i]])
        {
          Eigen::Vector3d pvec = x_buf[i].R * pv.pnt + x_buf[i].p;
          ap.x = pvec[0]; ap.y = pvec[1]; ap.z = pvec[2];
          // ap.normal_x = sqrt(eig_values[0]);
          // ap.normal_y = sqrt(eig_values[2] / eig_values[0]);
          // ap.normal_z = pcr_add.N;
          // ap.curvature = pcr_add.N - pcr_fix.N;
          pl_wind.push_back(ap);
        }
      }

    }
    else
    {
      for(int i=0; i<8; i++)
        if(leaves[i] != nullptr)
          leaves[i]->tras_display(win_count, pl_fixd, pl_wind, x_buf);
    }

  }

  bool inside(Eigen::Vector3d &wld)
  {
    double hl = quater_length * 2;
    return (wld[0] >= voxel_center[0] - hl &&
            wld[0] <= voxel_center[0] + hl &&
            wld[1] >= voxel_center[1] - hl &&
            wld[1] <= voxel_center[1] + hl &&
            wld[2] >= voxel_center[2] - hl &&
            wld[2] <= voxel_center[2] + hl);
  }

  void clear_slwd(vector<SlideWindow*> &sws)
  {
    if(octo_state != 0)
    {
      for(int i=0; i<8; i++)
      if(leaves[i] != nullptr)
      {
        leaves[i]->clear_slwd(sws);
      }
    }

    if(sw != nullptr)
    {
      sw->clear();
      sws.push_back(sw);
      sw = nullptr;
    }

  }

};

/**
 * @brief 将点云切分到体素地图中
 * @param feat_map 特征地图
 * @param pvec 点变量向量指针
 * @param win_count 窗口计数
 * @param feat_tem_map 临时特征地图
 * @param wdsize 窗口大小
 * @param pwld 世界坐标系下的点位置向量
 * @param sws 滑动窗口向量
 */
void cut_voxel(unordered_map<VOXEL_LOC, OctoTree*> &feat_map, PVecPtr pvec, int win_count, unordered_map<VOXEL_LOC, OctoTree*> &feat_tem_map, int wdsize, PLV(3) &pwld, vector<SlideWindow*> &sws)
{
  int plsize = pvec->size();
  for(int i=0; i<plsize; i++)
  {
    pointVar &pv = (*pvec)[i];
    Eigen::Vector3d &pw = pwld[i];
    float loc[3];
    // 计算体素索引
    for(int j=0; j<3; j++)
    {
      loc[j] = pw[j] / voxel_size;
      if(loc[j] < 0) loc[j] -= 1;  // 处理负坐标
    }

    VOXEL_LOC position(loc[0], loc[1], loc[2]);
    auto iter = feat_map.find(position);
    if(iter != feat_map.end())
    {
      // 体素已存在，分配点到该体素
      iter->second->allocate(win_count, pv, pw, sws);
      iter->second->isexist = true;
      if(feat_tem_map.find(position) == feat_map.end())
        feat_tem_map[position] = iter->second;
    }
    else
    {
      // 创建新体素
      OctoTree* ot = new OctoTree(0, wdsize);
      ot->allocate(win_count, pv, pw, sws);
      ot->voxel_center[0] = (0.5+position.x) * voxel_size;
      ot->voxel_center[1] = (0.5+position.y) * voxel_size;
      ot->voxel_center[2] = (0.5+position.z) * voxel_size;
      ot->quater_length = voxel_size / 4.0;
      feat_map[position] = ot;
      feat_tem_map[position] = ot;
    }
  }

}

/**
 * @brief 多线程将当前扫描切分到对应体素中
 * @param feat_map 特征地图
 * @param pvec 点变量向量指针
 * @param win_count 窗口计数
 * @param feat_tem_map 临时特征地图
 * @param wdsize 窗口大小
 * @param pwld 世界坐标系下的点位置向量
 * @param sws 多线程滑动窗口向量
 */
void cut_voxel_multi(unordered_map<VOXEL_LOC, OctoTree*> &feat_map, PVecPtr pvec, int win_count, unordered_map<VOXEL_LOC, OctoTree*> &feat_tem_map, int wdsize, PLV(3) &pwld, vector<vector<SlideWindow*>> &sws)
{
  unordered_map<OctoTree*, vector<int>> map_pvec;
  int plsize = pvec->size();
  for(int i=0; i<plsize; i++)
  {
    pointVar &pv = (*pvec)[i];
    Eigen::Vector3d &pw = pwld[i];
    float loc[3];
    for(int j=0; j<3; j++)
    {
      // loc[j] = pv.world[j] / voxel_size;
      loc[j] = pw[j] / voxel_size;
      if(loc[j] < 0) loc[j] -= 1;
    }

    VOXEL_LOC position(loc[0], loc[1], loc[2]);
    auto iter = feat_map.find(position);
    OctoTree* ot = nullptr;
    if(iter != feat_map.end())
    {
      iter->second->isexist = true;
      if(feat_tem_map.find(position) == feat_map.end())
        feat_tem_map[position] = iter->second;
      ot = iter->second;
    }
    else
    {
      ot = new OctoTree(0, wdsize);
      ot->voxel_center[0] = (0.5+position.x) * voxel_size;
      ot->voxel_center[1] = (0.5+position.y) * voxel_size;
      ot->voxel_center[2] = (0.5+position.z) * voxel_size;
      ot->quater_length = voxel_size / 4.0;
      feat_map[position] = ot;
      feat_tem_map[position] = ot;
    }

    map_pvec[ot].push_back(i);
  }

  // for(auto iter=map_pvec.begin(); iter!=map_pvec.end(); iter++)
  // {
  //   for(int i: iter->second)
  //   {
  //     iter->first->allocate(win_count, (*pvec)[i], pwld[i], sws);
  //   }
  // }

  vector<pair<OctoTree *const, vector<int>>*> octs; octs.reserve(map_pvec.size());
  for(auto iter=map_pvec.begin(); iter!=map_pvec.end(); iter++)
    octs.push_back(&(*iter));

  int thd_num = sws.size();
  int g_size = octs.size();
  if(g_size < thd_num) return;
  vector<thread*> mthreads(thd_num);
  double part = 1.0 * g_size / thd_num;

  int swsize = sws[0].size() / thd_num;
  for(int i=1; i<thd_num; i++)
  {
    sws[i].insert(sws[i].end(), sws[0].end() - swsize, sws[0].end());
    sws[0].erase(sws[0].end() - swsize, sws[0].end());
  }

  for(int i=1; i<thd_num; i++)
  {
    mthreads[i] = new thread
    (
      [&](int head, int tail, vector<SlideWindow*> &sw)
      {
        for(int j=head; j<tail; j++)
        {
          for(int k: octs[j]->second)
            octs[j]->first->allocate(win_count, (*pvec)[k], pwld[k], sw);
        }
      }, part*i, part*(i+1), ref(sws[i])
    );
  }

  for(int i=0; i<thd_num; i++)
  {
    if(i == 0)
    {
      for(int j=0; j<int(part); j++)
        for(int k: octs[j]->second)
          octs[j]->first->allocate(win_count, (*pvec)[k], pwld[k], sws[0]);
    }
    else
    {
      mthreads[i]->join();
      delete mthreads[i];
    }
    
  }

}

/**
 * @brief 将固定点切分到体素地图中
 * @param feat_map 特征地图
 * @param pvec 点变量向量
 * @param wdsize 窗口大小
 * @param jour 时间戳
 */
void cut_voxel(unordered_map<VOXEL_LOC, OctoTree*> &feat_map, PVec &pvec, int wdsize, double jour)
{
  for(pointVar &pv: pvec)
  {
    float loc[3];
    // 计算体素索引
    for(int j=0; j<3; j++)
    {
      loc[j] = pv.pnt[j] / voxel_size;
      if(loc[j] < 0) loc[j] -= 1;  // 处理负坐标
    }

    VOXEL_LOC position(loc[0], loc[1], loc[2]);
    auto iter = feat_map.find(position);
    if(iter != feat_map.end())
    {
      // 体素已存在，添加固定点
      iter->second->allocate_fix(pv);
    }
    else
    {
      // 创建新体素并添加固定点
      OctoTree* ot = new OctoTree(0, wdsize);
      ot->push_fix_novar(pv);
      ot->voxel_center[0] = (0.5+position.x) * voxel_size;
      ot->voxel_center[1] = (0.5+position.y) * voxel_size;
      ot->voxel_center[2] = (0.5+position.z) * voxel_size;
      ot->quater_length = voxel_size / 4.0;
      ot->jour = jour;
      feat_map[position] = ot;
    }
  }

}

/**
 * @brief 在体素地图中匹配点与平面
 * @param feat_map 特征地图
 * @param wld 世界坐标系下的点
 * @param pla 输出的匹配平面指针
 * @param var_wld 点的协方差矩阵
 * @param sigma_d 输出的标准差
 * @param oc 输出的八叉树节点指针
 * @return 是否匹配成功
 */
int match(unordered_map<VOXEL_LOC, OctoTree*> &feat_map, Eigen::Vector3d &wld, Plane* &pla, Eigen::Matrix3d &var_wld, double &sigma_d, OctoTree* &oc)
{
  int flag = 0;

  float loc[3];
  // 计算点所在的体素索引
  for(int j=0; j<3; j++)
  {
    loc[j] = wld[j] / voxel_size;
    if(loc[j] < 0) loc[j] -= 1;  // 处理负坐标
  }
  VOXEL_LOC position(loc[0], loc[1], loc[2]);
  auto iter = feat_map.find(position);
  if(iter != feat_map.end())
  {
    // 找到对应体素，进行匹配
    double max_prob = 0;
    flag = iter->second->match(wld, pla, max_prob, var_wld, sigma_d, oc);
    if(flag && pla==nullptr)
    {
      // 调试信息：匹配成功但平面指针为空
      printf("pla null max_prob: %lf %ld %ld %ld\n", max_prob, iter->first.x, iter->first.y, iter->first.z);
    }
  }

  return flag;
}

#endif
