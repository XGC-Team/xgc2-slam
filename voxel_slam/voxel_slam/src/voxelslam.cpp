#include "voxelslam.hpp"

using namespace std;

// ResultOutput类：负责SLAM系统的结果输出和可视化
// 包括里程计发布、局部轨迹、局部地图、全局路径和全局地图的发布
class ResultOutput
{
public:
  // 单例模式：获取ResultOutput的唯一实例
  static ResultOutput &instance()
  {
    static ResultOutput inst;
    return inst;
  }

  // 发布里程计信息：将当前状态转换为TF变换并发布
  // 参数：xc - 当前IMU状态（包含旋转矩阵R和位置p）
  void pub_odom_func(IMUST &xc)
  {
    Eigen::Quaterniond q_this(xc.R);  // 将旋转矩阵转换为四元数
    Eigen::Vector3d t_this = xc.p;    // 获取位置向量

    static tf::TransformBroadcaster br;  // 静态TF广播器
    tf::Transform transform;
    tf::Quaternion q;
    // 设置平移部分
    transform.setOrigin(tf::Vector3(t_this.x(), t_this.y(), t_this.z()));
    // 设置旋转部分（四元数）
    q.setW(q_this.w());
    q.setX(q_this.x());
    q.setY(q_this.y());
    q.setZ(q_this.z());
    transform.setRotation(q);
    ros::Time ct = ros::Time::now();
    // 发布从camera_init到aft_mapped的坐标变换
    br.sendTransform(tf::StampedTransform(transform, ct, "/camera_init", "/aft_mapped"));
  }

  // 发布局部轨迹：发布当前扫描点云和轨迹路径
  // 参数：pwld - 世界坐标系下的点云
  //      jour - 当前行程距离
  //      x_curr - 当前IMU状态
  //      cur_session - 当前会话ID
  //      pcl_path - 轨迹路径点云（输出）
  void pub_localtraj(PLV(3) &pwld, double jour, IMUST &x_curr, int cur_session, pcl::PointCloud<PointType> &pcl_path)
  {
    pub_odom_func(x_curr);  // 发布里程计信息
    pcl::PointCloud<PointType> pcl_send;
    pcl_send.reserve(pwld.size());
    // 将世界坐标系下的点云转换为PointType格式
    for(Eigen::Vector3d &pw: pwld)
    {
      Eigen::Vector3d pvec = pw;
      PointType ap;
      ap.x = pvec.x();
      ap.y = pvec.y();
      ap.z = pvec.z();
      pcl_send.push_back(ap);
    }
    pub_pl_func(pcl_send, pub_scan);  // 发布当前扫描点云

    Eigen::Vector3d pcurr = x_curr.p;  // 当前位置

    // 将当前位置添加到轨迹路径
    PointType ap;
    ap.x = pcurr[0];
    ap.y = pcurr[1];
    ap.z = pcurr[2];
    ap.curvature = jour;        // 使用曲率字段存储行程距离
    ap.intensity = cur_session; // 使用强度字段存储会话ID
    pcl_path.push_back(ap);
    pub_pl_func(pcl_path, pub_curr_path);  // 发布轨迹路径
  }

  // 发布局部地图：发布滑动窗口内的点云地图和轨迹路径
  // 参数：mgsize - 边缘化的帧数
  //      cur_session - 当前会话ID
  //      pvec_buf - 点云缓冲区
  //      x_buf - 状态缓冲区
  //      pcl_path - 轨迹路径
  //      win_base - 窗口基准索引
  //      win_count - 窗口内帧数
  void pub_localmap(int mgsize, int cur_session, vector<PVecPtr> &pvec_buf, vector<IMUST> &x_buf, pcl::PointCloud<PointType> &pcl_path, int win_base, int win_count)
  {
    pcl::PointCloud<PointType> pcl_send;
    // 将局部坐标系的点云转换到世界坐标系
    for(int i=0; i<mgsize; i++)
    {
      for(int j=0; j<pvec_buf[i]->size(); j+=3)  // 每隔3个点采样一次
      {
        pointVar &pv = pvec_buf[i]->at(j);
        Eigen::Vector3d pvec = x_buf[i].R*pv.pnt + x_buf[i].p;  // 转换到世界坐标系
        PointType ap;
        ap.x = pvec[0];
        ap.y = pvec[1];
        ap.z = pvec[2];
        ap.intensity = cur_session;
        pcl_send.push_back(ap);
      }
    }

    // 更新轨迹路径点
    for(int i=0; i<win_count; i++)
    {
      Eigen::Vector3d pcurr = x_buf[i].p;
      pcl_path[i+win_base].x = pcurr[0];
      pcl_path[i+win_base].y = pcurr[1];
      pcl_path[i+win_base].z = pcurr[2];
    }

    pub_pl_func(pcl_path, pub_curr_path);  // 发布轨迹路径
    pub_pl_func(pcl_send, pub_cmap);       // 发布当前地图
  }

  // 发布全局路径：发布多会话的全局轨迹路径
  // 参数：relc_bl_buf - 重定位块缓冲区（多会话）
  //      pub_relc - ROS发布器
  //      ids - 需要发布的会话ID列表
  void pub_global_path(vector<vector<ScanPose*>*> &relc_bl_buf, ros::Publisher &pub_relc, vector<int> &ids)
  {
    pcl::PointCloud<pcl::PointXYZI> pl;
    pcl::PointXYZI pp;
    int idsize = ids.size();

    // 遍历所有指定会话，提取位置点
    for(int i=0; i<idsize; i++)
    {
      pp.intensity = ids[i];  // 使用强度字段标记会话ID
      for(ScanPose* bl: *(relc_bl_buf[ids[i]]))
      {
        pp.x = bl->x.p[0]; pp.y = bl->x.p[1]; pp.z = bl->x.p[2];
        pl.push_back(pp);
      }
    }
    pub_pl_func(pl, pub_relc);  // 发布全局路径
  }

  // 发布全局地图：发布多会话的全局关键帧点云地图
  // 参数：relc_submaps - 重定位子地图（多会话）
  //      ids - 需要发布的会话ID列表
  //      pub - ROS发布器
  void pub_globalmap(vector<vector<Keyframe*>*> &relc_submaps, vector<int> &ids, ros::Publisher &pub)
  {
    pcl::PointCloud<pcl::PointXYZI> pl;
    pub_pl_func(pl, pub);  // 先发布空点云清空之前的显示
    pcl::PointXYZI pp;

    uint interval_size = 5e6;  // 每次发布的最大点数（500万）
    uint psize = 0;
    // 统计总点数
    for(int id: ids)
    {
      vector<Keyframe*> &smps = *(relc_submaps[id]);
      for(int i=0; i<smps.size(); i++)
        psize += smps[i]->plptr->size();
    }
    int jump = psize / (10 * interval_size) + 1;  // 计算降采样跳跃步长

    // 遍历所有会话和关键帧，发布点云
    for(int id: ids)
    {
      pp.intensity = id;  // 使用强度字段标记会话ID
      vector<Keyframe*> &smps = *(relc_submaps[id]);
      for(int i=0; i<smps.size(); i++)
      {
        IMUST xx = smps[i]->x0;  // 关键帧的位姿
        // 降采样发布点云（每jump个点取一个）
        for(int j=0; j<smps[i]->plptr->size(); j+=jump)
        // for(int j=0; j<smps[i]->plptr->size(); j+=1)
        {
          PointType &ap = smps[i]->plptr->points[j];
          Eigen::Vector3d vv(ap.x, ap.y, ap.z);
          vv = xx.R * vv + xx.p;  // 转换到世界坐标系
          pp.x = vv[0]; pp.y = vv[1]; pp.z = vv[2];
          pl.push_back(pp);
        }

        // 分批发布，避免单次发布数据过大
        if(pl.size() > interval_size)
        {
          pub_pl_func(pl, pub);
          sleep(0.05);  // 短暂延时，避免网络拥塞
          pl.clear();
        }
      }
    }
    pub_pl_func(pl, pub);  // 发布剩余点云
  }

};

// FileReaderWriter类：负责文件的读写操作
// 包括PCD文件保存、位姿保存、回环边的IO以及离线地图加载
class FileReaderWriter
{
public:
  // 单例模式：获取FileReaderWriter的唯一实例
  static FileReaderWriter &instance()
  {
    static FileReaderWriter inst;
    return inst;
  }

  // 保存PCD文件：将点云数据保存为PCD格式
  // 参数：pptr - 点云数据指针
  //      xx - IMU状态（未使用）
  //      count - 帧计数（用于文件命名）
  //      savename - 保存路径
  void save_pcd(PVecPtr pptr, IMUST &xx, int count, const string &savename)
  {
    pcl::PointCloud<pcl::PointXYZI> pl_save;
    // 将自定义点格式转换为PCL标准格式
    for(pointVar &pw: *pptr)
    {
      pcl::PointXYZI ap;
      ap.x = pw.pnt[0]; ap.y = pw.pnt[1]; ap.z = pw.pnt[2];
      pl_save.push_back(ap);
    }
    string pcdname = savename + "/" + to_string(count) + ".pcd";
    pcl::io::savePCDFileBinary(pcdname, pl_save);  // 以二进制格式保存
  }

  // 保存位姿文件：将扫描位姿保存到文本文件
  // 参数：bbuf - 扫描位姿缓冲区
  //      fname - 文件名
  //      posename - 位姿文件后缀
  //      savepath - 保存路径
  void save_pose(vector<ScanPose*> &bbuf, string &fname, string posename, string &savepath)
  {
    if(bbuf.size() < 100) return;  // 数据量太少则不保存
    int topsize = bbuf.size();

    ofstream posfile(savepath + fname + posename);
    // 保存每一帧的完整状态
    for(int i=0; i<topsize; i++)
    {
      IMUST &xx = bbuf[i]->x;
      Eigen::Quaterniond qq(xx.R);  // 旋转矩阵转四元数
      posfile << fixed << setprecision(6) << xx.t << " ";  // 时间戳
      posfile << setprecision(7) << xx.p[0] << " " << xx.p[1] << " " << xx.p[2] << " ";  // 位置
      posfile << qq.x() << " " << qq.y() << " " << qq.z() << " " << qq.w();  // 四元数
      posfile << " " << xx.v[0] << " " << xx.v[1] << " " << xx.v[2];  // 速度
      posfile << " " << xx.bg[0] << " " << xx.bg[1] << " " << xx.bg[2];  // 陀螺仪偏置
      posfile << " " << xx.ba[0] << " " << xx.ba[1] << " " << xx.ba[2];  // 加速度计偏置
      posfile << " " << xx.g[0] << " " << xx.g[1] << " " << xx.g[2];  // 重力向量
      for(int j=0; j<6; j++) posfile << " " << bbuf[i]->v6[j];  // 6维协方差
      posfile << endl;
    }
    posfile.close();

  }

  // 多会话回环闭合边的输入输出
  // 参数：edges - PGO边集合
  //      fnames - 文件名列表
  //      io - 0表示读取，1表示写入
  //      savepath - 保存路径
  //      bagname - 当前bag文件名
  void pgo_edges_io(PGO_Edges &edges, vector<string> &fnames, int io, string &savepath, string &bagname)
  {
    static vector<string> seq_absent;  // 存储不存在的序列
    Eigen::Matrix<double, 6, 1> v6_init;
    v6_init << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6;  // 初始协方差
    if(io == 0) // 读取模式
    {
      ifstream infile(savepath + "edge.txt");
      string lineStr, str;
      vector<string> sts;
      // 逐行读取边文件
      while(getline(infile, lineStr))
      {
        sts.clear();
        stringstream ss(lineStr);
        while(ss >> str)
          sts.push_back(str);

        // 查找文件名对应的索引
        int mp[2] = {-1, -1};
        for(int i=0; i<2; i++)
        for(int j=0; j<fnames.size(); j++)
        if(sts[i] == fnames[j])
        {
          mp[i] = j;
          break;
        }

        // 如果两个文件名都找到了，添加边
        if(mp[0] != -1 && mp[1] != -1)
        {
          int id1 = stoi(sts[2]);  // 第一个关键帧ID
          int id2 = stoi(sts[3]);  // 第二个关键帧ID
          Eigen::Vector3d v3;      // 平移向量
          v3 << stod(sts[4]), stod(sts[5]), stod(sts[6]);
          Eigen::Quaterniond qq(stod(sts[10]), stod(sts[7]), stod(sts[8]), stod(sts[9]));  // 四元数
          Eigen::Matrix3d rot(qq.matrix());  // 旋转矩阵
          // 确保索引顺序正确
          if(mp[0] <= mp[1])
            edges.push(mp[0], mp[1], id1, id2, rot, v3, v6_init);
          else
          {
            // 交换顺序，需要反转变换
            v3 = -rot.transpose() * v3;
            rot = qq.matrix().transpose();
            edges.push(mp[1], mp[0], id2, id1, rot, v3, v6_init);
          }
        }
        else
        {
          // 如果不涉及当前bagname，保存到缺失序列
          if(sts[0] != bagname && sts[1] != bagname)
            seq_absent.push_back(lineStr);
        }

      }
    }
    else // 写入模式
    {
      ofstream outfile(savepath + "edge.txt");
      // 先写入之前不存在的序列
      for(string &str: seq_absent)
        outfile << str << endl;

      // 写入所有边信息
      for(PGO_Edge &edge: edges.edges)
      {
        for(int i=0; i<edge.rots.size(); i++)
        {
          outfile << fnames[edge.m1] << " ";  // 第一个地图名
          outfile << fnames[edge.m2] << " ";  // 第二个地图名
          outfile << edge.ids1[i] << " ";     // 第一个关键帧ID
          outfile << edge.ids2[i] << " ";     // 第二个关键帧ID
          Eigen::Vector3d v(edge.tras[i]);   // 平移向量
          outfile << setprecision(7) << v[0] << " " << v[1] << " " << v[2] << " ";
          Eigen::Quaterniond qq(edge.rots[i]);  // 旋转四元数
          outfile << qq.x() << " " << qq.y() << " " << qq.z() << " " << qq.w() << endl;
        }
      }
      outfile.close();
    }

  }

  // 加载离线地图：解析离线地图名称和判断阈值
  // 参数：n - ROS节点句柄
  //      fnames - 输出的文件名列表
  //      juds - 输出的判断阈值列表
  void previous_map_names(ros::NodeHandle &n, vector<string> &fnames, vector<double> &juds)
  {
    string premap;
    n.param<string>("General/previous_map", premap, "");
    premap.erase(remove_if(premap.begin(), premap.end(), ::isspace), premap.end());  // 删除空格
    stringstream ss(premap);
    string str;
    // 解析逗号分隔的地图列表
    while(getline(ss, str, ','))
    {
      stringstream ss2(str);
      vector<string> strs;
      // 解析冒号分隔的名称和阈值
      while(getline(ss2, str, ':'))
        strs.push_back(str);

      if(strs.size() != 2)
      {
        printf("previous map name wrong\n");
        return;
      }

      // 如果不是注释（#开头），则添加到列表
      if(strs[0][0] != '#')
      {
        fnames.push_back(strs[0]);        // 地图名称
        juds.push_back(stod(strs[1]));    // 判断阈值
      }
    }

  }

  // 读取离线地图：加载之前保存的地图和位姿数据
  // 参数：std_managers - 场景描述子管理器列表（输出）
  //      multimap_scanPoses - 多地图扫描位姿（输出）
  //      multimap_keyframes - 多地图关键帧（输出）
  //      config_setting - 配置设置
  //      edges - PGO边（输入输出）
  //      n - ROS节点句柄
  //      fnames - 文件名列表
  //      juds - 判断阈值列表
  //      savepath - 保存路径
  //      win_size - 窗口大小
  void previous_map_read(vector<STDescManager*> &std_managers, vector<vector<ScanPose*>*> &multimap_scanPoses, vector<vector<Keyframe*>*> &multimap_keyframes, ConfigSetting &config_setting, PGO_Edges &edges, ros::NodeHandle &n, vector<string> &fnames, vector<double> &juds, string &savepath, int win_size)
  {
    int acsize = 10; int mgsize = 5;  // 累积大小和边缘化大小
    n.param<int>("Loop/acsize", acsize, 10);
    n.param<int>("Loop/mgsize", mgsize, 5);

    for(int fn=0; fn<fnames.size() && n.ok(); fn++)
    {
      string fname = savepath + fnames[fn];
      vector<ScanPose*>* bl_tem = new vector<ScanPose*>();
      vector<Keyframe*>* keyframes_tem = new vector<Keyframe*>();
      STDescManager *std_manager = new STDescManager(config_setting);

      std_managers.push_back(std_manager);
      multimap_scanPoses.push_back(bl_tem);
      multimap_keyframes.push_back(keyframes_tem);
      read_lidarstate(fname+"/alidarState.txt", *bl_tem);

      cout << "Reading " << fname << ": " << bl_tem->size() << " scans." << "\n";
      deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> plbuf;
      deque<IMUST> xxbuf;
      pcl::PointCloud<PointType> pl_lc;
      pcl::PointCloud<pcl::PointXYZI>::Ptr pl_btc(new pcl::PointCloud<pcl::PointXYZI>());

      for(int i=0; i<bl_tem->size() && n.ok(); i++)
      {
        IMUST &xc = bl_tem->at(i)->x;
        string pcdname = fname + "/" + to_string(i) + ".pcd";
        pcl::PointCloud<pcl::PointXYZI>::Ptr pl_tem(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::io::loadPCDFile(pcdname, *pl_tem);

        xxbuf.push_back(xc);
        plbuf.push_back(pl_tem);
        
        if(xxbuf.size() < win_size)
          continue;
        
        pl_lc.clear();
        Keyframe *smp = new Keyframe(xc);
        smp->id = i;
        PointType pt;
        for(int j=0; j<win_size; j++)
        {
          Eigen::Vector3d delta_p = xc.R.transpose() * (xxbuf[j].p - xc.p);
          Eigen::Matrix3d delta_R = xc.R.transpose() *  xxbuf[j].R;

          for(pcl::PointXYZI pp: plbuf[j]->points)
          {
            Eigen::Vector3d v3(pp.x, pp.y, pp.z);
            v3 = delta_R * v3 + delta_p;
            pt.x = v3[0]; pt.y = v3[1]; pt.z = v3[2];
            pl_lc.push_back(pt);
          }
        }

        down_sampling_voxel(pl_lc, voxel_size/10);
        smp->plptr->reserve(pl_lc.size());
        for(PointType &pp: pl_lc.points)
          smp->plptr->push_back(pp);
        keyframes_tem->push_back(smp);
        
        for(int j=0; j<win_size; j++)
        {
          plbuf.pop_front(); xxbuf.pop_front();
        }
      }
      
      cout << "Generating BTC descriptors..." << "\n";

      int subsize = keyframes_tem->size();
      for(int i=0; i+acsize<subsize && n.ok(); i+=mgsize)
      {
        int up = i + acsize;
        pl_btc->clear();
        IMUST &xc = keyframes_tem->at(up - 1)->x0;
        for(int j=i; j<up; j++)
        {
          IMUST &xj = keyframes_tem->at(j)->x0;
          Eigen::Vector3d delta_p = xc.R.transpose() * (xj.p - xc.p);
          Eigen::Matrix3d delta_R = xc.R.transpose() *  xj.R;
          pcl::PointXYZI pp;
          for(PointType ap: keyframes_tem->at(j)->plptr->points)
          {
            Eigen::Vector3d v3(ap.x, ap.y, ap.z);
            v3 = delta_R * v3 + delta_p;
            pp.x = v3[0]; pp.y = v3[1]; pp.z = v3[2];
            pl_btc->push_back(pp);
          }
        }

        vector<STD> stds_vec;
        std_manager->GenerateSTDescs(pl_btc, stds_vec, keyframes_tem->at(up-1)->id);
        std_manager->AddSTDescs(stds_vec);
      }
      std_manager->config_setting_.skip_near_num_ = -(std_manager->plane_cloud_vec_.size()+10);

      cout << "Read " << fname << " done." << "\n\n";
    }

    vector<int> ids_all;
    for(int fn=0; fn<fnames.size() && n.ok(); fn++)
      ids_all.push_back(fn);

    // gtsam::Values initial;
    // gtsam::NonlinearFactorGraph graph;
    // vector<int> ids_cnct, stepsizes;
    // Eigen::Matrix<double, 6, 1> v6_init;
    // v6_init << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
    // gtsam::noiseModel::Diagonal::shared_ptr odom_noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector(v6_init));
    // build_graph(initial, graph, ids_all.back(), edges, odom_noise, ids_cnct, stepsizes, 1);

    // gtsam::ISAM2Params parameters;
    // parameters.relinearizeThreshold = 0.01;
    // parameters.relinearizeSkip = 1;
    // gtsam::ISAM2 isam(parameters);
    // isam.update(graph, initial);

    // for(int i=0; i<5; i++) isam.update();
    // gtsam::Values results = isam.calculateEstimate();
    // int resultsize = results.size();
    // int idsize = ids_cnct.size();
    // for(int ii=0; ii<idsize; ii++)
    // {
    //   int tip = ids_cnct[ii];
    //   for(int j=stepsizes[ii]; j<stepsizes[ii+1]; j++)
    //   {
    //     int ord = j - stepsizes[ii];
    //     multimap_scanPoses[tip]->at(ord)->set_state(results.at(j).cast<gtsam::Pose3>());
    //   }
    // }
    // for(int ii=0; ii<idsize; ii++)
    // {
    //   int tip = ids_cnct[ii];
    //   for(Keyframe *kf: *multimap_keyframes[tip])
    //     kf->x0 = multimap_scanPoses[tip]->at(kf->id)->x;
    // }

    ResultOutput::instance().pub_global_path(multimap_scanPoses, pub_prev_path, ids_all);
    ResultOutput::instance().pub_globalmap(multimap_keyframes, ids_all, pub_pmap);

    printf("All the maps are loaded\n");
  }
  
};

// Initialization类：负责SLAM系统的初始化
// 包括重力对齐、运动模糊补偿和运动初始化
class Initialization
{
public:
  // 单例模式：获取Initialization的唯一实例
  static Initialization &instance()
  {
    static Initialization inst;
    return inst;
  }

  // 重力对齐：将重力向量对齐到Z轴方向
  // 参数：xs - 状态序列（包含重力向量）
  void align_gravity(vector<IMUST> &xs)
  {
    Eigen::Vector3d g0 = xs[0].g;  // 初始重力向量
    Eigen::Vector3d n0 = g0 / g0.norm();  // 归一化
    Eigen::Vector3d n1(0, 0, 1);  // 目标方向（Z轴正方向）
    if(n0[2] < 0)
      n1[2] = -1;  // 如果重力向下，目标方向为Z轴负方向

    // 计算从n0到n1的旋转
    Eigen::Vector3d rotvec = n0.cross(n1);  // 旋转轴
    double rnorm = rotvec.norm();  // 旋转角度的正弦值
    rotvec = rotvec / rnorm;  // 归一化旋转轴

    Eigen::AngleAxisd angaxis(asin(rnorm), rotvec);  // 轴角表示
    Eigen::Matrix3d rot = angaxis.matrix();  // 转换为旋转矩阵
    g0 = rot * g0;  // 旋转后的重力向量

    // 对所有状态应用相同的旋转
    Eigen::Vector3d p0 = xs[0].p;
    for(int i=0; i<xs.size(); i++)
    {
      xs[i].p = rot * (xs[i].p - p0) + p0;  // 旋转位置
      xs[i].R = rot * xs[i].R;              // 旋转姿态
      xs[i].v = rot * xs[i].v;              // 旋转速度
      xs[i].g = g0;                         // 更新重力向量
    }

  }

  // 运动模糊补偿：使用IMU数据对激光点云进行去畸变
  // 参数：pl - 输入点云
  //      pvec - 输出的点变量向量
  //      xc - 当前帧状态
  //      xl - 上一帧状态
  //      imus - IMU数据队列
  //      pcl_beg_time - 点云起始时间
  //      extrin_para - 激光到IMU的外参
  void motion_blur(pcl::PointCloud<PointType> &pl, PVec &pvec, IMUST xc, IMUST xl, deque<sensor_msgs::Imu::Ptr> &imus, double pcl_beg_time, IMUST &extrin_para)
  {
    xc.bg = xl.bg; xc.ba = xl.ba;  // 使用上一帧的偏置
    Eigen::Vector3d acc_imu, angvel_avr, acc_avr, vel_imu(xc.v), pos_imu(xc.p);
    Eigen::Matrix3d R_imu(xc.R);
    vector<IMUST> imu_poses;  // 存储IMU积分位姿

    // 反向遍历IMU数据，进行积分
    for(auto it_imu=imus.end()-1; it_imu!=imus.begin(); it_imu--)
    {
      sensor_msgs::Imu &head = **(it_imu-1);
      sensor_msgs::Imu &tail = **(it_imu);

      // 计算平均角速度
      angvel_avr << 0.5*(head.angular_velocity.x + tail.angular_velocity.x),
                    0.5*(head.angular_velocity.y + tail.angular_velocity.y),
                    0.5*(head.angular_velocity.z + tail.angular_velocity.z);
      // 计算平均加速度
      acc_avr << 0.5*(head.linear_acceleration.x + tail.linear_acceleration.x),
                 0.5*(head.linear_acceleration.y + tail.linear_acceleration.y),
                 0.5*(head.linear_acceleration.z + tail.linear_acceleration.z);

      angvel_avr -= xc.bg;  // 减去陀螺仪偏置
      acc_avr = acc_avr * imupre_scale_gravity - xc.ba;  // 重力缩放并减去加速度计偏置

      double dt = head.header.stamp.toSec() - tail.header.stamp.toSec();
      Eigen::Matrix3d acc_avr_skew = hat(acc_avr);
      Eigen::Matrix3d Exp_f = Exp(angvel_avr, dt);  // 旋转增量

      // IMU运动学积分
      acc_imu = R_imu * acc_avr + xc.g;  // 世界坐标系加速度
      pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;  // 位置更新
      vel_imu = vel_imu + acc_imu * dt;  // 速度更新
      R_imu = R_imu * Exp_f;  // 姿态更新

      double offt = head.header.stamp.toSec() - pcl_beg_time;
      imu_poses.emplace_back(offt, R_imu, pos_imu, vel_imu, angvel_avr, acc_imu);
    }

    pointVar pv; pv.var.setIdentity();
    // 如果点云没有时间戳，直接转换到IMU坐标系
    if(point_notime)
    {
      for(PointType &ap: pl.points)
      {
        pv.pnt << ap.x, ap.y, ap.z;
        pv.pnt = extrin_para.R * pv.pnt + extrin_para.p;  // 激光坐标系到IMU坐标系
        pvec.push_back(pv);
      }
      return;
    }
    // 如果有时间戳，进行去畸变处理
    auto it_pcl = pl.end() - 1;
    // for(auto it_kp=imu_poses.end(); it_kp!=imu_poses.begin(); it_kp--)
    for(auto it_kp=imu_poses.begin(); it_kp!=imu_poses.end(); it_kp++)
    {
      // IMUST &head = *(it_kp - 1);
      IMUST &head = *it_kp;
      R_imu = head.R;
      acc_imu = head.ba;
      vel_imu = head.v;
      pos_imu = head.p;
      angvel_avr = head.bg;

      // 对时间戳大于当前IMU时刻的点进行补偿
      for(; it_pcl->curvature > head.t; it_pcl--)
      {
        double dt = it_pcl->curvature - head.t;
        Eigen::Matrix3d R_i = R_imu * Exp(angvel_avr, dt);  // 该点时刻的姿态
        Eigen::Vector3d T_ei = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - xc.p;  // 该点时刻的位置

        Eigen::Vector3d P_i(it_pcl->x, it_pcl->y, it_pcl->z);
        // 将点补偿到扫描结束时刻
        Eigen::Vector3d P_compensate = xc.R.transpose() * (R_i * (extrin_para.R * P_i + extrin_para.p) + T_ei);

        pv.pnt = P_compensate;
        pvec.push_back(pv);
        if(it_pcl == pl.begin()) break;
      }

    }
  }

  // 运动初始化：使用滑动窗口内的数据估计初始状态和重力方向
  // 参数：pl_origs - 原始点云序列
  //      vec_imus - IMU数据序列
  //      beg_times - 起始时间序列
  //      hess - Hessian矩阵（输出）
  //      voxhess - 体素因子
  //      x_buf - 状态缓冲区（输入输出）
  //      surf_map - 体素地图（输出）
  //      surf_map_slide - 滑动窗口地图（输出）
  //      pvec_buf - 点云缓冲区（输出）
  //      win_size - 窗口大小
  //      sws - 滑动窗口
  //      x_curr - 当前状态（输出）
  //      imu_pre_buf - IMU预积分缓冲区（输出）
  //      extrin_para - 外参
  // 返回：1表示初始化成功，0表示失败
  int motion_init(vector<pcl::PointCloud<PointType>::Ptr> &pl_origs, vector<deque<sensor_msgs::Imu::Ptr>> &vec_imus, vector<double> &beg_times, Eigen::MatrixXd *hess, LidarFactor &voxhess, vector<IMUST> &x_buf, unordered_map<VOXEL_LOC, OctoTree*> &surf_map, unordered_map<VOXEL_LOC, OctoTree*> &surf_map_slide, vector<PVecPtr> &pvec_buf, int win_size, vector<vector<SlideWindow*>> &sws, IMUST &x_curr, deque<IMU_PRE*> &imu_pre_buf, IMUST &extrin_para)
  {
    PLV(3) pwld;
    double last_g_norm = x_buf[0].g.norm();
    int converge_flag = 0;  // 收敛标志

    double min_eigen_value_orig = min_eigen_value;
    vector<double> eigen_value_array_orig = plane_eigen_value_thre;

    min_eigen_value = 0.02;
    for(double &iter: plane_eigen_value_thre)
      iter = 1.0 / 4;

    double t0 = ros::Time::now().toSec();
    double converge_thre = 0.05;
    int converge_times = 0;
    bool is_degrade = true;
    Eigen::Vector3d eigvalue; eigvalue.setZero();
    for(int iterCnt = 0; iterCnt < 10; iterCnt++)
    {
      if(converge_flag == 1)
      {
        min_eigen_value = min_eigen_value_orig;
        plane_eigen_value_thre = eigen_value_array_orig;
      }

      vector<OctoTree*> octos;
      for(auto iter=surf_map.begin(); iter!=surf_map.end(); ++iter)
      {
        iter->second->tras_ptr(octos);
        iter->second->clear_slwd(sws[0]);
        delete iter->second;
      }
      for(int i=0; i<octos.size(); i++)
        delete octos[i];
      surf_map.clear(); octos.clear(); surf_map_slide.clear();

      for(int i=0; i<win_size; i++)
      {
        pwld.clear();
        pvec_buf[i]->clear();
        int l = i==0 ? i : i - 1;
        motion_blur(*pl_origs[i], *pvec_buf[i], x_buf[i], x_buf[l], vec_imus[i], beg_times[i], extrin_para);

        if(converge_flag == 1)
        {
          for(pointVar &pv: *pvec_buf[i])
            calcBodyVar(pv.pnt, dept_err, beam_err, pv.var);
          pvec_update(pvec_buf[i], x_buf[i], pwld);
        }
        else
        {
          for(pointVar &pv: *pvec_buf[i])
            pwld.push_back(x_buf[i].R * pv.pnt + x_buf[i].p);
        }

        cut_voxel(surf_map, pvec_buf[i], i, surf_map_slide, win_size, pwld, sws[0]);
      }

      // LidarFactor voxhess(win_size);
      voxhess.clear(); voxhess.win_size = win_size;
      for(auto iter=surf_map.begin(); iter!=surf_map.end(); ++iter)
      {
        iter->second->recut(win_size, x_buf, sws[0]);
        iter->second->tras_opt(voxhess);
      }

      if(voxhess.plvec_voxels.size() < 10)
        break;
      LI_BA_OptimizerGravity opt_lsv;
      vector<double> resis;
      opt_lsv.damping_iter(x_buf, voxhess, imu_pre_buf, resis, hess, 3);
      Eigen::Matrix3d nnt; nnt.setZero();

      printf("%d: %lf %lf %lf: %lf %lf\n", iterCnt, x_buf[0].g[0], x_buf[0].g[1], x_buf[0].g[2], x_buf[0].g.norm(), fabs(resis[0] - resis[1]) / resis[0]);

      for(int i=0; i<win_size-1; i++)
        delete imu_pre_buf[i];
      imu_pre_buf.clear();

      for(int i=1; i<win_size; i++)
      {
        imu_pre_buf.push_back(new IMU_PRE(x_buf[i-1].bg, x_buf[i-1].ba));
        imu_pre_buf.back()->push_imu(vec_imus[i]);
      }

      if(fabs(resis[0] - resis[1]) / resis[0] < converge_thre && iterCnt >= 2)
      {
        for(Eigen::Matrix3d &iter: voxhess.eig_vectors)
        {
          Eigen::Vector3d v3 = iter.col(0);
          nnt += v3 * v3.transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(nnt);
        eigvalue = saes.eigenvalues();
        is_degrade = eigvalue[0] < 15 ? true : false;

        converge_thre = 0.01;
        if(converge_flag == 0)
        {
          align_gravity(x_buf);
          converge_flag = 1;
          continue;
        }
        else
          break;
      }
    }

    x_curr = x_buf[win_size - 1];
    double gnm = x_curr.g.norm();
    if(is_degrade || gnm < 9.6 || gnm > 10.0)
    {
      converge_flag = 0;
    }
    if(converge_flag == 0)
    {
      vector<OctoTree*> octos;
      for(auto iter=surf_map.begin(); iter!=surf_map.end(); ++iter)
      {
        iter->second->tras_ptr(octos);
        iter->second->clear_slwd(sws[0]);
        delete iter->second;
      }
      for(int i=0; i<octos.size(); i++)
        delete octos[i];
      surf_map.clear(); octos.clear(); surf_map_slide.clear();
    }

    printf("mn: %lf %lf %lf\n", eigvalue[0], eigvalue[1], eigvalue[2]);
    Eigen::Vector3d angv(vec_imus[0][0]->angular_velocity.x, vec_imus[0][0]->angular_velocity.y, vec_imus[0][0]->angular_velocity.z);
    Eigen::Vector3d acc(vec_imus[0][0]->linear_acceleration.x, vec_imus[0][0]->linear_acceleration.y, vec_imus[0][0]->linear_acceleration.z);
    acc *= 9.8;

    pl_origs.clear(); vec_imus.clear(); beg_times.clear();
    double t1 = ros::Time::now().toSec();
    printf("init time: %lf\n", t1 - t0);

    // align_gravity(x_buf);
    pcl::PointCloud<PointType> pcl_send; PointType pt;
    for(int i=0; i<win_size; i++)
    for(pointVar &pv: *pvec_buf[i])
    {
      Eigen::Vector3d vv = x_buf[i].R * pv.pnt + x_buf[i].p;
      pt.x = vv[0]; pt.y = vv[1]; pt.z = vv[2];
      pcl_send.push_back(pt);
    }
    pub_pl_func(pcl_send, pub_init);

    return converge_flag;
  }

};

// VOXEL_SLAM类：Voxel-SLAM的主类
// 包含里程计、局部建图、回环检测和全局优化等核心功能
class VOXEL_SLAM
{
public:
  pcl::PointCloud<PointType> pcl_path;  // 轨迹路径点云
  IMUST x_curr, extrin_para;  // 当前状态和外参
  IMUEKF odom_ekf;  // IMU-EKF里程计
  unordered_map<VOXEL_LOC, OctoTree*> surf_map, surf_map_slide;  // 体素地图和滑动窗口地图
  double down_size;  // 降采样尺寸

  // 滑动窗口相关
  int win_size;              // 滑动窗口大小
  vector<IMUST> x_buf;       // 状态缓冲区
  vector<PVecPtr> pvec_buf;  // 点云缓冲区
  deque<IMU_PRE*> imu_pre_buf;  // IMU预积分缓冲区
  int win_count = 0, win_base = 0;  // 窗口计数和基准索引
  vector<vector<SlideWindow*>> sws;  // 滑动窗口

  // 回环检测相关
  vector<ScanPose*> *scanPoses;  // 扫描位姿
  mutex mtx_loop;  // 回环检测互斥锁
  deque<ScanPose*> buf_lba2loop, buf_lba2loop_tem;  // 局部BA到回环的缓冲区
  vector<Keyframe*> *keyframes;  // 关键帧
  int loop_detect = 0;  // 回环检测标志
  unordered_map<VOXEL_LOC, OctoTree*> map_loop;  // 回环地图
  IMUST dx;  // 回环修正的变换
  pcl::PointCloud<PointType>::Ptr pl_kdmap;  // 关键帧KD树点云
  pcl::KdTreeFLANN<PointType> kd_keyframes;  // 关键帧KD树
  int history_kfsize = 0;  // 历史关键帧数量
  vector<OctoTree*> octos_release;  // 待释放的八叉树节点
  int reset_flag = 0;  // 重置标志
  int g_update = 0;  // 重力更新标志
  int thread_num = 5;  // 线程数
  int degrade_bound = 10;  // 退化边界

  // 多会话相关
  vector<vector<ScanPose*>*> multimap_scanPoses;  // 多地图扫描位姿
  vector<vector<Keyframe*>*> multimap_keyframes;  // 多地图关键帧
  volatile int gba_flag = 0;  // 全局BA标志
  int gba_size = 0;  // 全局BA大小
  vector<int> cnct_map;  // 连接的地图ID
  mutex mtx_keyframe;  // 关键帧互斥锁
  PGO_Edges gba_edges1, gba_edges2;  // 位姿图优化的边（两层）
  bool is_finish = false;  // 结束标志

  // 文件保存相关
  vector<string> sessionNames;  // 会话名称列表
  string bagname, savepath;  // bag文件名和保存路径
  int is_save_map;  // 是否保存地图

  VOXEL_SLAM(ros::NodeHandle &n)
  {
    double cov_gyr, cov_acc, rand_walk_gyr, rand_walk_acc;
    vector<double> vecR(9), vecT(3);
    scanPoses = new vector<ScanPose*>();
    keyframes = new vector<Keyframe*>();
    
    string lid_topic, imu_topic;
    n.param<string>("General/lid_topic", lid_topic, "/livox/lidar");
    n.param<string>("General/imu_topic", imu_topic, "/livox/imu");
    n.param<string>("General/bagname", bagname, "site3_handheld_4");
    n.param<string>("General/save_path", savepath, "");
    n.param<int>("General/lidar_type", feat.lidar_type, 0);
    n.param<double>("General/blind", feat.blind, 0.1);
    n.param<int>("General/point_filter_num", feat.point_filter_num, 3);
    n.param<vector<double>>("General/extrinsic_tran", vecT, vector<double>());
    n.param<vector<double>>("General/extrinsic_rota", vecR, vector<double>());
    n.param<int>("General/is_save_map", is_save_map, 0);

    sub_imu = n.subscribe(imu_topic, 80000, imu_handler);
    if(feat.lidar_type == LIVOX)
      sub_pcl = n.subscribe<livox_ros_driver::CustomMsg>(lid_topic, 1000, pcl_handler);
    else
      sub_pcl = n.subscribe<sensor_msgs::PointCloud2>(lid_topic, 1000, pcl_handler);
    odom_ekf.imu_topic = imu_topic;

    n.param<double>("Odometry/cov_gyr", cov_gyr, 0.1);
    n.param<double>("Odometry/cov_acc", cov_acc, 0.1);
    n.param<double>("Odometry/rdw_gyr", rand_walk_gyr, 1e-4);
    n.param<double>("Odometry/rdw_acc", rand_walk_acc, 1e-4);
    n.param<double>("Odometry/down_size", down_size, 0.1);
    n.param<double>("Odometry/dept_err", dept_err, 0.02);
    n.param<double>("Odometry/beam_err", beam_err, 0.05);
    n.param<double>("Odometry/voxel_size", voxel_size, 1);
    n.param<double>("Odometry/min_eigen_value", min_eigen_value, 0.0025);
    n.param<int>("Odometry/degrade_bound", degrade_bound, 10);
    n.param<int>("Odometry/point_notime", point_notime, 0);
    odom_ekf.point_notime = point_notime;

    feat.blind = feat.blind * feat.blind;
    odom_ekf.cov_gyr << cov_gyr, cov_gyr, cov_gyr;
    odom_ekf.cov_acc << cov_acc, cov_acc, cov_acc;
    odom_ekf.cov_bias_gyr << rand_walk_gyr, rand_walk_gyr, rand_walk_gyr;
    odom_ekf.cov_bias_acc << rand_walk_acc, rand_walk_acc, rand_walk_acc;
    odom_ekf.Lid_offset_to_IMU  << vecT[0], vecT[1], vecT[2];
    odom_ekf.Lid_rot_to_IMU << vecR[0], vecR[1], vecR[2],
                            vecR[3], vecR[4], vecR[5],
                            vecR[6], vecR[7], vecR[8];                
    extrin_para.R = odom_ekf.Lid_rot_to_IMU;
    extrin_para.p = odom_ekf.Lid_offset_to_IMU;
    min_point << 5, 5, 5, 5;

    n.param<int>("LocalBA/win_size", win_size, 10);
    n.param<int>("LocalBA/max_layer", max_layer, 2);
    n.param<double>("LocalBA/cov_gyr", cov_gyr, 0.1);
    n.param<double>("LocalBA/cov_acc", cov_acc, 0.1);
    n.param<double>("LocalBA/rdw_gyr", rand_walk_gyr, 1e-4);
    n.param<double>("LocalBA/rdw_acc", rand_walk_acc, 1e-4);
    n.param<int>("LocalBA/min_ba_point", min_ba_point, 20);
    n.param<vector<double>>("LocalBA/plane_eigen_value_thre", plane_eigen_value_thre, vector<double>({1, 1, 1, 1}));
    n.param<double>("LocalBA/imu_coef", imu_coef, 1e-4);
    n.param<int>("LocalBA/thread_num", thread_num, 5);

    for(double &iter: plane_eigen_value_thre) iter = 1.0 / iter;
    // for(double &iter: plane_eigen_value_thre) iter = 1.0 / iter;

    noiseMeas.setZero(); noiseWalk.setZero();
    noiseMeas.diagonal() << cov_gyr, cov_gyr, cov_gyr, 
                            cov_acc, cov_acc, cov_acc;
    noiseWalk.diagonal() << 
    rand_walk_gyr, rand_walk_gyr, rand_walk_gyr, 
    rand_walk_acc, rand_walk_acc, rand_walk_acc;

    int ss = 0;
    if(access((savepath+bagname+"/").c_str(), X_OK) == -1)
    {
      string cmd = "mkdir " + savepath + bagname + "/";
      ss = system(cmd.c_str());
    }
    else
      ss = -1;

    if(ss != 0 && is_save_map == 1)
    {
      printf("The pointcloud will be saved in this run.\n");
      printf("So please clear or rename the existed folder.\n"); 
      exit(0);
    }

    sws.resize(thread_num);
    cout << "bagname: " << bagname << endl;
  }

  // 点到平面对齐的里程计估计
  // 参数：pptr - 输入点云
  // 返回：是否收敛（根据特征值判断）
  bool lio_state_estimation(PVecPtr pptr)
  {
    IMUST x_prop = x_curr;  // 传播状态（用于正则化）

    const int num_max_iter = 4;  // 最大迭代次数
    bool EKF_stop_flg = 0, flg_EKF_converged = 0;
    Eigen::Matrix<double, DIM, DIM> G, H_T_H, I_STATE;
    G.setZero(); H_T_H.setZero(); I_STATE.setIdentity();
    int rematch_num = 0;  // 重新匹配次数
    int match_num = 0;    // 匹配点数

    int psize = pptr->size();
    vector<OctoTree*> octos;  // 缓存匹配到的八叉树节点
    octos.resize(psize, nullptr);

    Eigen::Matrix3d nnt; 
    Eigen::Matrix<double, DIM, DIM> cov_inv = x_curr.cov.inverse();
    for(int iterCount=0; iterCount<num_max_iter; iterCount++)
    {
      Eigen::Matrix<double, 6, 6> HTH; HTH.setZero();
      Eigen::Matrix<double, 6, 1> HTz; HTz.setZero();
      Eigen::Matrix3d rot_var = x_curr.cov.block<3, 3>(0, 0);
      Eigen::Matrix3d tsl_var = x_curr.cov.block<3, 3>(3, 3);
      match_num = 0;
      nnt.setZero();

      for(int i=0; i<psize; i++)
      {
        pointVar &pv = pptr->at(i);
        Eigen::Matrix3d phat = hat(pv.pnt);
        Eigen::Matrix3d var_world = x_curr.R * pv.var * x_curr.R.transpose() + phat * rot_var * phat.transpose() + tsl_var;
        Eigen::Vector3d wld = x_curr.R * pv.pnt + x_curr.p;

        double sigma_d = 0;
        Plane* pla = nullptr;
        int flag = 0;
        if(octos[i] != nullptr && octos[i]->inside(wld))
        {
          double max_prob = 0;
          flag = octos[i]->match(wld, pla, max_prob, var_world, sigma_d, octos[i]);
        }
        else
        {
          flag = match(surf_map, wld, pla, var_world, sigma_d, octos[i]);
        }

        if(flag)
        // if(pla != nullptr)
        {
          Plane &pp = *pla;
          double R_inv = 1.0 / (0.0005 + sigma_d);
          double resi = pp.normal.dot(wld - pp.center);

          Eigen::Matrix<double, 6, 1> jac;
          jac.head(3) = phat * x_curr.R.transpose() * pp.normal;
          jac.tail(3) = pp.normal;
          HTH += R_inv * jac * jac.transpose();
          HTz -= R_inv * jac * resi;
          nnt += pp.normal * pp.normal.transpose();
          match_num++;
        }

      }

      H_T_H.block<6, 6>(0, 0) = HTH;
      Eigen::Matrix<double, DIM, DIM> K_1 = (H_T_H + cov_inv).inverse();
      G.block<DIM, 6>(0, 0) = K_1.block<DIM, 6>(0, 0) * HTH;
      Eigen::Matrix<double, DIM, 1> vec = x_prop - x_curr;
      Eigen::Matrix<double, DIM, 1> solution = K_1.block<DIM, 6>(0, 0) * HTz + vec - G.block<DIM, 6>(0, 0) * vec.block<6, 1>(0, 0);

      x_curr += solution;
      Eigen::Vector3d rot_add = solution.block<3, 1>(0, 0);
      Eigen::Vector3d tra_add = solution.block<3, 1>(3, 0);

      EKF_stop_flg = false;
      flg_EKF_converged = false;

      if ((rot_add.norm() * 57.3 < 0.01) && (tra_add.norm() * 100 < 0.015)) 
        flg_EKF_converged = true;

      if(flg_EKF_converged || ((rematch_num==0) && (iterCount==num_max_iter-2)))
      {       
        rematch_num++;
      }

      if(rematch_num >= 2 || (iterCount == num_max_iter-1))
      {
        x_curr.cov = (I_STATE - G) * x_curr.cov;
        EKF_stop_flg = true;
      }

      if(EKF_stop_flg) break;
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(nnt);
    Eigen::Vector3d evalue = saes.eigenvalues();
    // printf("eva %d: %lf\n", match_num, evalue[0]);

    if(evalue[0] < 14)
      return false;
    else
      return true;
  }

  // 基于KD树的点到平面对齐（用于初始化阶段）
  // 参数：pptr - 输入点云
  pcl::PointCloud<PointType>::Ptr pl_tree;
  void lio_state_estimation_kdtree(PVecPtr pptr)
  {
    static pcl::KdTreeFLANN<PointType> kd_map;
    // 如果地图点数不足，直接添加点到地图
    if(pl_tree->size() < 100)
    {
      for(pointVar pv: *pptr)
      {
        PointType pp;
        pv.pnt = x_curr.R * pv.pnt + x_curr.p;  // 转换到世界坐标系
        pp.x = pv.pnt[0]; pp.y = pv.pnt[1]; pp.z = pv.pnt[2];
        pl_tree->push_back(pp);
      }
      kd_map.setInputCloud(pl_tree);
      return;
    }

    const int num_max_iter = 4;
    IMUST x_prop = x_curr;
    int psize = pptr->size();
    bool EKF_stop_flg = 0, flg_EKF_converged = 0;
    Eigen::Matrix<double, DIM, DIM> G, H_T_H, I_STATE;
    G.setZero(); H_T_H.setZero(); I_STATE.setIdentity();

    double max_dis = 2*2;
    vector<float> sqdis(NMATCH); vector<int> nearInd(NMATCH);
    PLV(3) vecs(NMATCH);
    int rematch_num = 0;
    Eigen::Matrix<double, DIM, DIM> cov_inv = x_curr.cov.inverse();

    Eigen::Matrix<double, NMATCH, 1> b;
    b.setOnes();
    b *= -1.0f;

    vector<double> ds(psize, -1);
    PLV(3) directs(psize);
    bool refind = true;

    for(int iterCount=0; iterCount<num_max_iter; iterCount++)
    {
      Eigen::Matrix<double, 6, 6> HTH; HTH.setZero();
      Eigen::Matrix<double, 6, 1> HTz; HTz.setZero();
      int valid = 0;
      for(int i=0; i<psize; i++)
      {
        pointVar &pv = pptr->at(i);
        Eigen::Matrix3d phat = hat(pv.pnt);
        Eigen::Vector3d wld = x_curr.R * pv.pnt + x_curr.p;

        if(refind)
        {
          PointType apx;
          apx.x = wld[0]; apx.y = wld[1]; apx.z = wld[2];
          kd_map.nearestKSearch(apx, NMATCH, nearInd, sqdis);

          Eigen::Matrix<double, NMATCH, 3> A;
          for(int i=0; i<NMATCH; i++)
          {
            PointType &pp = pl_tree->points[nearInd[i]];
            A.row(i) << pp.x, pp.y, pp.z;
          }
          Eigen::Vector3d direct = A.colPivHouseholderQr().solve(b);
          bool check_flag = false;
          for(int i=0; i<NMATCH; i++)
          {
            if(fabs(direct.dot(A.row(i)) + 1.0) > 0.1) 
              check_flag = true;
          }

          if(check_flag) 
          {
            ds[i] = -1;
            continue;
          }
          
          double d = 1.0 / direct.norm();
          // direct *= d;
          ds[i] = d;
          directs[i] = direct * d;
        }

        if(ds[i] >= 0)
        {
          double pd2 = directs[i].dot(wld) + ds[i];
          Eigen::Matrix<double, 6, 1> jac_s;
          jac_s.head(3) = phat * x_curr.R.transpose() * directs[i];
          jac_s.tail(3) = directs[i];

          HTH += jac_s * jac_s.transpose();
          HTz += jac_s * (-pd2);
          valid++;
        }
      }

      H_T_H.block<6, 6>(0, 0) = HTH;
      Eigen::Matrix<double, DIM, DIM> K_1 = (H_T_H + cov_inv / 1000).inverse();
      G.block<DIM, 6>(0, 0) = K_1.block<DIM, 6>(0, 0) * HTH;
      Eigen::Matrix<double, DIM, 1> vec = x_prop - x_curr;
      Eigen::Matrix<double, DIM, 1> solution = K_1.block<DIM, 6>(0, 0) * HTz + vec - G.block<DIM, 6>(0, 0) * vec.block<6, 1>(0, 0);

      x_curr += solution;
      Eigen::Vector3d rot_add = solution.block<3, 1>(0, 0);
      Eigen::Vector3d tra_add = solution.block<3, 1>(3, 0);

      refind = false;
      if ((rot_add.norm() * 57.3 < 0.01) && (tra_add.norm() * 100 < 0.015))
      {
        refind = true;
        flg_EKF_converged = true;
        rematch_num++;
      }

      if(iterCount == num_max_iter-2 && !flg_EKF_converged)
      {
        refind = true;
      }

      if(rematch_num >= 2 || (iterCount == num_max_iter-1))
      {
        x_curr.cov = (I_STATE - G) * x_curr.cov;
        EKF_stop_flg = true;
      }

      if(EKF_stop_flg) break;
    }

    double tt1 = ros::Time::now().toSec();
    for(pointVar pv: *pptr)
    {
      pv.pnt = x_curr.R * pv.pnt + x_curr.p;
      PointType ap;
      ap.x = pv.pnt[0]; ap.y = pv.pnt[1]; ap.z = pv.pnt[2];
      pl_tree->push_back(ap);
    }
    down_sampling_voxel(*pl_tree, 0.5);
    kd_map.setInputCloud(pl_tree);
    double tt2 = ros::Time::now().toSec();
  }

  // 回环更新：检测到回环后，更新当前地图和状态
  // 将回环检测线程计算的修正应用到主线程的状态和地图
  void loop_update()
  {
    printf("loop update: %zu\n", sws[0].size());
    double t1 = ros::Time::now().toSec();
    // 清除旧的地图，准备用回环后的地图替换
    for(auto iter=surf_map.begin(); iter!=surf_map.end(); iter++)
    {
      // octos_release.push_back(iter->second);
      iter->second->tras_ptr(octos_release);  // 将八叉树节点加入待释放列表
      iter->second->clear_slwd(sws[0]);       // 清除滑动窗口
      delete iter->second; iter->second = nullptr;
    }
    surf_map.clear(); surf_map_slide.clear();
    surf_map = map_loop;  // 用回环优化后的地图替换
    map_loop.clear();

    printf("scanPoses: %zu %zu %zu %d %d %zu\n", scanPoses->size(), buf_lba2loop.size(), x_buf.size(), win_base, win_count, sws[0].size());
    int blsize = scanPoses->size();
    PointType ap = pcl_path[0];
    pcl_path.clear();
    
    for(int i=0; i<blsize; i++)
    {
      ap.x = scanPoses->at(i)->x.p[0];
      ap.y = scanPoses->at(i)->x.p[1];
      ap.z = scanPoses->at(i)->x.p[2];
      pcl_path.push_back(ap);
    }

    for(ScanPose *bl: buf_lba2loop)
    {
      bl->update(dx);
      ap.x = bl->x.p[0];
      ap.y = bl->x.p[1];
      ap.z = bl->x.p[2];
      pcl_path.push_back(ap);
    }
    
    for(int i=0; i<win_count; i++)
    {
      IMUST &x = x_buf[i];
      x.v = dx.R * x.v;
      x.p = dx.R * x.p + dx.p;
      x.R = dx.R * x.R;
      if(g_update == 1)
        x.g = dx.R * x.g;
      // PointType ap;
      ap.x = x.p[0]; ap.y = x.p[1]; ap.z = x.p[2];
      pcl_path.push_back(ap);
    }

    pub_pl_func(pcl_path, pub_curr_path);

    x_curr.R = x_buf[win_count-1].R;
    x_curr.p = x_buf[win_count-1].p;
    x_curr.v = dx.R * x_curr.v;
    x_curr.g = x_buf[win_count-1].g;
    
    for(int i=0; i<win_size; i++)
      mp[i] = i;

    for(ScanPose *bl: buf_lba2loop)
    {
      IMUST xx = bl->x;
      PVec pvec_tem = *(bl->pvec);
      for(pointVar &pv: pvec_tem)
        pv.pnt = xx.R * pv.pnt + xx.p;
      cut_voxel(surf_map, pvec_tem, win_size, 0);
    }
    
    PLV(3) pwld;
    for(int i=0; i<win_count; i++)
    {
      pwld.clear();
      for(pointVar &pv: *pvec_buf[i])
        pwld.push_back(x_buf[i].R * pv.pnt + x_buf[i].p);
      cut_voxel(surf_map, pvec_buf[i], i, surf_map_slide, win_size, pwld, sws[0]);
    }

    for(auto iter=surf_map.begin(); iter!=surf_map.end(); ++iter)
      iter->second->recut(win_count, x_buf, sws[0]);

    if(g_update == 1) g_update = 2;
    loop_detect = 0;
    double t2 = ros::Time::now().toSec();
    printf("loop head: %lf %zu\n", t2 - t1, sws[0].size());
  }

  // 加载历史关键帧到局部体素地图
  // 将附近的历史关键帧加载到当前地图中，提高匹配质量
  // 参数：jour - 当前行程距离
  void keyframe_loading(double jour)
  {
    if(history_kfsize <= 0) return;  // 没有历史关键帧则返回
    double tt1 = ros::Time::now().toSec();
    PointType ap_curr;
    ap_curr.x = x_curr.p[0];
    ap_curr.y = x_curr.p[1];
    ap_curr.z = x_curr.p[2];
    vector<int> vec_idx;
    vector<float> vec_dis;
    // 搜索半径10米内的历史关键帧
    kd_keyframes.radiusSearch(ap_curr, 10, vec_idx, vec_dis);

    for(int id: vec_idx)
    {
      int ord_kf = pl_kdmap->points[id].curvature;
      if(keyframes->at(id)->exist)
      {
        Keyframe &kf = *(keyframes->at(id));
        IMUST &xx = kf.x0;
        PVec pvec; pvec.reserve(kf.plptr->size());

        pointVar pv; pv.var.setZero();
        int plsize = kf.plptr->size();
        // for(int j=0; j<plsize; j+=2)
        for(int j=0; j<plsize; j++)
        {
          PointType ap = kf.plptr->points[j];
          pv.pnt << ap.x, ap.y, ap.z;
          pv.pnt = xx.R * pv.pnt + xx.p;
          pvec.push_back(pv);
        }

        cut_voxel(surf_map, pvec, win_size, jour);
        kf.exist = 0;
        history_kfsize--;
        break;
      }
    }
    
  }

  // 初始化流程：累积多帧数据后进行运动初始化
  // 参数：imus - IMU数据
  //      hess - Hessian矩阵
  //      voxhess - 体素因子
  //      pwld - 世界坐标系点云（输出）
  //      pcl_curr - 当前点云
  // 返回：0-继续累积，1-初始化成功，-1-初始化失败需要重置
  int initialization(deque<sensor_msgs::Imu::Ptr> &imus, Eigen::MatrixXd &hess, LidarFactor &voxhess, PLV(3) &pwld, pcl::PointCloud<PointType>::Ptr pcl_curr)
  {
    static vector<pcl::PointCloud<PointType>::Ptr> pl_origs;  // 原始点云缓存
    static vector<double> beg_times;  // 起始时间缓存
    static vector<deque<sensor_msgs::Imu::Ptr>> vec_imus;  // IMU数据缓存

    pcl::PointCloud<PointType>::Ptr orig(new pcl::PointCloud<PointType>(*pcl_curr));
    // 使用EKF处理当前帧
    if(odom_ekf.process(x_curr, *pcl_curr, imus) == 0)
      return 0;

    if(win_count == 0)
      imupre_scale_gravity = odom_ekf.scale_gravity;

    PVecPtr pptr(new PVec);
    double downkd = down_size >= 0.5 ? down_size : 0.5;
    down_sampling_voxel(*pcl_curr, downkd);
    var_init(extrin_para, *pcl_curr, pptr, dept_err, beam_err);
    lio_state_estimation_kdtree(pptr);

    pwld.clear();
    pvec_update(pptr, x_curr, pwld);

    win_count++;
    x_buf.push_back(x_curr);
    pvec_buf.push_back(pptr);
    ResultOutput::instance().pub_localtraj(pwld, 0, x_curr, sessionNames.size()-1, pcl_path);

    if(win_count > 1)
    {
      imu_pre_buf.push_back(new IMU_PRE(x_buf[win_count-2].bg, x_buf[win_count-2].ba));
      imu_pre_buf[win_count-2]->push_imu(imus);
    }

    pcl::PointCloud<PointType> pl_mid = *orig;
    down_sampling_close(*orig, down_size);
    if(orig->size() < 1000)
    {
      *orig = pl_mid;
      down_sampling_close(*orig, down_size / 2);
    }

    sort(orig->begin(), orig->end(), [](PointType &x, PointType &y)
    {return x.curvature < y.curvature;});

    pl_origs.push_back(orig);
    beg_times.push_back(odom_ekf.pcl_beg_time);
    vec_imus.push_back(imus);

    int is_success = 0;
    if(win_count >= win_size)
    {
      is_success = Initialization::instance().motion_init(pl_origs, vec_imus, beg_times, &hess, voxhess, x_buf, surf_map, surf_map_slide, pvec_buf, win_size, sws, x_curr, imu_pre_buf, extrin_para);

      if(is_success == 0)
        return -1;
      return 1;
    }
    return 0;
  }

  // 系统重置：在初始化失败或严重退化时重置系统状态
  // 参数：imus - 当前IMU数据（用于重新初始化）
  void system_reset(deque<sensor_msgs::Imu::Ptr> &imus)
  {
    // 清除所有地图数据
    for(auto iter=surf_map.begin(); iter!=surf_map.end(); iter++)
    {
      iter->second->tras_ptr(octos_release);
      iter->second->clear_slwd(sws[0]);
      delete iter->second;
    }
    surf_map.clear(); surf_map_slide.clear();

    // 重置状态
    x_curr.setZero();
    x_curr.p = Eigen::Vector3d(0, 0, 30);  // 初始高度30米
    odom_ekf.mean_acc.setZero();
    odom_ekf.init_num = 0;
    odom_ekf.IMU_init(imus);  // 重新初始化IMU
    x_curr.g = -odom_ekf.mean_acc * imupre_scale_gravity;  // 重新估计重力

    for(int i=0; i<imu_pre_buf.size(); i++)
      delete imu_pre_buf[i];
    x_buf.clear(); pvec_buf.clear(); imu_pre_buf.clear();
    pl_tree->clear();

    for(int i=0; i<win_size; i++)
      mp[i] = i;
    win_base = 0; win_count = 0; pcl_path.clear();
    pub_pl_func(pcl_path, pub_cmap);
    ROS_WARN("Reset");
  }

  // 局部BA后的边缘化：更新地图并边缘化最老的扫描
  // 使用多线程加速处理
  // 参数：feat_map - 特征地图
  //      jour - 当前行程距离
  //      win_count - 窗口内帧数
  //      xs - 状态序列
  //      voxopt - 体素优化器
  //      sw - 滑动窗口
  void multi_margi(unordered_map<VOXEL_LOC, OctoTree*> &feat_map, double jour, int win_count, vector<IMUST> &xs, LidarFactor &voxopt, vector<SlideWindow*> &sw)
  {
    // for(auto iter=feat_map.begin(); iter!=feat_map.end();)
    // {
    //   iter->second->jour = jour;
    //   iter->second->margi(win_count, 1, xs, voxopt);
    //   if(iter->second->isexist)
    //     iter++;
    //   else
    //   {
    //     iter->second->clear_slwd(sw);
    //     feat_map.erase(iter++);
    //   }
    // }
    // return;

    int thd_num = thread_num;
    vector<vector<OctoTree*>*> octs;
    for(int i=0; i<thd_num; i++) 
      octs.push_back(new vector<OctoTree*>());

    int g_size = feat_map.size();
    if(g_size < thd_num) return;
    vector<thread*> mthreads(thd_num);
    double part = 1.0 * g_size / thd_num;
    int cnt = 0;
    for(auto iter=feat_map.begin(); iter!=feat_map.end(); iter++)
    {
      iter->second->jour = jour;
      octs[cnt]->push_back(iter->second);
      if(octs[cnt]->size() >= part && cnt < thd_num-1)
        cnt++;
    }

    auto margi_func = [](int win_cnt, vector<OctoTree*> *oct, vector<IMUST> xxs, LidarFactor &voxhess)
    {
      for(OctoTree *oc: *oct)
      {
        oc->margi(win_cnt, 1, xxs, voxhess);
      }
    };

    for(int i=1; i<thd_num; i++)
    {
      mthreads[i] = new thread(margi_func, win_count, octs[i], xs, ref(voxopt));
    }
    
    for(int i=0; i<thd_num; i++)
    {
      if(i == 0)
      {
        margi_func(win_count, octs[i], xs, voxopt);
      }
      else
      {
        mthreads[i]->join();
        delete mthreads[i];
      }
    }

    for(auto iter=feat_map.begin(); iter!=feat_map.end();)
    {
      if(iter->second->isexist)
        iter++;
      else
      {
        iter->second->clear_slwd(sw);
        feat_map.erase(iter++);
      }
    }

    for(int i=0; i<thd_num; i++)
      delete octs[i];

  }

  // 确定平面并在八叉树中重新分割体素地图
  // 使用多线程加速平面拟合和特征提取
  // 参数：feat_map - 特征地图
  //      win_count - 窗口内帧数
  //      xs - 状态序列
  //      voxopt - 体素优化器
  //      sws - 滑动窗口（多线程）
  void multi_recut(unordered_map<VOXEL_LOC, OctoTree*> &feat_map, int win_count, vector<IMUST> &xs, LidarFactor &voxopt, vector<vector<SlideWindow*>> &sws)
  {
    // for(auto iter=feat_map.begin(); iter!=feat_map.end(); iter++)
    // {
    //   iter->second->recut(win_count, xs, sws[0]);
    //   iter->second->tras_opt(voxopt);
    // }

    int thd_num = thread_num;
    vector<vector<OctoTree*>> octss(thd_num);
    int g_size = feat_map.size();
    if(g_size < thd_num) return;
    vector<thread*> mthreads(thd_num);
    double part = 1.0 * g_size / thd_num;
    int cnt = 0;
    for(auto iter=feat_map.begin(); iter!=feat_map.end(); iter++)
    {
      octss[cnt].push_back(iter->second);
      if(octss[cnt].size() >= part && cnt < thd_num-1)
        cnt++;
    }

    auto recut_func = [](int win_count, vector<OctoTree*> &oct, vector<IMUST> xxs, vector<SlideWindow*> &sw)
    {
      for(OctoTree *oc: oct)
        oc->recut(win_count, xxs, sw);
    };

    for(int i=1; i<thd_num; i++)
    {
      mthreads[i] = new thread(recut_func, win_count, ref(octss[i]), xs, ref(sws[i]));
    }

    for(int i=0; i<thd_num; i++)
    {
      if(i == 0)
      {
        recut_func(win_count, octss[i], xs, sws[i]);
      }
      else
      {
        mthreads[i]->join();
        delete mthreads[i];
      }
    }

    for(int i=1; i<sws.size(); i++)
    {
      sws[0].insert(sws[0].end(), sws[i].begin(), sws[i].end());
      sws[i].clear();
    }

    for(auto iter=feat_map.begin(); iter!=feat_map.end(); iter++)
      iter->second->tras_opt(voxopt);

  }

  // 里程计和局部建图主线程
  // 负责处理传感器数据、状态估计、局部BA和地图维护
  void thd_odometry_localmapping(ros::NodeHandle &n)
  {
    PLV(3) pwld;  // 世界坐标系点云
    double down_sizes[3] = {0.1, 0.2, 0.4};  // 不同的降采样尺寸
    Eigen::Vector3d last_pos(0, 0 ,0);  // 上次位置
    double jour = 0;  // 行程距离
    int counter = 0;  // 计数器

    pcl::PointCloud<PointType>::Ptr pcl_curr(new pcl::PointCloud<PointType>());
    int motion_init_flag = 1;  // 运动初始化标志
    pl_tree.reset(new pcl::PointCloud<PointType>());  // 用于初始化的KD树点云
    vector<pcl::PointCloud<PointType>::Ptr> pl_origs;  // 原始点云
    vector<double> beg_times;  // 起始时间
    vector<deque<sensor_msgs::Imu::Ptr>> vec_imus;  // IMU数据
    bool release_flag = false;  // 内存释放标志
    int degrade_cnt = 0;  // 退化计数器
    LidarFactor voxhess(win_size);  // 激光因子（用于优化）
    const int mgsize = 1;  // 边缘化帧数
    Eigen::MatrixXd hess;  // Hessian矩阵
    while(n.ok())
    {
      ros::spinOnce();
      if(loop_detect == 1)
      {
        loop_update(); last_pos = x_curr.p; jour = 0;
      }
      
      n.param<bool>("finish", is_finish, false);
      if(is_finish)
      {
        break;
      }

      deque<sensor_msgs::Imu::Ptr> imus;
      if(!sync_packages(pcl_curr, imus, odom_ekf))
      {
        if(octos_release.size() != 0)
        {
          int msize = octos_release.size();
          if(msize > 1000) msize = 1000;
          for(int i=0; i<msize; i++)
          {
            delete octos_release.back();
            octos_release.pop_back();
          }
          malloc_trim(0);
        }
        else if(release_flag)
        {
          release_flag = false;
          vector<OctoTree*> octos;
          for(auto iter=surf_map.begin(); iter!=surf_map.end();)
          {
            int dis = jour - iter->second->jour;
            if(dis < 700)
            // if(dis < 200)
            {
              iter++;
            }
            else
            {
              octos.push_back(iter->second);
              iter->second->tras_ptr(octos);
              surf_map.erase(iter++);
            }
          }
          int ocsize = octos.size();
          for(int i=0; i<ocsize; i++)
            delete octos[i];
          octos.clear();
          malloc_trim(0);
        }
        else if(sws[0].size() > 10000)
        {
          for(int i=0; i<500; i++)
          {
            delete sws[0].back();
            sws[0].pop_back();
          }
          malloc_trim(0);
        }

        sleep(0.001);
        continue;
      }

      static int first_flag = 1;
      if (first_flag)
      {
        pcl::PointCloud<PointType> pl;
        pub_pl_func(pl, pub_pmap);
        pub_pl_func(pl, pub_prev_path);
        first_flag = 0;
      }

      double t0 = ros::Time::now().toSec();
      double t1=0, t2=0, t3=0, t4=0, t5=0, t6=0, t7=0, t8=0;

      if(motion_init_flag)
      {
        int init = initialization(imus, hess, voxhess, pwld, pcl_curr);

        if(init == 1)
        {
          motion_init_flag = 0;
        }
        else
        {
          if(init == -1)
            system_reset(imus);
          continue;
        }
      }
      else
      {
        if(odom_ekf.process(x_curr, *pcl_curr, imus) == 0)
          continue;

        pcl::PointCloud<PointType> pl_down = *pcl_curr;
        down_sampling_voxel(pl_down, down_size);

        if(pl_down.size() < 500)
        {
          pl_down = *pcl_curr;
          down_sampling_voxel(pl_down, down_size / 2);
        }

        PVecPtr pptr(new PVec);
        var_init(extrin_para, pl_down, pptr, dept_err, beam_err);

        if(lio_state_estimation(pptr))
        {
          if(degrade_cnt > 0) degrade_cnt--;
        }
        else
          degrade_cnt++;

        pwld.clear();
        pvec_update(pptr, x_curr, pwld);
        ResultOutput::instance().pub_localtraj(pwld, jour, x_curr, sessionNames.size()-1, pcl_path);

        t1 = ros::Time::now().toSec();

        win_count++;
        x_buf.push_back(x_curr);
        pvec_buf.push_back(pptr);
        if(win_count > 1)
        {
          imu_pre_buf.push_back(new IMU_PRE(x_buf[win_count-2].bg, x_buf[win_count-2].ba));
          imu_pre_buf[win_count-2]->push_imu(imus);
        }
        
        keyframe_loading(jour);
        voxhess.clear(); voxhess.win_size = win_size;

        // cut_voxel(surf_map, pvec_buf[win_count-1], win_count-1, surf_map_slide, win_size, pwld, sws[0]);
        cut_voxel_multi(surf_map, pvec_buf[win_count-1], win_count-1, surf_map_slide, win_size, pwld, sws);
        t2 = ros::Time::now().toSec();

        multi_recut(surf_map_slide, win_count, x_buf, voxhess, sws);
        t3 = ros::Time::now().toSec();

        if(degrade_cnt > degrade_bound)
        {
          degrade_cnt = 0;
          system_reset(imus);

          last_pos = x_curr.p; jour = 0;

          mtx_loop.lock();
          buf_lba2loop_tem.swap(buf_lba2loop);
          mtx_loop.unlock();
          reset_flag = 1;

          motion_init_flag = 1;
          history_kfsize = 0;

          continue;
        }
      }

      if(win_count >= win_size)
      {
        t4 = ros::Time::now().toSec();
        
        if(g_update == 2)
        {
          LI_BA_OptimizerGravity opt_lsv;
          vector<double> resis;
          opt_lsv.damping_iter(x_buf, voxhess, imu_pre_buf, resis, &hess, 5);
          printf("g update: %lf %lf %lf: %lf\n", x_buf[0].g[0], x_buf[0].g[1], x_buf[0].g[2], x_buf[0].g.norm());
          g_update = 0;
          x_curr.g = x_buf[win_count-1].g;
        }
        else
        {
          LI_BA_Optimizer opt_lsv;
          opt_lsv.damping_iter(x_buf, voxhess, imu_pre_buf, &hess);
        }

        ScanPose *bl = new ScanPose(x_buf[0], pvec_buf[0]);
        bl->v6 = hess.block<6, 6>(0, DIM).diagonal();
        for(int i=0; i<6; i++) bl->v6[i] = 1.0 / fabs(bl->v6[i]);
        mtx_loop.lock();
        buf_lba2loop.push_back(bl);
        mtx_loop.unlock();

        x_curr.R = x_buf[win_count-1].R;
        x_curr.p = x_buf[win_count-1].p;
        t5 = ros::Time::now().toSec();

        ResultOutput::instance().pub_localmap(mgsize, sessionNames.size()-1, pvec_buf, x_buf, pcl_path, win_base, win_count);

        multi_margi(surf_map_slide, jour, win_count, x_buf, voxhess, sws[0]);
        t6 = ros::Time::now().toSec();

        if((win_base + win_count) % 10 == 0)
        {
          double spat = (x_curr.p - last_pos).norm();
          if(spat > 0.5)
          {
            jour += spat;
            last_pos = x_curr.p;
            release_flag = true;
          }
        }

        if(is_save_map)
        {
          for(int i=0; i<mgsize; i++)
            FileReaderWriter::instance().save_pcd(pvec_buf[i], x_buf[i], win_base + i, savepath + bagname);
        }

        for(int i=0; i<win_size; i++)
        {
          mp[i] += mgsize;
          if(mp[i] >= win_size) mp[i] -= win_size;
        }

        for(int i=mgsize; i<win_count; i++)
        {
          x_buf[i-mgsize] = x_buf[i];
          PVecPtr pvec_tem = pvec_buf[i-mgsize];
          pvec_buf[i-mgsize] = pvec_buf[i];
          pvec_buf[i] = pvec_tem;
        }

        for(int i=win_count-mgsize; i<win_count; i++)
        {
          x_buf.pop_back();
          pvec_buf.pop_back();

          delete imu_pre_buf.front();
          imu_pre_buf.pop_front();
        }

        win_base += mgsize; win_count -= mgsize;
      }
      
      double t_end = ros::Time::now().toSec();
      double mem = get_memory();
      // printf("%d: %.4lf: %.4lf %.4lf %.4lf %.4lf %.4lf %.2lfGb %.1lf\n", win_base+win_count, t_end-t0, t1-t0, t2-t1, t3-t2, t5-t4, t6-t5, mem, jour);

      // printf("%d: %lf %lf %lf\n", win_base + win_count, x_curr.p[0], x_curr.p[1], x_curr.p[2]);
    }

    vector<OctoTree *> octos;
    for(auto iter=surf_map.begin(); iter!=surf_map.end(); iter++)
    {
      iter->second->tras_ptr(octos);
      iter->second->clear_slwd(sws[0]);
      delete iter->second;
    }

    for(int i=0; i<octos.size(); i++)
      delete octos[i];
    octos.clear();

    for(int i=0; i<sws[0].size(); i++)
      delete sws[0][i];
    sws[0].clear();
    malloc_trim(0);
  }

  // 构建回环闭合的位姿图
  // 参数：initial - 初始值（输出）
  //      graph - 因子图（输出）
  //      cur_id - 当前会话ID
  //      lp_edges - 回环边
  //      default_noise - 默认噪声模型
  //      ids - 连接的会话ID列表（输出）
  //      stepsizes - 步长列表（输出）
  //      lpedge_enable - 是否启用回环边（1启用，0禁用）
  void build_graph(gtsam::Values &initial, gtsam::NonlinearFactorGraph &graph, int cur_id, PGO_Edges &lp_edges, gtsam::noiseModel::Diagonal::shared_ptr default_noise, vector<int> &ids, vector<int> &stepsizes, int lpedge_enable)
  {
    initial.clear(); graph = gtsam::NonlinearFactorGraph();
    ids.clear();
    lp_edges.connect(cur_id, ids);  // 查找与当前会话连接的所有会话

    stepsizes.clear(); stepsizes.push_back(0);
    for(int i=0; i<ids.size(); i++)
      stepsizes.push_back(stepsizes.back() + multimap_scanPoses[ids[i]]->size());
    
    for(int ii=0; ii<ids.size(); ii++)
    {
      int bsize = stepsizes[ii], id = ids[ii];
      for(int j=bsize; j<stepsizes[ii+1]; j++)
      {
        IMUST &xc = multimap_scanPoses[id]->at(j-bsize)->x;
        gtsam::Pose3 pose3(gtsam::Rot3(xc.R), gtsam::Point3(xc.p));
        initial.insert(j, pose3);
        if(j > bsize)
        {
          gtsam::Vector samv6(6);
          samv6 = multimap_scanPoses[ids[ii]]->at(j-1-bsize)->v6;
          gtsam::noiseModel::Diagonal::shared_ptr v6_noise = gtsam::noiseModel::Diagonal::Variances(samv6);
          add_edge(j-1, j, multimap_scanPoses[id]->at(j-1-bsize)->x, multimap_scanPoses[id]->at(j-bsize)->x, graph, v6_noise);
          // add_edge(j-1, j, multimap_scanPoses[id]->at(j-1-bsize)->x, multimap_scanPoses[id]->at(j-bsize)->x, graph, default_noise);
        }
      }
    }

    if(multimap_scanPoses[ids[0]]->size() != 0)
    {
      int ceil = multimap_scanPoses[ids[0]]->size();
      // if(ceil > 10) ceil = 10;
      ceil = 1;
      for(int i=0; i<ceil; i++)
      {
        Eigen::Matrix<double, 6, 1> v6_fixd;
        v6_fixd << 1e-9, 1e-9, 1e-9, 1e-9, 1e-9, 1e-9;
        gtsam::noiseModel::Diagonal::shared_ptr fixd_noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector(v6_fixd));
        IMUST xf = multimap_scanPoses[ids[0]]->at(i)->x;
        gtsam::Pose3 pose3 = gtsam::Pose3(gtsam::Rot3(xf.R), gtsam::Point3(xf.p));
        graph.addPrior(i, pose3, fixd_noise);
      }
    }

    if(lpedge_enable == 1)
    for(PGO_Edge &edge: lp_edges.edges)
    {
      vector<int> step(2);
      if(edge.is_adapt(ids, step))
      {
        int mp[2] = {stepsizes[step[0]], stepsizes[step[1]]};
        for(int i=0; i<edge.rots.size(); i++)
        {
          int id1 = mp[0] + edge.ids1[i];
          int id2 = mp[1] + edge.ids2[i];
          add_edge(id1, id2, edge.rots[i], edge.tras[i], graph, default_noise);
        }
      }
    }
    
  }

  // 回环闭合主线程
  // 负责回环检测、位姿图优化和HBA的自顶向下过程
  void thd_loop_closure(ros::NodeHandle &n)
  {
    pl_kdmap.reset(new pcl::PointCloud<PointType>);
    vector<STDescManager*> std_managers;  // 场景描述子管理器（多会话）
    PGO_Edges lp_edges;  // 回环边

    // 回环检测参数
    double jud_default = 0.45, icp_eigval = 14;  // 默认判断阈值和ICP特征值阈值
    double ratio_drift = 0.05;  // 漂移比例阈值
    int curr_halt = 10, prev_halt = 30;  // 当前/历史回环检测间隔
    int isHighFly = 0;  // 是否高空飞行
    n.param<double>("Loop/jud_default", jud_default, 0.45);
    n.param<double>("Loop/icp_eigval", icp_eigval, 14);
    n.param<double>("Loop/ratio_drift", ratio_drift, 0.05);
    n.param<int>("Loop/curr_halt", curr_halt, 10);
    n.param<int>("Loop/prev_halt", prev_halt, 30);
    n.param<int>("Loop/isHighFly", isHighFly, 0);
    ConfigSetting config_setting;
    read_parameters(n, config_setting, isHighFly);

    vector<double> juds;
    FileReaderWriter::instance().previous_map_names(n, sessionNames, juds);
    FileReaderWriter::instance().pgo_edges_io(lp_edges, sessionNames, 0, savepath, bagname);
    FileReaderWriter::instance().previous_map_read(std_managers, multimap_scanPoses, multimap_keyframes, config_setting, lp_edges, n, sessionNames, juds, savepath, win_size);
    
    STDescManager *std_manager = new STDescManager(config_setting);
    sessionNames.push_back(bagname);
    std_managers.push_back(std_manager);
    multimap_scanPoses.push_back(scanPoses);
    multimap_keyframes.push_back(keyframes);
    juds.push_back(jud_default);
    vector<double> jours(std_managers.size(), 0);

    vector<int> relc_counts(std_managers.size(), prev_halt);
    
    deque<ScanPose*> bl_local;
    Eigen::Matrix<double, 6, 1> v6_init, v6_fixd;
    v6_init << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
    v6_fixd << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6;
    gtsam::noiseModel::Diagonal::shared_ptr odom_noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector(v6_init));
    gtsam::noiseModel::Diagonal::shared_ptr fixd_noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector(v6_fixd));
    gtsam::Values initial;
    gtsam::NonlinearFactorGraph graph;

    vector<int> ids(1, std_managers.size() - 1), stepsizes(2, 0);
    pcl::PointCloud<pcl::PointXYZI>::Ptr plbtc(new pcl::PointCloud<pcl::PointXYZI>);
    IMUST x_key;
    int buf_base = 0;

    while(n.ok())
    {
      if(reset_flag == 1)
      {
        reset_flag = 0;
        scanPoses->insert(scanPoses->end(), buf_lba2loop_tem.begin(), buf_lba2loop_tem.end());
        for(ScanPose *bl: buf_lba2loop_tem) bl->pvec = nullptr;
        buf_lba2loop_tem.clear();

        keyframes = new vector<Keyframe*>();
        multimap_keyframes.push_back(keyframes);
        scanPoses = new vector<ScanPose*>();
        multimap_scanPoses.push_back(scanPoses);

        bl_local.clear(); buf_base = 0; 
        std_manager->config_setting_.skip_near_num_ = -(std_manager->plane_cloud_vec_.size()+10);
        std_manager = new STDescManager(config_setting);
        std_managers.push_back(std_manager);
        relc_counts.push_back(prev_halt);
        sessionNames.push_back(bagname + to_string(sessionNames.size()));
        juds.push_back(jud_default);
        jours.push_back(0);

        bagname = sessionNames.back();
        string cmd = "mkdir " + savepath + bagname + "/";
        int ss = system(cmd.c_str());

        ResultOutput::instance().pub_global_path(multimap_scanPoses, pub_prev_path, ids);
        ResultOutput::instance().pub_globalmap(multimap_keyframes, ids, pub_pmap);

        initial.clear(); graph = gtsam::NonlinearFactorGraph();
        ids.clear(); ids.push_back(std_managers.size()-1); 
        stepsizes.clear(); stepsizes.push_back(0); stepsizes.push_back(0);
      }

      if(is_finish && buf_lba2loop.empty())
      {
        break;
      }

      if(buf_lba2loop.empty() || loop_detect == 1)
      {
        sleep(0.01); continue;
      }
      ScanPose *bl_head = nullptr;
      mtx_loop.lock();
      if(!buf_lba2loop.empty()) 
      {
        bl_head = buf_lba2loop.front();
        buf_lba2loop.pop_front();
      }
      mtx_loop.unlock();
      if(bl_head == nullptr) continue;

      int cur_id = std_managers.size() - 1;
      scanPoses->push_back(bl_head);
      bl_local.push_back(bl_head);
      IMUST xc = bl_head->x;
      gtsam::Pose3 pose3(gtsam::Rot3(xc.R), gtsam::Point3(xc.p));
      int g_pos = stepsizes.back();
      initial.insert(g_pos, pose3);

      if(g_pos > 0)
      {
        gtsam::Vector samv6(scanPoses->at(buf_base-1)->v6);
        gtsam::noiseModel::Diagonal::shared_ptr v6_noise = gtsam::noiseModel::Diagonal::Variances(samv6);
        add_edge(g_pos-1, g_pos, scanPoses->at(buf_base-1)->x, xc, graph, v6_noise);
      }
      else
      {
        gtsam::Pose3 pose3(gtsam::Rot3(xc.R), gtsam::Point3(xc.p));
        graph.addPrior(0, pose3, fixd_noise);
      }

      if(buf_base == 0) x_key = xc;
      buf_base++; stepsizes.back() += 1;

      if(bl_local.size() < win_size) continue;
      double ang = Log(x_key.R.transpose() * xc.R).norm() * 57.3;
      double len = (xc.p - x_key.p).norm();
      if(ang < 5 && len < 0.1 && buf_base > win_size)
      {
        bl_local.front()->pvec = nullptr;
        bl_local.pop_front();
        continue;
      }
      for(double &jour: jours)
        jour += len;
      x_key = xc;

      PVecPtr pptr(new PVec);
      for(int i=0; i<win_size; i++)
      {
        ScanPose &bl = *bl_local[i];
        Eigen::Vector3d delta_p = xc.R.transpose() * (bl.x.p - xc.p);
        Eigen::Matrix3d delta_R = xc.R.transpose() *  bl.x.R;
        for(pointVar pv: *(bl.pvec))
        {
          pv.pnt = delta_R * pv.pnt + delta_p;
          pptr->push_back(pv);
        }
      }
      for(int i=0; i<win_size; i++)
      {
        bl_local.front()->pvec = nullptr;
        bl_local.pop_front();
      }

      Keyframe *smp = new Keyframe(xc);
      smp->id = buf_base - 1;
      smp->jour = jours[cur_id];
      down_sampling_pvec(*pptr, voxel_size/10, *(smp->plptr));

      plbtc->clear();
      pcl::PointXYZI ap;
      for(pointVar &pv: *pptr)
      {
        Eigen::Vector3d &wld = pv.pnt;
        ap.x = wld[0]; ap.y = wld[1]; ap.z = wld[2];
        plbtc->push_back(ap);
      }
      mtx_keyframe.lock();
      keyframes->push_back(smp);
      mtx_keyframe.unlock();

      vector<STD> stds_vec;
      std_manager->GenerateSTDescs(plbtc, stds_vec, buf_base-1);
      pair<int, double> search_result(-1, 0);
      pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform;
      vector<pair<STD, STD>> loop_std_pair;

      bool isGraph = false, isOpt = false;
      int match_num = 0;
      for(int id=0; id<=cur_id; id++)
      {
        std_managers[id]->SearchLoop(stds_vec, search_result, loop_transform, loop_std_pair, std_manager->plane_cloud_vec_.back());

        if(search_result.first >= 0)
        {
          printf("Find Loop in session%d: %d %d\n", id, buf_base, search_result.first);
          printf("score: %lf\n", search_result.second);
        }

        if(search_result.first >= 0 && search_result.second > juds[id])
        {
          if(icp_normal(*(std_manager->plane_cloud_vec_.back()), *(std_managers[id]->plane_cloud_vec_[search_result.first]), loop_transform, icp_eigval))
          {
            int ord_bl = std_managers[id]->plane_cloud_vec_[search_result.first]->header.seq;

            IMUST &xx = multimap_scanPoses[id]->at(ord_bl)->x;
            double drift_p = (xx.R * loop_transform.first + xx.p - xc.p).norm();

            bool isPush = false;
            int step = -1;
            if(id == cur_id)
            {
              double span = smp->jour - keyframes->at(search_result.first)->jour;
              printf("drift: %lf %lf\n", drift_p, span);

              if(drift_p / span < ratio_drift)
              {
                isPush = true;
                step = stepsizes.size() - 2;

                if(relc_counts[id] > curr_halt && drift_p > 0.10)
                {
                  isOpt = true;
                  for(int &cnt: relc_counts) cnt = 0;
                }
              }
            }
            else
            {
              for(int i=0; i<ids.size(); i++)
                if(id == ids[i]) 
                  step = i;
              
              printf("drift: %lf %lf\n", drift_p, jours[id]);

              if(step == -1)
              {
                isGraph = true;
                isOpt = true;
                relc_counts[id] = 0;
                g_update = 1;
                isPush = true;
                jours[id] = 0;
              }
              else
              {
                if(drift_p / jours[id] < 0.05)
                {
                  jours[id] = 1e-6; // set to 0
                  isPush = true;
                  if(relc_counts[id] > prev_halt && drift_p > 0.25)
                  {
                    isOpt = true;
                    for(int &cnt: relc_counts) cnt = 0;
                  }
                }
              }

            }

            if(isPush)
            {
              match_num++;
              lp_edges.push(id, cur_id, ord_bl, buf_base-1, loop_transform.second, loop_transform.first, v6_init);
              if(step > -1)
              {
                int id1 = stepsizes[step] + ord_bl;
                int id2 = stepsizes.back() - 1;
                add_edge(id1, id2, loop_transform.second, loop_transform.first, graph, odom_noise);
                printf("addedge: (%d %d) (%d %d)\n", id, cur_id, ord_bl, buf_base-1);
              }
            }

            // if(isPush)
            // {
            //   icp_check(*(smp->plptr), *(std_managers[id]->plane_cloud_vec_[search_result.first]), pub_test, pub_init, loop_transform, multimap_scanPoses[id]->at(ord_bl)->x);
            // }

          }
        }
        
      }
      for(int &it: relc_counts) it++;
      std_manager->AddSTDescs(stds_vec);
  
      if(isGraph)
      {
        build_graph(initial, graph, cur_id, lp_edges, odom_noise, ids, stepsizes, 1);
      }

      if(isOpt)
      {
        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        parameters.relinearizeSkip = 1;
        gtsam::ISAM2 isam(parameters);
        isam.update(graph, initial);

        for(int i=0; i<5; i++) isam.update();
        gtsam::Values results = isam.calculateEstimate();
        int resultsize = results.size();
        
        IMUST x1 = scanPoses->at(buf_base-1)->x;
        int idsize = ids.size();

        history_kfsize = 0;
        for(int ii=0; ii<idsize; ii++)
        {
          int tip = ids[ii];
          for(int j=stepsizes[ii]; j<stepsizes[ii+1]; j++)
          {
            int ord = j - stepsizes[ii];
            multimap_scanPoses[tip]->at(ord)->set_state(results.at(j).cast<gtsam::Pose3>());
          }
        }
        mtx_keyframe.lock();
        for(int ii=0; ii<idsize; ii++)
        {
          int tip = ids[ii];
          for(Keyframe *kf: *multimap_keyframes[tip])
            kf->x0 = multimap_scanPoses[tip]->at(kf->id)->x;
        }
        mtx_keyframe.unlock();

        initial.clear();
        for(int i=0; i<resultsize; i++)
          initial.insert(i, results.at(i).cast<gtsam::Pose3>());
        
        IMUST x3 = scanPoses->at(buf_base-1)->x;
        dx.p = x3.p - x3.R * x1.R.transpose() * x1.p;
        dx.R = x3.R * x1.R.transpose();
        x_key = x3;

        PVec pvec_tem;
        int subsize = keyframes->size();
        int init_num = 5;
        for(int i=subsize-init_num; i<subsize; i++)
        {
          if(i < 0) continue;
          Keyframe &sp = *(keyframes->at(i));
          sp.exist = 0;
          pvec_tem.reserve(sp.plptr->size());
          pointVar pv; pv.var.setZero();
          for(PointType &ap: sp.plptr->points)
          {
            pv.pnt << ap.x, ap.y, ap.z;
            pv.pnt = sp.x0.R * pv.pnt + sp.x0.p;
            for(int j=0; j<3; j++)
              pv.var(j, j) = ap.normal[j];
            pvec_tem.push_back(pv);
          }
          cut_voxel(map_loop, pvec_tem, win_size, 0);
        }

        if(subsize > init_num)
        {
          pl_kdmap->clear();
          for(int i=0; i<subsize-init_num; i++)
          {
            Keyframe &kf = *(keyframes->at(i));
            kf.exist = 1;
            PointType pp;
            pp.x = kf.x0.p[0]; pp.y = kf.x0.p[1]; pp.z = kf.x0.p[2];
            pp.intensity = cur_id; pp.curvature = i;
            pl_kdmap->push_back(pp);
          }

          kd_keyframes.setInputCloud(pl_kdmap);
          history_kfsize = pl_kdmap->size();
        }
        loop_detect = 1;

        vector<int> ids2 = ids; ids2.pop_back();
        ResultOutput::instance().pub_global_path(multimap_scanPoses, pub_prev_path, ids2);
        ResultOutput::instance().pub_globalmap(multimap_keyframes, ids2, pub_pmap);
        ids2.clear(); ids2.push_back(ids.back());
        ResultOutput::instance().pub_globalmap(multimap_keyframes, ids2, pub_cmap);

      }

    }

    for(int i=0; i<std_managers.size(); i++)
      delete std_managers[i];
    malloc_trim(0);

    if(is_finish)
    {
      if(keyframes->empty())
      {
        sessionNames.pop_back();
        std_managers.pop_back();
        multimap_scanPoses.pop_back();
        multimap_keyframes.pop_back();
        juds.pop_back();
        jours.pop_back();
        relc_counts.pop_back();
      }

      if(multimap_keyframes.empty()) 
      {
        printf("no data\n"); return;
      }

      int cur_id = std_managers.size() - 1;
      build_graph(initial, graph, cur_id, lp_edges, odom_noise, ids, stepsizes, 0);

      topDownProcess(initial, graph, ids, stepsizes);
    }

    if(is_save_map)
    {
      for(int i=0; i<ids.size(); i++)
        FileReaderWriter::instance().save_pose(*(multimap_scanPoses[ids[i]]), sessionNames[ids[i]], "/alidarState.txt", savepath);

      FileReaderWriter::instance().pgo_edges_io(lp_edges, sessionNames, 1, savepath, bagname);
    }

    for(int i=0; i<multimap_scanPoses.size(); i++)
    {
      for(int j=0; j<multimap_scanPoses[i]->size(); j++)
        delete multimap_scanPoses[i]->at(j);
    }
    for(int i=0; i<multimap_keyframes.size(); i++)
    {
      for(int j=0; j<multimap_keyframes[i]->size(); j++)
        delete multimap_keyframes[i]->at(j);
    }
    
    malloc_trim(0);
  }

  // HBA的自顶向下过程：整合自底向上计算的边，执行最终的全局优化
  // 参数：initial - 初始值
  //      graph - 因子图
  //      ids - 会话ID列表
  //      stepsizes - 步长列表
  void topDownProcess(gtsam::Values &initial, gtsam::NonlinearFactorGraph &graph, vector<int> &ids, vector<int> &stepsizes)
  {
    cnct_map = ids;
    gba_size = multimap_keyframes.back()->size();
    gba_flag = 1;  // 通知全局建图线程开始GBA

    pcl::PointCloud<PointType> pl0;
    pub_pl_func(pl0, pub_pmap);
    pub_pl_func(pl0, pub_cmap);
    pub_pl_func(pl0, pub_curr_path);
    pub_pl_func(pl0, pub_prev_path);
    pub_pl_func(pl0, pub_scan);

    double t0 = ros::Time::now().toSec();
    while(gba_flag);
    
    for(PGO_Edge &edge: gba_edges1.edges)
    {
      vector<int> step(2);
      if(edge.is_adapt(ids, step))
      {
        int mp[2] = {stepsizes[step[0]], stepsizes[step[1]]};
        for(int i=0; i<edge.rots.size(); i++)
        {
          int id1 = mp[0] + edge.ids1[i];
          int id2 = mp[1] + edge.ids2[i];
          gtsam::noiseModel::Diagonal::shared_ptr v6_noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector(edge.covs[i]));
          add_edge(id1, id2, edge.rots[i], edge.tras[i], graph, v6_noise);
        }
      }
    }

    for(PGO_Edge &edge: gba_edges2.edges)
    {
      vector<int> step(2);
      if(edge.is_adapt(ids, step))
      {
        int mp[2] = {stepsizes[step[0]], stepsizes[step[1]]};
        for(int i=0; i<edge.rots.size(); i++)
        {
          int id1 = mp[0] + edge.ids1[i];
          int id2 = mp[1] + edge.ids2[i];
          gtsam::noiseModel::Diagonal::shared_ptr v6_noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector(edge.covs[i]));
          add_edge(id1, id2, edge.rots[i], edge.tras[i], graph, v6_noise);
        }
      }
    }

    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    gtsam::ISAM2 isam(parameters);
    isam.update(graph, initial);

    for(int i=0; i<5; i++) isam.update();
    gtsam::Values results = isam.calculateEstimate();
    int resultsize = results.size();

    int idsize = ids.size();
    for(int ii=0; ii<idsize; ii++)
    {
      int tip = ids[ii];
      for(int j=stepsizes[ii]; j<stepsizes[ii+1]; j++)
      {
        int ord = j - stepsizes[ii];
        multimap_scanPoses[tip]->at(ord)->set_state(results.at(j).cast<gtsam::Pose3>());
      }
    }

    Eigen::Quaterniond qq(multimap_scanPoses[0]->at(0)->x.R);

    double t1 = ros::Time::now().toSec();
    printf("GBA opt: %lfs\n", t1 - t0);

    for(int ii=0; ii<idsize; ii++)
    {
      int tip = ids[ii];
      for(Keyframe *smp: *multimap_keyframes[tip])
        smp->x0 = multimap_scanPoses[tip]->at(smp->id)->x;
    }

    ResultOutput::instance().pub_global_path(multimap_scanPoses, pub_prev_path, ids);
    vector<int> ids2 = ids; ids2.pop_back();
    ResultOutput::instance().pub_globalmap(multimap_keyframes, ids2, pub_pmap);
    ids2.clear(); ids2.push_back(ids.back());
    ResultOutput::instance().pub_globalmap(multimap_keyframes, ids2, pub_cmap);
  }

  // HBA的自底向上过程：添加边到位姿图
  // 通过局部BA计算关键帧之间的约束，构建分层的位姿图
  // 参数：p_xs - 位姿序列
  //      p_smps - 子地图序列
  //      gba_edges - 全局BA边（输出）
  //      maps - 地图ID列表
  //      max_iter - 最大迭代次数
  //      thread_num - 线程数
  //      plptr - 可选的点云输出（用于可视化）
  void HBA_add_edge(vector<IMUST> &p_xs, vector<Keyframe*> &p_smps, PGO_Edges &gba_edges, vector<int> &maps, int max_iter, int thread_num, pcl::PointCloud<PointType>::Ptr plptr = nullptr)
  {
    bool is_display = false;
    if(plptr == nullptr) is_display = true;  // 是否显示优化过程

    double t0 = ros::Time::now().toSec();
    vector<Keyframe*> smps;
    vector<IMUST> xs;
    int last_mp = -1, isCnct = 0;
    for(int i=0; i<p_smps.size(); i++)
    {
      Keyframe *smp = p_smps[i];
      if(smp->mp != last_mp)
      {
        isCnct = 0;
        for(int &m: maps)
        if(smp->mp == m)
        {
          isCnct = 1; break;
        }
        last_mp = smp->mp;
      }

      if(isCnct)
      {
        smps.push_back(smp);
        xs.push_back(p_xs[i]);
      }
    }
    
    int wdsize = smps.size();
    Eigen::MatrixXd hess;
    vector<double> gba_eigen_value_array_orig = gba_eigen_value_array;
    double gba_min_eigen_value_orig = gba_min_eigen_value;
    double gba_voxel_size_orig = gba_voxel_size;

    int up = 4;
    int converge_flag = 0;
    double converge_thre = 0.05;

    for(int iterCnt = 0; iterCnt < max_iter; iterCnt++)
    {
      if(converge_flag == 1 || iterCnt == max_iter-1)
      {
        // if(plptr == nullptr)
        // {
        //   break;
        // }

        gba_voxel_size = voxel_size;
        gba_eigen_value_array = plane_eigen_value_thre;
        gba_min_eigen_value = min_eigen_value;
      }

      unordered_map<VOXEL_LOC, OctreeGBA*> oct_map;
      for(int i=0; i<wdsize; i++)
        OctreeGBA::cut_voxel(oct_map, xs[i], smps[i]->plptr, i, wdsize);

      LidarFactor voxhess(wdsize);
      OctreeGBA_multi_recut(oct_map, voxhess, thread_num);

      Lidar_BA_Optimizer opt_lsv;
      opt_lsv.thd_num = thread_num;
      vector<double> resis;
      bool is_converge = opt_lsv.damping_iter(xs, voxhess, &hess, resis, up, is_display);
      if(is_display)
        printf("%lf\n", fabs(resis[0] - resis[1]) / resis[0]);
      if((fabs(resis[0] - resis[1]) / resis[0] < converge_thre && is_converge) || (iterCnt == max_iter-2 && converge_flag == 0))
      {
        converge_thre = 0.01;
        if(converge_flag == 0)
        {
          converge_flag = 1;
        }
        else if(converge_flag == 1)
        {
          break;
        }
      }
    }

    gba_eigen_value_array = gba_eigen_value_array_orig;
    gba_min_eigen_value = gba_min_eigen_value_orig;
    gba_voxel_size = gba_voxel_size_orig;

    for(int i=0; i<wdsize - 1; i++)
    for(int j=i+1; j<wdsize; j++)
    {
      bool isAdd = true;
      Eigen::Matrix<double, 6, 1> v6;
      for(int k=0; k<6; k++)
      {
        double hc = fabs(hess(6*i+k, 6*j+k));
        if(hc < 1e-6) // 1e-6
        {
          isAdd = false; break;
        }
        v6[k] = 1.0 / hc;
      }

      if(isAdd)
      {
        Keyframe &s1 = *smps[i]; Keyframe &s2 = *smps[j];
        Eigen::Vector3d tra = xs[i].R.transpose() * (xs[j].p - xs[i].p);
        Eigen::Matrix3d rot = xs[i].R.transpose() *  xs[j].R;
        gba_edges.push(s1.mp, s2.mp, s1.id, s2.id, rot, tra, v6);
      }
    }

    if(plptr != nullptr)
    {
      pcl::PointCloud<PointType> pl;
      IMUST xc = xs[0];
      for(int i=0; i<wdsize; i++)
      {
        Eigen::Vector3d dp = xc.R.transpose() * (xs[i].p - xc.p);
        Eigen::Matrix3d dR = xc.R.transpose() *  xs[i].R;
        for(PointType ap: smps[i]->plptr->points)
        {
          Eigen::Vector3d v3(ap.x, ap.y, ap.z);
          v3 = dR * v3 + dp;
          ap.x = v3[0]; ap.y = v3[1]; ap.z = v3[2];
          ap.intensity = smps[i]->mp;
          pl.push_back(ap);
        }
      }
      
      down_sampling_voxel(pl, voxel_size / 8);
      plptr->clear(); plptr->reserve(pl.size());
      for(PointType &ap: pl.points)
        plptr->push_back(ap);
    }
    else
    {
      // pcl::PointCloud<PointType> pl, path;
      // pub_pl_func(pl, pub_test);
      // for(int i=0; i<wdsize; i++)
      // {
      //   PointType pt;
      //   pt.x = xs[i].p[0]; pt.y = xs[i].p[1]; pt.z = xs[i].p[2];
      //   path.push_back(pt);
      //   for(int j=1; j<smps[i]->plptr->size(); j+=2)
      //   {
      //     PointType ap = smps[i]->plptr->points[j];
      //     Eigen::Vector3d v3(ap.x, ap.y, ap.z);
      //     v3 = xs[i].R * v3 + xs[i].p;
      //     ap.x = v3[0]; ap.y = v3[1]; ap.z = v3[2];
      //     ap.intensity = smps[i]->mp;
      //     pl.push_back(ap);

      //     if(pl.size() > 1e7)
      //     {
      //       pub_pl_func(pl, pub_test);
      //       pl.clear();
      //       sleep(0.05);
      //     }
      //   }
      // }
      // pub_pl_func(pl, pub_test);
      // return;
    }

  }

  // 全局建图主线程（自底向上）
  // 负责分层BA（HBA）的自底向上过程，构建全局一致的地图
  void thd_globalmapping(ros::NodeHandle &n)
  {
    // 全局BA参数
    n.param<double>("GBA/voxel_size", gba_voxel_size, 1.0);
    n.param<double>("GBA/min_eigen_value", gba_min_eigen_value, 0.01);
    n.param<vector<double>>("GBA/eigen_value_array", gba_eigen_value_array, vector<double>());
    for(double &iter: gba_eigen_value_array) iter = 1.0 / iter;
    int total_max_iter = 1;  // 最大迭代次数
    n.param<int>("GBA/total_max_iter", total_max_iter, 1);

    vector<Keyframe*> gba_submaps;
    deque<int> localID;

    int smp_mp = 0;
    int buf_base = 0;
    int wdsize = 10;
    int mgsize = 5;
    int thread_num = 5;

    while(n.ok())
    {
      if(multimap_keyframes.empty())
      {
        sleep(0.1); continue;
      }

      int smp_flag = 0;
      if(smp_mp+1 < multimap_keyframes.size() && !multimap_keyframes.back()->empty())
        smp_flag = 1;

      vector<Keyframe*> &smps = *multimap_keyframes[smp_mp];
      int total_ba = 0;
      if(gba_flag == 1 && smp_mp >= cnct_map.back() && gba_size <= buf_base)
      {
        printf("gba_flag enter: %d\n", gba_flag);
        total_ba = 1;
      }
      else if(smps.size() <= buf_base)
      {
        if(smp_flag == 0)
        {
          sleep(0.1); continue;
        }
      }
      else
      {
        smps[buf_base]->mp = smp_mp;
        localID.push_back(buf_base);

        buf_base++;
        if(localID.size() < wdsize)
        {
          sleep(0.1); continue;
        }
      }

      vector<IMUST> xs;
      vector<Keyframe*> smp_local;
      mtx_keyframe.lock();
      for(int i: localID)
      {
        xs.push_back(multimap_keyframes[smp_mp]->at(i)->x0);
        smp_local.push_back(multimap_keyframes[smp_mp]->at(i));
      }
      mtx_keyframe.unlock();

      double tg1 = ros::Time::now().toSec();

      Keyframe *gba_smp = new Keyframe(smp_local[0]->x0);
      vector<int> mps{smp_mp};
      HBA_add_edge(xs, smp_local, gba_edges1, mps, 1, 2, gba_smp->plptr);
      gba_smp->id = smp_local[0]->id;
      gba_smp->mp = smp_mp;
      gba_submaps.push_back(gba_smp);

      if(total_ba == 1)
      {
        printf("GBAsize: %d\n", gba_size);
        vector<IMUST> xs;
        mtx_keyframe.lock();
        for(Keyframe *smp: gba_submaps)
        {
          xs.push_back(multimap_scanPoses[smp->mp]->at(smp->id)->x);
        }
        mtx_keyframe.unlock();
        gba_edges2.edges.clear(); gba_edges2.mates.clear();
        HBA_add_edge(xs, gba_submaps, gba_edges2, cnct_map, total_max_iter, thread_num);

        if(is_finish)
        {
          for(int i=0; i<gba_submaps.size(); i++)
            delete gba_submaps[i];
        }
        gba_submaps.clear();

        malloc_trim(0);
        gba_flag = 0;
      }
      else if(smp_flag == 1 && multimap_keyframes[smp_mp]->size() <= buf_base)
      {
        smp_mp++; buf_base = 0; localID.clear();
        // printf("switch: %d\n", smp_mp);
      }
      else
      {
        for(int i=0; i<mgsize; i++)
          localID.pop_front();
      }
  
    }

  }

};

// 主函数：Voxel-SLAM系统入口
// 初始化ROS节点，创建发布器，启动三个主线程（里程计、回环、全局优化）
int main(int argc, char **argv)
{
  ros::init(argc, argv, "cmn_voxel");
  ros::NodeHandle n;

  // 创建各种点云发布器
  pub_cmap = n.advertise<sensor_msgs::PointCloud2>("/map_cmap", 100);      // 当前地图
  pub_pmap = n.advertise<sensor_msgs::PointCloud2>("/map_pmap", 100);      // 历史地图
  pub_scan = n.advertise<sensor_msgs::PointCloud2>("/map_scan", 100);      // 当前扫描
  pub_init = n.advertise<sensor_msgs::PointCloud2>("/map_init", 100);      // 初始化点云
  pub_test = n.advertise<sensor_msgs::PointCloud2>("/map_test", 100);      // 测试点云
  pub_curr_path = n.advertise<sensor_msgs::PointCloud2>("/map_path", 100); // 当前路径
  pub_prev_path = n.advertise<sensor_msgs::PointCloud2>("/map_true", 100); // 历史路径

  // 创建VOXEL_SLAM对象
  VOXEL_SLAM vs(n);
  mp = new int[vs.win_size];
  for(int i=0; i<vs.win_size; i++)
    mp[i] = i;

  // 启动三个主线程
  thread thread_loop(&VOXEL_SLAM::thd_loop_closure, &vs, ref(n));      // 回环检测线程
  thread thread_gba(&VOXEL_SLAM::thd_globalmapping, &vs, ref(n));      // 全局建图线程
  vs.thd_odometry_localmapping(n);  // 里程计和局部建图线程（主线程）

  // 等待线程结束
  thread_loop.join();
  thread_gba.join();
  ros::spin(); return 0;
}

