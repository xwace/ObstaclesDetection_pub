#pragma once
#include "../obstacle_detection/obstacle_detection.h"
#include "../cliff_detection/cliff_detection.h"
#include <algorithm> //vector find_if
#define row 20
#define col 60
#define grid_size_x 20
#define grid_size_z 10
/**
 * 算法说明：实时避障只关注悬崖和前方障碍物情况
 * 算法步骤：
 * 1.接入障碍物检测接口/悬崖检测接口，障碍物/悬崖边界点云。
 * 2.映射到栅格，获取对应索引位置。
 * 3.融合上述两个矩阵。
 * 4.逻辑判定避障类型。
*/

namespace NS_REALTIME_AVOID
{
struct obstacle_location
{
        float x;
        float z;
};
//int a;这种多次重定义的话pragma once 也救不了。
enum env_type
{
        //障碍物类型
        ENUM_NOTHING,
        ENUM_BOTH_SIDE,
        ENUM_LEFT_SIDE,
        ENUM_RIGHT_SIDE,
        ENUM_NONE_SIDE,
        //too close
         ENUM_TOO_CLOSE,
};

struct STR_ENV_TYPE
{
        obstacle_location location;
        env_type type;
};

class Realtime_avoid
{
public:
        bool inital();
        bool set_input_imformation(pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud_in_cliff, pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud_in_obstacle);
        void get_env(STR_ENV_TYPE &);

private:
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_cliff;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_obstacle;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_obs;
        vector<vector<double>> vector_index_obstacle;
        vector<vector<double>> vector_index_cliff;
        vector<vector<int>> vector_index_merge;
        vector<vector<double>> vector_gridsize_cliff;
        vector<vector<double>> vector_gridsize_obs;
        NS_OBSTACLE_DETECTION::obstacle_detection realtime_obstacle_detector;
        NS_CLIFF_DECTECTION::Cliff_detection realtime_cliff_detector;
        bool creat_index_obstacle();
        bool creat_index_cliff();
        bool process();
        Grid grid_cliff[row][col];
        Grid grid_obs[row][col];
        env_type ahead;
        obstacle_location obs;
};

} // namespace  NS_REALTIME_AVOID

/*****************************************
 * glider调试记录
 * 1.移植代码主框架,接口不匹配,修改接口信息.//11.26
 * 2.realtime_aviod放在cpp文件,编辑器未报错,编译器报错.//11.27
 * 3.编译通过,core dump,从外到里,一个函数,一段代码的打loge.
 * 4.row和col不匹配.
 * 5.vector定义里未初始化直接循环赋值.//11.28
 * 6.去除30的数据,erase()用一次就失效了,再循环里要特别注意.
 * 7.vector_index_merge数据没clear导致show时候都一样.
 * 8.count_if加的谓词需要注意两点:1.统计的数据类型需为int;2.valude只能取一半条件?
 * 9.if内是==
 * 10.cout << "xxxxxxxxxx1111" << cloud_in_obstacle->points[2].z << endl;打logo按照这种方式打.导致想象:第一次空点云core dump,第一次有点云无影响
*****************************************/