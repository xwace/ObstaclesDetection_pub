#ifndef _PROCESS_H__
#define _PROCESS_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <termios.h>

#ifdef __cplusplus
}
#endif

#include <map>
#include <atomic>
#include <functional>
#include "muxin/muxin.h"
#include "thread_pool/thread_pool.h"
#include "pretreatment/pretreatment.h"
#include <pcl/visualization/cloud_viewer.h>
#include "base.h"
#include "../common/socket/ClientSokcet.h"
#include "../common/fifo.h"
#include <mutex>
#include "realtime_avoid/realtime_avoid.h"
namespace NS_PROCESS
{
    enum ENUM_OBJECT_CLASS:uint8_t
    {
        ENUM_CLIFF,
        ENUM_SLOP_DOWN,
        ENUM_SLOP_UP,
        ENUM_THRESHOULD,
        ENUM_GENERAL,
        ENUM_PASSTHROUGH,
    };

    struct STR_ATTITUDE     //机器姿态信息
    {
        float x = 0;
        float y = 0;
        float angle = 0;
    };

    struct STR_ATTITUDE_TIEMSTAMP     //用于缓冲使用
    {
        STR_ATTITUDE position;
        int64_t timestamp;
    };

    struct STR_GRIDS_FLAG     //地图索引输出，带属性
    {
        STR_GRIDS_FLAG(){}
        STR_GRIDS_FLAG(int x,int y,int f):x_index(x),y_index(y),flag(f){}
        int x_index = 0;
        int y_index = 0;
        uint8_t flag;
    }__attribute__((packed));

    struct STR_GRIDS_FLAG_COUNT
    {
        STR_GRIDS_FLAG_COUNT(int nums)
        {
            count = new uint32_t [nums];
            count_num = nums;
            memset(count, 0, nums*sizeof(uint32_t));
        }

        STR_GRIDS_FLAG_COUNT(const STR_GRIDS_FLAG_COUNT&a)
        {
            this->y_index = a.y_index;
            this->x_index = a.x_index;
            this->count_num = a.count_num;
            this->count = new uint32_t [a.count_num];
            memcpy(this->count,a.count,a.count_num*sizeof(uint32_t));
        }

        ~STR_GRIDS_FLAG_COUNT()
        {
            if(count != nullptr)
            {
                delete [] count;
            }
        }
        
        bool operator == (const STR_GRIDS_FLAG_COUNT & obj) const
        {
            return x_index == obj.x_index && y_index == obj.y_index;
        }

        int x_index;
        int y_index;
        uint32_t* count;
        int count_num = 0;

    };
    using attitude_cb_type = std::function<void (STR_ATTITUDE&)>;
    class Process
    {
        public:
            void init();
            void process();
            void get_env(NS_REALTIME_AVOID::STR_ENV_TYPE&);
            //注册获取位姿回调
            void set_pose_callback(attitude_cb_type cb);
        private:
            NS_MUXIN::Muxin muxin;
            float x_min = -300;
            float x_max = 300;
            float y_min = -215;
            float y_max = 100;
            float z_min = 100;
            float z_max = 1000;

            NS_THREAD_POOL::Thread_pool thread_pools;
            pcl::PointCloud<pcl::PointXYZ>::Ptr original_points;
            pcl::PointCloud<pcl::PointXYZ>::Ptr pre_points;

            //各种处理模块接口
            NS_PRETREATMENT::Pretreatment pre_treatment;
            NS_REALTIME_AVOID::Realtime_avoid avoider;
            std::shared_ptr<Object_detect> pCliffHandle;
            std::shared_ptr<Object_detect> pSlopDownHandle;
            std::shared_ptr<Object_detect> pPassThroughHandle;
            std::shared_ptr<Object_detect> pObstacleHandle;

            
            std::atomic<bool> system_on;

            //地图映射,坐标转换相关
            void mapping_grid(STR_ATTITUDE);
            void object_ptr_clean();
            void coordinate_transform(pcl::PointCloud<pcl::PointXYZ>::Ptr&,STR_ATTITUDE);
            std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> object_ptr;
            std::vector<STR_GRIDS_FLAG> map_index;          //存放最终输出的索引＋标志
            std::vector<STR_GRIDS_FLAG_COUNT> grids_count;  //点云映射记数
            float y_bias = 35;                              //３d传感器和机器中心相对位置
            float x_bias = 185;

            //map设置等级相关
            using RANGE_OBJECT_MAP = std::map<int,ENUM_OBJECT_CLASS,std::less<int>>;
            using OBJECT_RANGE_MAP = std::map<ENUM_OBJECT_CLASS,int >;
            RANGE_OBJECT_MAP map_range_obj;
            OBJECT_RANGE_MAP map_obj_range;
            bool set_object_range(ENUM_OBJECT_CLASS class_object,int range);
            int range_sort();
            int get_obj_index(ENUM_OBJECT_CLASS);

            //可视化相关
            #ifdef VISUAL
            boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
            #endif

            //获取当前位姿
            attitude_cb_type get_cur_position;

            //对外输出最终地图索引
            void transfer_map_index();
            Client pClientHandle;


            //实时对坐标做1000ms缓存
            NS_FIFO_C::Fifo_c<STR_ATTITUDE_TIEMSTAMP> p_cache {50};
            std::thread cashe_handle;
            void thread_position_cashe();
            int64_t get_systime_cur_ns();
            void coordinate_transform2(pcl::PointCloud<pcl::PointXYZ>::Ptr& input,pcl::PointCloud<pcl::PointXYZ>::Ptr& output,STR_ATTITUDE xy_angle);

            //实时避障接口
            std::mutex r_mutex;
            NS_REALTIME_AVOID::STR_ENV_TYPE real_loca_type;
            void set_realtime_env(NS_REALTIME_AVOID::STR_ENV_TYPE temp);


            //保存ply文件相关
            #ifdef PLY_FILE_ON
            std::vector<point3f_t> ply_points;
            void thread_key_detect();
            std::thread catch_image_handle;
            std::atomic<bool> file_on;
            #endif

    };

}


#endif
