#include "muxin.h"
#include "../../common/time/rtime.h"
using namespace NS_MUXIN;

namespace NS_MUXIN
{
    const char *topic_names[] =
    {
        "stereo_depth_map",
        "mx_version",
        nullptr
    };
}

bool Muxin::init()
{
    /*******************目心相关*****************/
    connect_remote_peers(&s_remote_sub_socket, REMOTE_IP_ADDR);
    import_remote_data(topic_names, s_remote_sub_socket);
    /*******************************************/
    
    //创建NUMS_CASHE个缓冲区
    for(int i(0);i<NUMS_CASHE;i++)
    {
        cashes[i] = new int16_t[RESO_COL*RESO_ROW];
        memset(cashes[i],0,RESO_COL*RESO_ROW*(sizeof(int16_t)));
    }
    system_on = true;

    //初始化点云指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr t(new pcl::PointCloud<pcl::PointXYZ>);
    point_cloud = t;

    x_min = x_max = y_min = y_max = z_min = z_max = 0;

}

void Muxin::stop()
{
    system_on = false;
    running_handle.join();
}

void Muxin::set_range_xyz(float x_min,float x_max,
float y_min,float y_max,float z_min,float z_max)
{
    this->x_min = x_min/1000.0;
    this->x_max = x_max/1000.0;
    this->y_min = y_min/1000.0;
    this->y_max = y_max/1000.0;
    this->z_min = z_min/1000.0;
    this->z_max = z_max/1000.0;
}

bool Muxin::start()
{
    running_handle = std::thread(&NS_MUXIN::Muxin::running,this);
}

void Muxin::running()
{
    printf("muxin get range image running!\n");
    char rec_topic_name[MAX_TOPIC_NAME_LEN];
    size_t option_len;
    bool not_too_close = false;
    int data_len = 0;
    msg_info_t info;
    
    uint8_t data[MAX_DDS_DATA_SIZE]={0};
    static uint8_t count_ = 0;
    while(system_on)
    {
        NS_TIMEMGR_E::TimeMgr time;
        time.time_init();
        data_len = zmq_recv(s_remote_sub_socket, rec_topic_name, sizeof(rec_topic_name), 0);
        if (data_len <= 0)
        {
            printf("invalid data len: %d\n", data_len);
            printf("reconnecting \n");
            reconnect_remote_peers(&s_remote_sub_socket, REMOTE_IP_ADDR);
            import_remote_data(topic_names, s_remote_sub_socket);
            continue;
        }
        rec_topic_name[data_len] = '\0';
        if (!zmq_getsockopt(s_remote_sub_socket, ZMQ_RCVMORE, NULL, &option_len))
        {
            printf("invalid remote packet\n");
            continue;
        }
        data_len = zmq_recv(s_remote_sub_socket, data, sizeof(data), 0);
        memcpy(&info, data, sizeof(msg_info_t));
        if ((data_len != sizeof(msg_info_t)) || INFO_MAGIC != info.magic)
        {
            if((!strcmp(rec_topic_name, "mx_version") ) && (data_len == sizeof(mx_version_info_t)))
            {
                mx_version_info_t *mx_version = (mx_version_info_t *) (data);
                // printf("mx version %d. %d. %d. build time: %s\n", mx_version->mx_version_major,
                //        mx_version->mx_version_minor,
                //        mx_version->mx_version_patch,
                //        mx_version->build_time);
                continue;
            }
            else
            {
                printf("receve msg info failed\n");
                continue;
            }
        }
        
        if (!zmq_getsockopt(s_remote_sub_socket, ZMQ_RCVMORE, NULL, &option_len))
        {
            printf("data body not come after msg info\n");
            continue;
        }
        data_len = zmq_recv(s_remote_sub_socket, data, sizeof(data), 0);
        if (data_len != info.size)
        {
            printf("invalid data body len: %d, expect: %d\n", data_len, info.size);
            continue;
        }
        bool ground_mode = false;
        if(!strcmp(rec_topic_name, "stereo_depth_map"))
        {
            // get cali para
            camera_parameters_t cam_para;
            int32_t scale = 16;
            int32_t width = *(int32_t*)(data+12);
            int32_t height = *(int32_t*)(data+16);
            cam_para.cx = *(float*)(data+32);
            cam_para.cy = *(float*)(data+36);
            cam_para.fx = *(float*)(data+40);
            cam_para.base_line = *(float*)(data+44);
            uint32_t depth_flag = *(uint32_t*)(data+48);
            ground_mode = depth_flag&GROUND_MODE;
            not_too_close = depth_flag&NOT_TOO_CLOSE;
            // get data pointer
            int16_t *disp = (int16_t*)(data+data_len-2*width*height);
            
            bool temp = (not_too_close == 1)?false:true;
            // if(ground_mode)
            push_depth_map(disp, width, height, scale, cam_para,temp);
            // printf("[%d] ground_mode = %d time = %d\n",++count_,ground_mode,time.time_eraUs()/1000);
            
        }

    }

}

void Muxin::push_depth_map(int16_t *disp, int32_t width, 
    int32_t height, int32_t scale, camera_parameters_t camera,bool is_too_close)
{
    NS_TIMEMGR_E::TimeMgr time;
    time.time_init();
    std::lock_guard<std::mutex> lock(mutex_cashe);
    static uint8_t index = 0;
    if(index >= NUMS_CASHE)
    {
        index = 0;
    }
    
    if(is_too_close)
    {
        for(int i(0);i<NUMS_CASHE;i++)
        {
            cashes[i] = new int16_t[RESO_COL*RESO_ROW];
            memset(cashes[i],0,RESO_COL*RESO_ROW*(sizeof(int16_t)));
        }
    }
    else
    {
        memcpy(cashes[index],disp,width*height*(sizeof(int16_t)));
    }

    index++;
    cashe_camera = camera;
    cashe_scale = scale;
    cashe_width = width;
    cashe_height = height;
    cashe_too_close = is_too_close;

    
    
    
}

pcl::PointCloud<pcl::PointXYZ>::Ptr& Muxin::get_depth_map(bool& too_close)
{
    NS_TIMEMGR_E::TimeMgr time;
    time.time_init();
    std::lock_guard<std::mutex> lock(mutex_cashe);
    point_cloud->points.clear();
    int img_scale = 1;
    if( 320 == cashe_width)
    {
        img_scale = 2;
    }

    float R[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    float T[3] = {0, 0, 0};

    pcl::PointXYZ tmp_point;
    for (int i=0;i<cashe_height;i++)
    {
        for (int j=0;j<cashe_width;j++)
        {
            for(int k(0);k<NUMS_CASHE;k++)
            {
                float d = cashes[k][i*cashe_width+j]/(float)cashe_scale;

                if (d>0)
                {
                    tmp_point.x = (j*img_scale-cashe_camera.cx)/d*cashe_camera.base_line;
                    tmp_point.y = (i*img_scale-cashe_camera.cy)/d*cashe_camera.base_line;
                    tmp_point.z = cashe_camera.fx/d*cashe_camera.base_line;
                    
                    pcl::PointXYZ tmp_point_recali;
                    tmp_point_recali.x = tmp_point.x*R[0] + tmp_point.y*R[1] +tmp_point.z*R[2] + T[0];
                    tmp_point_recali.y = tmp_point.x*R[3] + tmp_point.y*R[4] +tmp_point.z*R[5] + T[1];
                    tmp_point_recali.z = tmp_point.x*R[6] + tmp_point.y*R[7] +tmp_point.z*R[8] + T[2];
                    
                    if(
                        (tmp_point_recali.x>x_min&&tmp_point_recali.x<x_max)
                        &&(tmp_point_recali.y>y_min&&tmp_point_recali.y<y_max)
                        &&(tmp_point_recali.z>z_min&&tmp_point_recali.z<z_max)
                    )
                    {
                       
                        tmp_point_recali.x *= 1000.0;
                        tmp_point_recali.y *= 1000.0;
                        tmp_point_recali.z *= 1000.0;
                        point_cloud->points.push_back(tmp_point_recali);
                    }

                    break;      //其中一帧有就跳出循环

                }

            }

        }

    }
    too_close = cashe_too_close;
    // printf("time = %d\n",time.time_eraUs()/1000);
    return point_cloud;
    
}

int32_t Muxin::reproject_to_3d(int16_t *disp, int32_t width, int32_t height, int32_t scale, camera_parameters_t camera)
{
    float R[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    float T[3] = {0, 0, 0};
    static uint32_t count = 0;
    static uint32_t count1 = 0;
    int img_scale = 1;
    if( 320 == width)
    {
        img_scale = 2;
    }
    pcl::PointXYZ tmp_point;
    for (int i=0;i<height;i++)
        for (int j=0;j<width;j++)
        {
            count1++;
            float d = disp[i*width+j]/(float)scale;
            if (d>0)
            {
                tmp_point.x = (j*img_scale-camera.cx)/d*camera.base_line;
                tmp_point.y = (i*img_scale-camera.cy)/d*camera.base_line;
                tmp_point.z = camera.fx/d*camera.base_line;

                pcl::PointXYZ tmp_point_recali;
                tmp_point_recali.x = tmp_point.x*R[0] + tmp_point.y*R[1] +tmp_point.z*R[2] + T[0];
                tmp_point_recali.y = tmp_point.x*R[3] + tmp_point.y*R[4] +tmp_point.z*R[5] + T[1];
                tmp_point_recali.z = tmp_point.x*R[6] + tmp_point.y*R[7] +tmp_point.z*R[8] + T[2];

                if(
                    (tmp_point_recali.x>x_min&&tmp_point_recali.x<x_max)
                    &&(tmp_point_recali.y>y_min&&tmp_point_recali.y<y_max)
                    &&(tmp_point_recali.z>z_min&&tmp_point_recali.z<z_max)
                )
                {
                    count++;
                    tmp_point_recali.x *= 1000.0;
                    tmp_point_recali.y *= 1000.0;
                    tmp_point_recali.z *= 1000.0;
                    // point_cloud->points.push_back(tmp_point);
                }
                
                // printf("x=%f y=%f z=%f\n",tmp_point.x,tmp_point.y,tmp_point.z);
            }
        }
    printf("reject = %d %d\n",count,count1);
    count = 0;
    count1 = 0;
    return 0;
}   


int Muxin::connect_remote_peers(void **socket, char * ip)
{
    int ret;
    char addr[256];
    sprintf(addr, "tcp://%s:%s", ip, REMOTE_PORT);

    s_zmq_ctx = zmq_ctx_new();
    ret = zmq_ctx_set(s_zmq_ctx, ZMQ_IO_THREADS, 1);
    if (ret)
    {
        printf("zmq_ctx_set( failed, ret: %d, err: %s\n", ret, zmq_strerror(errno));
        return -1;
    }

    *socket = zmq_socket(s_zmq_ctx, ZMQ_XSUB);
    printf("connect to: %s\n", addr);
    zmq_connect(*socket, addr);
    int32_t timeout = TIME_OUT_MS;
    zmq_setsockopt(*socket, ZMQ_RCVTIMEO, &timeout, sizeof(timeout));
    return 0;
}

int Muxin::reconnect_remote_peers(void **socket, char * ip)
{
    int ret;
    char addr[256];
    sprintf(addr, "tcp://%s:%s", ip, REMOTE_PORT);
    zmq_close(*socket);
    zmq_ctx_destroy(s_zmq_ctx);

    s_zmq_ctx = zmq_ctx_new();
    ret = zmq_ctx_set(s_zmq_ctx, ZMQ_IO_THREADS, 1);
    if (ret)
    {
        printf("zmq_ctx_set( failed, ret: %d, err: %s\n", ret, zmq_strerror(errno));
        return -1;
    }

    *socket = zmq_socket(s_zmq_ctx, ZMQ_XSUB);
    printf("reconnect to: %s\n", addr);
    zmq_connect(*socket, addr);
    int32_t timeout = TIME_OUT_MS;
    zmq_setsockopt(*socket, ZMQ_RCVTIMEO, &timeout, sizeof(timeout));


    return 0;
}

int Muxin::import_remote_data(const char *names[], void *socket)
{
    char buf[MAX_TOPIC_NAME_LEN];
    buf[0] = 1;
    for (uint32_t i = 0;; i++)
    {
        if (NULL != names[i])
        {
            printf("import remote data: %s\n", names[i]);
            uint32_t topic_name_len = strlen(names[i]);
            memcpy(&buf[1], names[i], topic_name_len);
            zmq_send(socket, buf, topic_name_len + 1, 0);
        }
        else
        {
            break;
        }
    }
    return 0;
}



