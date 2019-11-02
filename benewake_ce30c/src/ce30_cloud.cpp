#include <ros/ros.h>
#include <ce30c_driver/ce30_sdk.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#define PHEIGHT 24
#define PWIDTH 660
typedef struct weight{
    float x;
    float y;
    float h;
    int n;
}weight;
typedef struct process{
    std::vector<float> p_s;
    std::vector<float> p_e;
    std::vector<float> p_x;
    std::vector<float> p_h;
    int p_n;
}process;
class Cloud{
    public:
    void init();
    void camera(const float camera_f);
    void point_pose(const unsigned short *range);
    void pub_cloud(std::string _frame_id);
    std::vector<int> diff_cloud();
    std::vector<weight> clust_cloud(const unsigned short *range);
    process people_count(const std::vector<weight> w_p, process pre);

    pcl::PointCloud<pcl::PointXYZ> cloud;

    std::vector<float> pose_x;
    std::vector<float> pose_y;
    // std::vector<float> pose_z;
    private:
    ros::NodeHandle nh;
    ros::Publisher pcl_pub;
    sensor_msgs::PointCloud2 output;
    std::vector<unsigned short> env;
};

void Cloud::init()
{
    cloud.width = PWIDTH;
    cloud.height = PHEIGHT;
    cloud.points.resize(cloud.width * cloud.height);
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("ce30c_output", 1);
    Cloud::camera(150);
}
void Cloud::camera(const float camera_f)
{
    pose_x.clear();
    pose_y.clear();
    // pose_z.clear();
    for(int j = 0; j<PHEIGHT; j++)
    {
        for(int i = 0; i<PWIDTH; i++)
        {
            float p_x, p_y;
            p_x = (i + 0.5 - PWIDTH/2)/camera_f;
            p_y = (PHEIGHT/2 - j - 0.5)/camera_f;
            pose_x.push_back(p_x);
            pose_y.push_back(p_y);
        }
    }
}

void Cloud::point_pose(const unsigned short *range)
{
    for(int i = 0; i<cloud.points.size(); i++)
    {
        if(range[i] > 10){
            cloud.points[i].x = pose_x[i]*range[i]/100.0f;
            cloud.points[i].y = range[i]/100.0f;
            cloud.points[i].z = pose_y[i]*range[i]/100.0f;
        }else{
            cloud.points[i].x = 0;
            cloud.points[i].y = 0;
            cloud.points[i].z = 0;
        }
    }
}

void Cloud::pub_cloud(std::string _frame_id)
{
    pcl::toROSMsg(cloud, output);
    output.header.frame_id = _frame_id;
    output.header.stamp = ros::Time::now();
    pcl_pub.publish(output);
}

bool gRun = true;

void sig_stop(int value)
{
    gRun = false;
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ce30_cloud");
    ros::NodeHandle privnh("~");
    gRun = true;
    signal(SIGINT, sig_stop);

    Cloud ce30_c;
    process result;
    result.p_n = 0;
    ce30_c.init();
    printf("Init device\n");
    int sd = 0;
    std::string ip, newip, frame_id;
    // get ip
    privnh.param("IP", ip, std::string("192.168.1.80"));
    privnh.param("frame_id", frame_id, std::string("ce30c_pointcloud"));

    int first = 0;
    // connect lidar
    char* addr = const_cast<char*>(ip.c_str());
    ROS_INFO("Connect to: %s.", addr);
    if(!benewake::initDevice(sd, addr))
    {
        ROS_ERROR("Connection failed! The node exited.");
        return -1;
    }
    ROS_INFO("Connection succeeded!");
    // reset ip
    if(privnh.param("newIP", newip, std::string("192.168.1.80")))
    {
        char* new_addr = const_cast<char*>(newip.c_str());
        benewake::changeIPAddress(sd, new_addr);
        ROS_INFO("LiDAR is rebooting! The node exited. Please restart the node later with new IP address.");
        privnh.deleteParam("/ce30_cloud/newIP");
        return 0;
    }

    int h = 24, w = 660;
    unsigned short data[h * w], amp[h * w] , tmp = 0;
    std::vector<weight> a;
    // start measurement
    if(!benewake::startMeasurement(sd, 0))
    {
        ROS_ERROR("Start measurement failed! The node exited.");
        return -1;
    }
    ROS_INFO("Start measurement ...");
    while(ros::ok()&&gRun)
    {
        if(!benewake::getDistanceData(sd, data, amp))
        {
            benewake::stopMeasurement(sd);
            benewake::closeDevice(sd);
            close(sd);

            return -1;
        }
        else
        {
            ce30_c.point_pose(data);
            ce30_c.pub_cloud(frame_id);
        }
    }
    benewake::stopMeasurement(sd);
    ROS_INFO("Measurement stopped.");
    benewake::closeDevice(sd);
    ROS_INFO("Device Disconneted!");
    close(sd);
    ROS_INFO("Socket closed!");
    return 0;
}
