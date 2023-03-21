#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>
#include <stdlib.h>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <iostream>

ros::Publisher pub;
void SaveToPCD(const pcl::PointCloud<pcl::PointXYZI>  & cloud, std::string &path)
{
    std::cout<<"PCD path :" <<path <<std::endl;
    pcl::io::savePCDFileASCII(path,cloud);

}
void SaveToBin(const pcl::PointCloud<pcl::PointXYZI>  & cloud, std::string &path)
{   
    std::cout << "Bin path: " <<path <<std::endl;
    std::ofstream out(path.c_str(), std::ios::out|std::ios::binary|std::ios::app);
    if(!out.good())
    {
        std::cout << "Couldn't open " << path <<std::endl;
        return;
    }
    for (size_t i = 0; i < cloud.size(); i++)
    {
        out.write((char*)&cloud.points[i].x,3*sizeof(float));
        out.write((char*)&cloud.points[i].intensity,sizeof(float));
    }
    out.close();
}
void ToPublish(const pcl::PointCloud<pcl::PointXYZI>  & cloud)
{
    sensor_msgs::PointCloud2 msg_publish;
    pcl::toROSMsg(cloud,msg_publish);
    msg_publish.header.frame_id="top_lidar";
    pub.publish(msg_publish);
}

int Rosbag2PcdAndBin(std::string path)
{
    boost::filesystem::path boost_path(path);
    std::string  parent_path = boost_path.parent_path().string();
    std::string stem = boost_path.stem().string();
    std::cout <<"input :  " <<path <<std::endl;
    int frame_cnt =0;
    rosbag::Bag bag;
    bag.open(path,rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back(std::string("/top/rslidar_points"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    for(rosbag::MessageInstance const m: view)
    {
        sensor_msgs::PointCloud2::ConstPtr input = m.instantiate<sensor_msgs::PointCloud2>();
        ++frame_cnt;
        if(input == NULL)
        {
            continue;
        }
        else
        {
            int point_bytes = input->point_step;
            int offset_x,offset_y,offset_z,offset_intensity;
            const auto& fields = input->fields;
            for (int f = 0; f < fields.size(); ++f) {
                if (fields[f].name == "x") offset_x = fields[f].offset;
                if (fields[f].name == "y") offset_y = fields[f].offset;
                if (fields[f].name == "z") offset_z = fields[f].offset;
                if (fields[f].name == "intensity") offset_intensity = fields[f].offset;
        }
        bool do_convert =false;
        pcl::PointCloud<pcl::PointXYZI> cloud;//定义转换的点云数据类型
        if (do_convert) 
        {
            for (int i=0; i<input->width; ++i)
            {
                pcl::PointXYZI point;
                point.x = *(float*)(input->data.data() + point_bytes*i + offset_x);
                point.y = *(float*)(input->data.data() + point_bytes*i + offset_y);
                point.z = *(float*)(input->data.data() + point_bytes*i + offset_z);
                point.intensity = *(unsigned char*)(input->data.data() + point_bytes*i + offset_intensity);
                auto tmp = point.x;
                point.x = point.z;
                point.y = -point.y;
                point.z = tmp;
                cloud.push_back(point);
            }
            cloud.width = input->width;
            cloud.height = input->height;
        } 
        else 
        {
            pcl::fromROSMsg(*input, cloud);  
        }
        if(cloud.empty())
        {
            std::cerr <<"path " <<path <<", frame" <<frame_cnt <<" no point cloud" << std::endl;
            continue;
        }
        std::string pcd_path = parent_path + "/"+"pcd/" +  std::to_string(frame_cnt) + ".pcd";
        SaveToPCD(cloud, pcd_path);
        std::string bin_path = parent_path + "/" +"bin/"+ std::to_string(frame_cnt) + ".bin";
        SaveToBin(cloud, bin_path);
        ToPublish(cloud);
    }
   }
   bag.close();
   return 0;
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"ROSBAGBINPCD");
    ros::NodeHandle nh;
    //ros::Publisher pub;
    pub = nh.advertise<sensor_msgs::PointCloud2>("cloud",1);
    
    std::string data_dir("/home/wf/plusgo/daxie_radar");
    boost::filesystem::directory_iterator end_iter;
    for(boost::filesystem::directory_iterator iter(data_dir);iter!=end_iter;iter++)
    {
        if(boost::filesystem::is_regular_file(*iter) && (*iter).path().string().find(".bag") != std::string::npos)
        {
            std::cout<<"(*iter).path() :"<<(*iter).path().string()<<std::endl;
            Rosbag2PcdAndBin((*iter).path().string());
        }
    }
}