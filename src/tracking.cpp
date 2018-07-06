#include <cmath>
#include <ros/ros.h>
#include "hungarian.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/impl/point_types.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <pcl/filters/extract_indices.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pointcloud_msgs/PointCloud2_Segments.h>

class Centroid_tracking{
public:

    int max_id ; 

    pointcloud_msgs::PointCloud2_Segments base_msg;

    Centroid_tracking ( pointcloud_msgs::PointCloud2_Segments& base_msg, int max_id ) 
    {
        this->base_msg = base_msg ;
        this->max_id = max_id ;
    }

    void track ( pointcloud_msgs::PointCloud2_Segments& msg ) {

        std::vector<Eigen::Vector4f> msg_centroid_vec;
        std::vector<Eigen::Vector4f> base_centroid_vec;

        //first frame 
        for (int i=0; i < base_msg.clusters.size(); i++)
        {
            pcl::PointXYZ centroidpoint ;
            pcl::PCLPointCloud2 pc2;
            pcl_conversions::toPCL ( base_msg.clusters[i] , pc2 );

            pcl::PointCloud<pcl::PointXYZ> cloud2;
            pcl::fromPCLPointCloud2 ( pc2 , cloud2 );

            Eigen::Vector4f base_centroid;
            pcl::compute3DCentroid ( cloud2 , base_centroid);
            //std::cerr << " Cluster_id " << base_msg.cluster_id[i] << "  centroid_x : " << base_centroid(0) << " centroid_y : " << base_centroid(1) << " centroid_z : " << base_centroid(2) << std::endl;

            base_centroid_vec.push_back( base_centroid );
        }
        //second frame
        for (int i=0; i < msg.clusters.size(); i++)
        {
            pcl::PointXYZ centroidpoint ;
            pcl::PCLPointCloud2 pc2;
            pcl_conversions::toPCL ( msg.clusters[i] , pc2 );

            pcl::PointCloud<pcl::PointXYZ> cloud2;
            pcl::fromPCLPointCloud2 ( pc2 , cloud2 );

            Eigen::Vector4f base_centroid;
            pcl::compute3DCentroid ( cloud2 , base_centroid);
            //std::cerr << " ---Cluster_id " << msg.cluster_id[i] << "  centroid_x : " << base_centroid(0) << " centroid_y : " << base_centroid(1) << " centroid_z : " << base_centroid(2) << std::endl;

            msg_centroid_vec.push_back( base_centroid );
            msg.cluster_id[i] = -1;

        }

        size_t size_old = base_centroid_vec.size();
        size_t size_new = msg_centroid_vec.size();
        unsigned totalsz = size_old + size_new;
        std::vector<std::vector<int> > dists(totalsz, std::vector<int>(totalsz , 10000));// TODO currently, 10000 is the maximum (2d) int distance with a 10 meter laser scanner. Initial value represents a point connected to bottom.

        for(unsigned i=0; i < size_old + size_new; i++){
            for(unsigned j=0; j < size_new + size_old; j++){
                if(i < size_old and j < size_new){
                    dists[i][j] = 1000 * sqrt(pow(base_centroid_vec[i][0]-msg_centroid_vec[j][0], 2) + pow(base_centroid_vec[i][1]-msg_centroid_vec[j][1], 2) + pow(base_centroid_vec[i][2]-msg_centroid_vec[j][2], 2));
                }
                else if(i >= size_old and j >= size_new){
                    dists[i][j] = 0; // connecting bottom to bottom is free!
                }
            }
        }

        Hungarian hungarian(dists , totalsz, totalsz, HUNGARIAN_MODE_MINIMIZE_COST) ;
        // fprintf(stderr, "cost-matrix:");
        // hungarian.print_cost();
        hungarian.solve();
        // fprintf(stderr, "assignment:");
        // hungarian.print_assignment();
        // std::cout << "size_old = " << size_old << ", size_new = " << size_new << std::endl;

        dists = hungarian.assignment();

        for(unsigned j=0; j < size_new; j++){
            for(unsigned i=0; i < size_old; i++){
                if (dists[i][j] == 1){
                    msg.cluster_id[j] = base_msg.cluster_id[i];
                    break;
                }
            }
            msg.cluster_id[j] = msg.cluster_id[j] == -1 ? ++max_id : msg.cluster_id[j];
        }

        // for (unsigned j=0; j < size_new; j++){
        //     std::cout << "cluster#" << j << ", clusterID:" << msg.cluster_id[j] << std::endl;
        // }
    }

};

ros::Publisher pub;
ros::Publisher pub2;
ros::Subscriber sub;

bool b = true;
int size , max_id ;
double time_offset ;
double overlap, offset ;

double z_overlap_height_min , z_overlap_height_max , height_after, height_before;

std::vector<pointcloud_msgs::PointCloud2_Segments> v;
std::vector<pointcloud_msgs::PointCloud2_Segments> v_;
std::vector<pointcloud_msgs::PointCloud2_Segments> new_v;


std::pair<double,double> calculate_offset (const pointcloud_msgs::PointCloud2_Segments vector , const pointcloud_msgs::PointCloud2_Segments& msg){
    pointcloud_msgs::PointCloud2_Segments c_;

    v.push_back(vector);
    for (unsigned i=0; i < v.size(); i++)
    {
        double offset;
        if ( i > 0 ){
            time_offset = (double)( ros::Duration( v[i].first_stamp - v[0].first_stamp ).toSec()) * (double)( msg.factor ) ;
            offset = ( 1.0 - overlap ) * time_offset;
            // ROS_WARN("2333:%f", (double)( ros::Duration( v[i].first_stamp - v[0].first_stamp ).toSec()));
        }
        else {
            offset = 0.0;
        }

        for (unsigned j=0; j < v[i].clusters.size(); j++){
            sensor_msgs::PointCloud cloud;
            sensor_msgs::convertPointCloud2ToPointCloud( v[i].clusters[j] , cloud );
            // ROS_WARN("1");
            for (unsigned k=0; k < cloud.points.size(); k++){

                height_before = cloud.points[k].z ;
                cloud.points[k].z = height_before + offset ;
                // ROS_WARN("3:%f", height_before);
                height_after = cloud.points[k].z ;
                // ROS_WARN("34:%f", height_after);
                /* the height of the overlap */
                if ( height_before != height_after and height_before != NAN and height_after != NAN){
                    z_overlap_height_min = height_before - (1.0 - (offset / time_offset));
                    z_overlap_height_max = height_after - height_before ;
                    ROS_WARN("1:%f" , z_overlap_height_max);
                }
            }
        }
    }
    return std::make_pair(z_overlap_height_min, z_overlap_height_max) ;

}



void callback (const pointcloud_msgs::PointCloud2_Segments& msg ){

    pointcloud_msgs::PointCloud2_Segments c_;
    sensor_msgs::PointCloud cloud;
//v_ has the original msg clusters with pointcloud2
    v_.push_back(msg);
    // for (int i=0; i < v_.size(); i++){
        std::pair<double,double> z_height = calculate_offset(v_[0] , msg);
        ROS_WARN("000:%f ", z_overlap_height_max);
        ROS_WARN("333:%f ", z_overlap_height_min);
        // ROS_WARN("2:%u", v_[0]);
        z_overlap_height_min = z_height.first ;
        ROS_WARN("111:%f ", z_overlap_height_min);
        // ROS_WARN("22222:%f ", z_overlap_height_min);

        z_overlap_height_max = z_height.second;
    // }

    //new_v is a vector with clusters and is empty
    // c_ is a pointcloud msg 

    if ( z_overlap_height_min != z_overlap_height_max and z_overlap_height_max != NAN ){
        for (unsigned i=0; i < v_.size(); i++)
       {
            for (unsigned j=0; j < v_[i].clusters.size(); j++){
                sensor_msgs::PointCloud cloud;
                sensor_msgs::convertPointCloud2ToPointCloud( v_[i].clusters[j] , cloud );
                // ROS_WARN("4");
                for (unsigned k=0; k < cloud.points.size(); k++){

                    if ( cloud.points[k].z > z_overlap_height_min and cloud.points[k].z < z_overlap_height_max){
                        pcl::PCLPointCloud2 pc2 ;
                        sensor_msgs::PointCloud2 cl2;

                        sensor_msgs::convertPointCloudToPointCloud2( cloud , cl2 );
                        pcl_conversions::toPCL ( cl2 , pc2 );

                        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
                        pcl::fromPCLPointCloud2 ( pc2 , *cloud2 );
                        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
                        pcl::ExtractIndices<pcl::PointXYZ> extract ;
                        for (int i=0; i < (*cloud2).size(); i++){
                            pcl::PointXYZ pt(cloud2->points[i].x , cloud2->points[i].y , cloud2->points[i].z);
                            if (pt.z < z_overlap_height_max and pt.z > z_overlap_height_min){
                                inliers->indices.push_back(i);
                            }
                        }
                        extract.setInputCloud(cloud2);
                        extract.setIndices(inliers);
                        extract.setNegative(true);
                        extract.filter(*cloud2);
                        ROS_WARN("5");
                        sensor_msgs::PointCloud2 output;
                        pcl::PCLPointCloud2 pcl_cloud;
                        pcl::toPCLPointCloud2(*cloud2, pcl_cloud);
                        pcl_conversions::fromPCL(pcl_cloud, output);
                        c_.clusters.push_back( output );
                        new_v.push_back(c_);
                    }
                }
                // ROS_WARN("2222:%u", new_v.size());
            }
        }
    }

    Centroid_tracking* t;

    if (new_v.size() > size){
        new_v.erase(new_v.begin());
        if ( b ){
            for (unsigned i=0 ; i < new_v[0].clusters.size(); i++){
                new_v[0].cluster_id.push_back(i);
                // ROS_WARN("3");
            }
            b = false;
        }
        for (int i=0 ; i < new_v[1].clusters.size(); i++){
            new_v[1].cluster_id.push_back(i);
        }

        for (unsigned i=0; i < new_v[0].cluster_id.size(); i++){
            if (new_v[0].cluster_id[i] > max_id){
                max_id = new_v[0].cluster_id[i];
                ROS_WARN("11");
                // std::pair<double,double> z_height = calculate_offset(new_v[0] , msg);
            }
        }
        t = new Centroid_tracking( new_v[0] , max_id ) ;
    }

    else {
        t = NULL;
    }

    if ( t != NULL ) {
        // ROS_WARN("0:%u" , v_[1].cluster_id[0]);
        // ROS_WARN("6");
        t->track( new_v[1] );
        // ROS_WARN("1:%u" , v_[1].cluster_id[0]);
    }
    for (unsigned i=0; i < new_v.size(); i++){
        for (unsigned j=0; j < new_v[i].clusters.size(); j++){
            for (unsigned k=0; k < new_v[i].cluster_id.size(); k++){
                c_.cluster_id.push_back(new_v[i].cluster_id[k]);
                // ROS_WARN("7");
            }
        }
    }

    pub.publish(c_);

}


int main(int argc, char** argv){

    ros::init(argc, argv, "pointcloud2_cluster_tracking");
    ros::NodeHandle n_;

    std::string out_topic;
    std::string input_topic;


    n_.param("pointcloud2_cluster_tracking/size", size , 2);
    n_.param("pointcloud2_cluster_tracking/overlap", overlap , 0.2);

    n_.param("pointcloud2_cluster_tracking/out_topic", out_topic , std::string("/pointcloud2_cluster_tracking/clusters"));
    n_.param("pointcloud2_cluster_tracking/input_topic", input_topic , std::string("pointcloud2_clustering/clusters"));

    sub = n_.subscribe( input_topic, 1 , callback);
    pub = n_.advertise<pointcloud_msgs::PointCloud2_Segments>( out_topic, 1);

    ros::spin();
}