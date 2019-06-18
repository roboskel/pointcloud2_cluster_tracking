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
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <algorithm>    // std::find
#include <vector>       // std::vector


bool first_time = true;
bool marker_flag, trackTheUntracked;
int maxHungDist;
double minMotionDist, max_dist;

visualization_msgs::Marker marker_sphere, marker_line;

std::vector<int> ids, idss, clusterInMotion;
std::vector<bool> prob_extinction,trackedOrnotIds;;
std::vector<Eigen::Vector4f> centroids;



std::vector<float> red = {0, 0, 1, 1, 1, 102.0/255, 102.0/255, 204.0/255, 0, 1};
std::vector<float> green = {0, 1.0, 0, 1, 1, 102.0/255, 102.0/255, 0, 1, 152.0/255};
std::vector<float> blue = {1.0, 0, 0, 0, 1, 152.0/255, 52.0/255, 152.0/255, 1, 52.0/255};


std::pair<double,double> minmaxz (sensor_msgs::PointCloud2 clust){  //return max and min z of cluster


    pcl::PCLPointCloud2 p2;
    pcl_conversions::toPCL ( clust , p2 ); //from sensor_msgs::pointcloud2 to pcl::pointcloud2

    pcl::PointCloud<pcl::PointXYZ> p3;
    pcl::fromPCLPointCloud2 ( p2 , p3 );       //from pcl::pointcloud2 to pcl::pointcloud

    double max_z = p3.points[0].z;
    double min_z = p3.points[0].z;
    for (int i=1; i < p3.points.size(); i++){   //find max and min z of cluster 
        if(p3.points[i].z > max_z){
            max_z = p3.points[i].z;
        }
        if(p3.points[i].z < min_z){
            min_z = p3.points[i].z;
        }
    }

    return std::make_pair( max_z , min_z );
}

pcl::PointCloud<pcl::PointXYZ> saveAllPoints(sensor_msgs::PointCloud2 clust){  //save all points of a cluster to pointcloud form


    pcl::PCLPointCloud2 pc2;
    pcl_conversions::toPCL ( clust , pc2 );   //from sensor_msgs::pointcloud2 to pcl::pointcloud2

    pcl::PointCloud<pcl::PointXYZ> pc;
    pcl::fromPCLPointCloud2 ( pc2 , pc );               //from pcl::pointcloud2 to pcl::pointcloud

    return pc;
}

pcl::PointCloud<pcl::PointXYZ> saveAllZValuePoints(sensor_msgs::PointCloud2 clust, double zvalue){  //save all points whose z is equal to zvalue

    pcl::PointCloud<pcl::PointXYZ> pcz3;
    pcl::PointCloud<pcl::PointXYZ> pc;
    pc=saveAllPoints(clust);

    for(int i=0; i < pc.points.size(); i++){        //add points with zvalue to a new pointcloud
        if(pc.points[i].z == zvalue){
            pcz3.push_back(pc.points[i]);
        }
    }

    return pcz3;
}

pcl::PointCloud<pcl::PointXYZ> saveAllZPointsFrom(sensor_msgs::PointCloud2 clust, double zFrom){   // save all points whose z is grater than or equal to zFrom

    pcl::PointCloud<pcl::PointXYZ> pcz3;
    pcl::PointCloud<pcl::PointXYZ> pc;
    pc=saveAllPoints(clust); 

    for(int i=0; i < pc.points.size(); i++){        //add points with max z to a new pointcloud
        if(pc.points[i].z >= zFrom  ){
            pcz3.push_back(pc.points[i]);
        }
    }

    return pcz3;
}

pcl::PointCloud<pcl::PointXYZ> saveAllZPointsUntil(sensor_msgs::PointCloud2 clust, double zUntil){   // save all points whose z is less than or equal to zUntil

    pcl::PointCloud<pcl::PointXYZ> pcz3;
    pcl::PointCloud<pcl::PointXYZ> pc;
    pc=saveAllPoints(clust);

    for(int i=0; i < pc.points.size(); i++){        //add points with max z to a new pointcloud
        if(pc.points[i].z <= zUntil  ){
            pcz3.push_back(pc.points[i]);
        }
    }

    return pcz3;
}


void trackUntrackedClusters(std::vector<Eigen::Vector4f> untracked_msg, pointcloud_msgs::PointCloud2_Segments& msg, std::vector<Eigen::Vector4f> msg_centroid_vec, size_t size_new  ){ //try to track the untracked clusters with untracked centroids 

    std::vector<Eigen::Vector4f> untracked_centr;

    for (int i=0; i < ids.size(); i++) {
        if (trackedOrnotIds[i] == false)
            untracked_centr.push_back(centroids[i]);
    }

    int centrs_size = untracked_centr.size();
    int msg_size = untracked_msg.size();

    if (centrs_size != 0 && msg_size != 0 ) {

        std::vector<std::vector<int> > newdists(centrs_size, std::vector<int>(msg_size , 10000));// TODO currently, 10000 is the maximum (2d) int distance with a 10 meter laser scanner. Initial value represents a point connected to bottom.

        for(unsigned i=0; i < centrs_size; i++){
            for(unsigned j=0; j < msg_size; j++){
                newdists[i][j] = 1000 * sqrt(pow(untracked_centr[i][0]-untracked_msg[j][0], 2) + pow(untracked_centr[i][1]-untracked_msg[j][1], 2));                 
            }
        }

        Hungarian::Result p = Hungarian::Solve(newdists, Hungarian::MODE_MINIMIZE_COST);

        // Hungarian::PrintMatrix(p.assignment);

        newdists = p.assignment;

        double distt;
        for(unsigned j=0; j < msg_size; j++){
            for(unsigned i=0; i < centrs_size; i++){

                if (newdists[i][j] == 1){

                    distt = 1000 * sqrt(pow(untracked_centr[i][0]-untracked_msg[j][0], 2) + pow(untracked_centr[i][1]-untracked_msg[j][1], 2));

                    if (distt<maxHungDist) {

                        int temp_pos = -1;

                        for(int k=0; k < centroids.size(); k++) {
                            if(untracked_centr[i] == centroids[k] && trackedOrnotIds[k] == false ){

                                temp_pos = k;
                                trackedOrnotIds[k] = true;
                                break;
                            }
                        }          
                        if(temp_pos != -1) {

                            for(int k=0; k < size_new; k++) {
                                if(untracked_msg[j] == msg_centroid_vec[k] && msg.cluster_id[k] == -1 ){

                                    msg.cluster_id[k] = ids[temp_pos];
                                    centroids[temp_pos]=msg_centroid_vec[k];
                                    break;
                                }
                            }
                        }
                    }
                    break; 
                }           
            }
        }
    } 
}


void checkClustersDistance(pointcloud_msgs::PointCloud2_Segments base_msg, size_t size_old, pointcloud_msgs::PointCloud2_Segments& msg ){ // check if the distance between two clusters is less than max_dist in base_msg. if it is true check if one of two clusters missing from the msg

    std::vector<pcl::PointCloud<pcl::PointXYZ>> pcz(size_old); 

    double max_z;

    for(int l=0; l<clusterInMotion.size(); l++){
        for(unsigned j=0; j < base_msg.clusters.size(); j++){
            if(base_msg.cluster_id[j] == clusterInMotion[l]){

                std::pair<double,double> temp_maxz = minmaxz(base_msg.clusters[j]);
                max_z = temp_maxz.first;

                pcz[j]=saveAllZPointsFrom(base_msg.clusters[j], (max_z + base_msg.middle_z)/2);

                for(unsigned i=0; i < base_msg.clusters.size(); i++){
                    if(i!=j){

                        temp_maxz = minmaxz(base_msg.clusters[i]);
                        max_z = temp_maxz.first;

                        pcz[i]=saveAllZPointsFrom(base_msg.clusters[i], (max_z + base_msg.middle_z)/2);

                        bool dist_less_04 = false;

                        for (int k=0; k < pcz[j].points.size(); k++){      //for every point in the cluster, find min y and max y  
                            for (int n=0; n < pcz[i].points.size(); n++){

                                double dist = sqrt(pow(pcz[j].points[k].x-pcz[i].points[n].x, 2) + pow(pcz[j].points[k].y-pcz[i].points[n].y, 2) + pow(pcz[j].points[k].z-pcz[i].points[n].z, 2));

                                if(dist <= max_dist) {

                                    bool id1=false;
                                    bool id2 = false;
                                    int pos1 = -1;
                                        

                                    for(int m=0; m < msg.cluster_id.size(); m++) {
                                        if(msg.cluster_id[m] == base_msg.cluster_id[j]) id1 = true;
                                        if(msg.cluster_id[m] == base_msg.cluster_id[i]) id2 = true;

                                        if(id1==true && id2==true) break;
                                    }

                                    if(id1==true && id2==false){
                                        idss.push_back(base_msg.cluster_id[j]);
                                        idss.push_back(base_msg.cluster_id[i]);
                                        std::cout << "Clusters too close cluster_id = " << base_msg.cluster_id[j] << "cluster_id = " << base_msg.cluster_id[i] << std::endl;
                                    }
                                    else if(id1==false && id2==true){
                                        idss.push_back(base_msg.cluster_id[i]);
                                        idss.push_back(base_msg.cluster_id[j]);
                                        std::cout << "Clusters too close cluster_id = " << base_msg.cluster_id[i] << "cluster_id = " << base_msg.cluster_id[j] << std::endl;
                                    }

                                    dist_less_04 = true;
                                    break;
                                }
                            }

                            if(dist_less_04 == true) break;
                        }
                    }
                }
            }
        }
    }
}


bool checkforsameXYpoints(pcl::PointCloud<pcl::PointXYZ> pcz_max, pcl::PointCloud<pcl::PointXYZ> pcz_min){ // check if a maxz point is enclosed to minz points

   
    double xmin,xmax, ymin, ymax;

    xmin= pcz_min.points[0].x;
    xmax= pcz_min.points[0].x;
    ymin= pcz_min.points[0].y;
    ymax= pcz_min.points[0].y;

    for(int i=1; i < pcz_min.points.size(); i++){        
        if(pcz_min.points[i].x < xmin){
            xmin= pcz_min.points[i].x;
        }
        if(pcz_min.points[i].x > xmax){
            xmax= pcz_min.points[i].x;
        }
        if(pcz_min.points[i].y < ymin){
            ymin= pcz_min.points[i].y;
        }
        if(pcz_min.points[i].y > ymax){
            ymax= pcz_min.points[i].y;
        }
    }

    bool same=false;

    for(int k=0; k < pcz_max.points.size(); k++){        
        if(pcz_max.points[k].x >= xmin && pcz_max.points[k].x <= xmax && pcz_max.points[k].y >= ymin && pcz_max.points[k].y <= ymax){
            same=true;
            break;
        }
    }
    return same;
}


void checkIfClusterMove(pointcloud_msgs::PointCloud2_Segments msg, size_t size_new){  //check if a cluster moves if it does, then store it


    for(unsigned j=0; j < size_new; j++){

        bool already_exist=false;
        for(int i=0; i<clusterInMotion.size(); i++){
            if(clusterInMotion[i] == msg.cluster_id[j]){
                already_exist=true;
                break;
            }
        }
        if(already_exist==false){ 

            bool find_id=false;

            for(int i=0; i<idss.size(); i+=2){
                if (idss[i]== msg.cluster_id[j]){
                    find_id=true;
                    break;
                }
            }
            if(find_id==false){

                pcl::PointCloud<pcl::PointXYZ> pczmax;
                pcl::PointCloud<pcl::PointXYZ> pczmin;

                double max_z, min_z;
                std::pair<double,double> z_minmax;

                z_minmax = minmaxz(msg.clusters[j]);
                max_z = z_minmax.first;
                min_z = z_minmax.second;

                pczmax=saveAllZValuePoints(msg.clusters[j], max_z);
                pczmin=saveAllZValuePoints(msg.clusters[j], min_z);

                Eigen::Vector4f max_centroid;
                pcl::compute3DCentroid ( pczmax , max_centroid);

                Eigen::Vector4f min_centroid;
                pcl::compute3DCentroid ( pczmin , min_centroid);

                double disttt;

                disttt = 1000 * sqrt(pow(max_centroid[0]-min_centroid[0], 2) + pow(max_centroid[1]-min_centroid[1], 2));
                if(disttt > minMotionDist){
                    //                                double check                    //
                    pczmax=saveAllZPointsFrom(msg.clusters[j], (max_z + msg.middle_z)/2);
                    pczmin=saveAllZPointsUntil(msg.clusters[j],  (min_z + msg.middle_z)/2);
                   bool samepoints=checkforsameXYpoints(pczmax, pczmin);
                   if(samepoints==false){

                        clusterInMotion.push_back(msg.cluster_id[j]);
                        prob_extinction.push_back(false);
                        
                        std::cout << "cluster in motion id = " << msg.cluster_id[j] << "dist = " << disttt << std::endl;    
                    }
                }
            }
        }
    }
}


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

        std::cout << "!!" << std::endl;

        std::vector<Eigen::Vector4f> msg_centroid_vec;
        std::vector<Eigen::Vector4f> base_centroid_vec;

        trackedOrnotIds.clear();
        std::vector<Eigen::Vector4f> untracked_msg;

        double dist, z_maxnew;
        std::pair<double,double> z_minmax;
        bool find_id;

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
            base_centroid_vec.push_back( base_centroid );

            if(first_time==true && trackTheUntracked == true){
                centroids.push_back(base_centroid);
                ids.push_back(i);
            }
        }
        first_time = false;
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

            msg_centroid_vec.push_back( base_centroid );
            msg.cluster_id[i] = -1;
        }

        if(trackTheUntracked == true){
            for (int i=0; i < ids.size(); i++) 
                trackedOrnotIds.push_back(false);
        }

        size_t size_old = base_centroid_vec.size();
        size_t size_new = msg_centroid_vec.size();
        std::vector<std::vector<int> > dists(size_old, std::vector<int>(size_new , 10000));// TODO currently, 10000 is the maximum (2d) int distance with a 10 meter laser scanner. Initial value represents a point connected to bottom.

        for(unsigned i=0; i < size_old; i++){
            for(unsigned j=0; j < size_new; j++){
                dists[i][j] = 1000 * sqrt(pow(base_centroid_vec[i][0]-msg_centroid_vec[j][0], 2) + pow(base_centroid_vec[i][1]-msg_centroid_vec[j][1], 2));                 
            }
        }


        Hungarian::Result r = Hungarian::Solve(dists, Hungarian::MODE_MINIMIZE_COST);

        //Hungarian::PrintMatrix(r.assignment);

        dists = r.assignment;

        if(marker_flag==true) {

            marker_line.points.clear();        
            marker_sphere.points.clear();
            marker_sphere.colors.clear();
            marker_line.colors.clear();
        }

        for(unsigned j=0; j < size_new; j++){
            for(unsigned i=0; i < size_old; i++){

                if (dists[i][j] == 1){

                    dist = 1000 * sqrt(pow(base_centroid_vec[i][0]-msg_centroid_vec[j][0], 2) + pow(base_centroid_vec[i][1]-msg_centroid_vec[j][1], 2) );

                    if (dist<maxHungDist) {
                        msg.cluster_id[j] = base_msg.cluster_id[i];

                        if(trackTheUntracked == true){

                            for (unsigned m=0; m<ids.size(); m++){
                                if(ids[m] == msg.cluster_id[j]){
                                    centroids[m]=msg_centroid_vec[j];
                                    trackedOrnotIds[m] = true;
                                    break;
                                }
                            }
                        }

                        if(marker_flag==true) {

                            uint mod = msg.cluster_id[j] % 10;
                            std_msgs::ColorRGBA c;
                                
                            c.r = red[mod];
                            c.g = green[mod];
                            c.b=blue[mod];
                            c.a=0.7;

                            geometry_msgs::Point p;
                               
                            p.x = base_centroid_vec[i][0];
                            p.y = base_centroid_vec[i][1];
                            p.z = base_centroid_vec[i][2];
                                                       
                            marker_line.points.push_back(p);
                            marker_sphere.points.push_back(p);

                            geometry_msgs::Point pp;                    

                            pp.x = msg_centroid_vec[j][0];
                            pp.y = msg_centroid_vec[j][1];
                            pp.z = msg_centroid_vec[j][2];
                               
                            marker_line.points.push_back(pp);
                            marker_sphere.points.push_back(pp);

                            marker_line.colors.push_back(c);
                            marker_line.colors.push_back(c);
                            marker_sphere.colors.push_back(c);
                            c.a=0.3;
                            marker_sphere.colors.push_back(c);
                        }
                    }
                    break;
                }
            }
            if(trackTheUntracked == true && msg.cluster_id[j] == -1) untracked_msg.push_back(msg_centroid_vec[j]);
        }

        //------------------------------------------------------------------------------------//
 
        //--------------------------try to track the untracked clusters----------------------------//

        if(trackTheUntracked == true) trackUntrackedClusters(untracked_msg, msg, msg_centroid_vec, size_new );

        //----------------------------same cluster ---------------------------------------// check whether there is a new cluster over an existing one or not 

        for(int i=0; i<size_new; i++){
            if(msg.cluster_id[i] == -1){

                pcl::PointCloud<pcl::PointXYZ> pc_untracked;
                pc_untracked=saveAllPoints(msg.clusters[i]);

                for (int j = 0; j < size_new; j++){
                    if( msg.cluster_id[j] != -1){

                        pcl::PointCloud<pcl::PointXYZ> pc_existCluster;
                        pc_existCluster=saveAllPoints(msg.clusters[j]);

                       bool same_cluster = checkforsameXYpoints(pc_existCluster, pc_untracked);
                       if(same_cluster==true){

                            msg.cluster_id[i] = msg.cluster_id[j];

                            if(trackTheUntracked == true){
                                for(int n=0; n<ids.size(); n++){
                                    if(ids[n] == msg.cluster_id[i]){
                                        centroids[n] = msg_centroid_vec[i];
                                        break;
                                    }
                                }
                            }
                            std::cout << "Find same cluster id = " << msg.cluster_id[j] << std::endl;
                            break;    
                        }
                    }
                }
            }
        }

        //------------------------------------------------------------------------------------//

       
        //-----------------------distance <= max_dist------------------------------//

        checkClustersDistance(base_msg, size_old, msg);


        //----------------------------------probability for extiction------------------------------------------------------------------// check clusters which is in motion if their maxz is less than 1.5*(regular)middlez  
        
         
        for(int k=0; k<clusterInMotion.size(); k++ ) {
            for(int j=0; j < size_new; j++) {
                if(clusterInMotion[k] == msg.cluster_id[j]){
                    z_minmax = minmaxz(msg.clusters[j]);
                    z_maxnew = z_minmax.first;
                    if(z_maxnew< 3*msg.middle_z/2) prob_extinction[k]=true;
                    else prob_extinction[k]=false;
                    break;    
                }
            }     
        }

        //-------------------------------------check for new cluster------------------------------------------------------------//

        for(int j=0; j < size_new; j++) {
            if(msg.cluster_id[j] == -1){
                std::cout << "untracked!!!!!!! " << std::endl;

                for(int k=0; k<clusterInMotion.size(); k++ ) {  
                    find_id=false;
                    for(int i=0; i < size_new; i++) {
                        if(clusterInMotion[k] == msg.cluster_id[i]){
                            find_id=true;
                            if(prob_extinction[k] == true) find_id=false;

                            break;
                        }
                    }
                    if(find_id==false){    // image check needed!!
                        msg.cluster_id[j] = clusterInMotion[k];
                        prob_extinction[k]=false;
                        if(trackTheUntracked == true){
                            for(int n=0; n<ids.size(); n++){
                                if(ids[n] == clusterInMotion[k]){
                                    centroids[n] = msg_centroid_vec[j];
                                    break;
                                }
                            }
                        }
                        break;
                    }
                }
                if(msg.cluster_id[j] == -1){
                    std::cout << "New Cluster!!!! " << std::endl;
                    msg.cluster_id[j] = ++max_id;
                    if(trackTheUntracked == true){
                        centroids.push_back(msg_centroid_vec[j]);
                        ids.push_back(msg.cluster_id[j]);
                    }
                }
            }
        }

        //---------------------------------------------------------------------------------------------//

        for(int i=1; i<idss.size(); i+=2){
            for(int j=0; j<size_new; j++){
                if(idss[i]==msg.cluster_id[j]){
                    idss.erase(idss.begin()+i-1, idss.begin()+i);
                    break;
                }
            }
        }

        //------------------------------check if cluster is in movement---------------------------------//

        checkIfClusterMove(msg, size_new);

       //---------------------------------------------------------------------------------------------//
    }
};



ros::Publisher pub;
ros::Subscriber sub;
ros::Publisher marker_pub;

int size , max_id ,method;
double overlap, offset ;


std::vector<pointcloud_msgs::PointCloud2_Segments> v_;
std::vector<pointcloud_msgs::PointCloud2_Segments> new_v(2);

visualization_msgs::MarkerArray marker;
std::string marker_frame_id;


std::pair<double,double> overlap_range (const pointcloud_msgs::PointCloud2_Segments& cls){

    double z_max = 0 ;
    double height_after ,height_before ;
    double z_min = std::numeric_limits<double>::max();

    std::vector<pointcloud_msgs::PointCloud2_Segments> vec;

    vec.push_back(cls);

    if (vec.size() > size){
        vec.erase(vec.begin());
    }

    for (unsigned i=0; i < vec.size(); i++)
    {
        double offset;

        if ( i > 0 ){

            offset = ( 1.0 - overlap ) * (double)( ros::Duration( vec[i].first_stamp - vec[0].first_stamp ).toSec()) * (double)( cls.factor );

            for (unsigned j=0; j < vec[i].clusters.size(); j++)
            {
                sensor_msgs::PointCloud cloud;
                sensor_msgs::convertPointCloud2ToPointCloud( vec[i].clusters[j] , cloud );

                for (unsigned k=0; k < cloud.points.size(); k++)
                {
                    cloud.points[k].z += offset ;
                    if (cloud.points[k].z < z_min){
                        z_min = cloud.points[k].z ;
                    }
                }
            }
        }
        else {

            offset = 0.0;

            for (unsigned j=0; j < vec[0].clusters.size(); j++){
                sensor_msgs::PointCloud cloud;
                sensor_msgs::convertPointCloud2ToPointCloud( vec[0].clusters[j] , cloud );

                for ( unsigned l=0; l < cloud.points.size(); l++){
                    if ( cloud.points[l].z > z_max ){
                        z_max = cloud.points[l].z;
                    }
                }
            }
        }

    }

    return std::make_pair( z_max , z_min );

}

pointcloud_msgs::PointCloud2_Segments clusters_in_overlap (const pointcloud_msgs::PointCloud2_Segments& cl , double z_overlap_min , double z_overlap_max ){

    sensor_msgs::PointCloud2 pc1;
    pointcloud_msgs::PointCloud2_Segments output;

    for ( unsigned i=0; i < cl.clusters.size(); i++){

        sensor_msgs::PointCloud cloud;
        sensor_msgs::convertPointCloud2ToPointCloud( cl.clusters[i] , cloud );

        pcl::PCLPointCloud2 pc2 ;
        sensor_msgs::PointCloud2 cl2 ;

        sensor_msgs::convertPointCloudToPointCloud2( cloud , cl2 );
        pcl_conversions::toPCL ( cl2 , pc2 );

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2 ( pc2 , *cloud2 );
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::ExtractIndices<pcl::PointXYZ> extract ;

        for (int i=0; i < (*cloud2).size(); i++){
            pcl::PointXYZ pt(cloud2->points[i].x , cloud2->points[i].y , cloud2->points[i].z);
            if (pt.z <= z_overlap_max and pt.z >= z_overlap_min){
                inliers->indices.push_back(i);
            }
        }
        extract.setInputCloud(cloud2);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*cloud2);
        pcl::PCLPointCloud2 pcl_cloud;
        pcl::toPCLPointCloud2(*cloud2, pcl_cloud);
        pcl_conversions::fromPCL(pcl_cloud, pc1);
    
        output.clusters.push_back(pc1);
    }

    return output;
}



void callback (const pointcloud_msgs::PointCloud2_Segments& msg ){


    if(marker_flag==1) {

        marker_sphere.ns = "shapeOfsphere";
        marker_sphere.id = 0;
        marker_sphere.type = 7;
        marker_sphere.action = visualization_msgs::Marker::ADD;
        marker_sphere.pose.orientation.w = 1.0;
        marker_sphere.color.a = 1.0;

        marker_sphere.scale.x = 0.3;
        marker_sphere.scale.y = 0.3;
        marker_sphere.scale.z = 0.3;

        marker_line.ns = "shapeOfline";
        marker_line.id = 1;    
        marker_line.type = 5;
        marker_line.action = visualization_msgs::Marker::ADD;
        marker_line.pose.orientation.w = 1.0;
        marker_line.color.a = 1.0;

        marker_line.scale.x = 0.1;
        marker_line.scale.y = 0.1;
        marker_line.scale.z = 0.1;


        marker_sphere.header.frame_id = marker_frame_id;
        marker_sphere.header.stamp = msg.header.stamp;
        marker_sphere.lifetime = ros::Duration();

        marker_line.header.frame_id = marker_frame_id;
        marker_line.header.stamp = msg.header.stamp;
        marker_line.lifetime = ros::Duration();

        marker.markers.push_back(marker_sphere);
        marker.markers.push_back(marker_line);
    }



    double overlap_height_min , overlap_height_max ;

    sensor_msgs::PointCloud cloud;
    pointcloud_msgs::PointCloud2_Segments c_;

    if ( method == 1 ){
        std::pair<double,double> z_height = overlap_range(msg);
        overlap_height_min = z_height.first;
        overlap_height_max = z_height.second;
    }

    v_.push_back(msg);

    Centroid_tracking* t;

    if (v_.size() > size){
        v_.erase(v_.begin());
    }

    if(v_.size()>=size) {

        if (method == 1 ){
            new_v[0] = v_[0];
            new_v[1] = clusters_in_overlap(v_[1] , overlap_height_min , overlap_height_max);
        
            for (int i=0; i < v_[1].clusters.size(); i++) {
                new_v[1].cluster_id.push_back(i);
            }
        }
        else if ( method == 2) {
            for (int i=0; i < v_[1].clusters.size(); i++) {
                v_[1].cluster_id.push_back(i);
            }
        }
        
        for (unsigned i=0; i < v_[0].cluster_id.size(); i++){
            if (v_[0].cluster_id[i] > max_id){
                max_id = v_[0].cluster_id[i];
            }
        }
        
        if (method == 1){
            t = new Centroid_tracking( new_v[0] , max_id );
        }
        else if (method == 2 ){
            t = new Centroid_tracking( v_[0] , max_id );
        }
    }
    else {
        t = NULL;

        for (unsigned i=0; i < v_[0].clusters.size(); i++){
            v_[0].cluster_id.push_back(i);
        }

    }

    if ( t != NULL ){
        if ( method == 1 ){
            t-> track( new_v[1] );

            for (unsigned i=0; i < new_v[1].cluster_id.size(); i++){
                v_[1].cluster_id.push_back(new_v[1].cluster_id[i]);
            }
        }
        else if ( method == 2){
            t-> track( v_[1] );
        }
    }

    for (unsigned i=0; i < v_.size(); i++)
    {
        for (int k=0; k < v_[i].cluster_id.size(); k++){
            c_.clusters.push_back(v_[i].clusters[k]);
            c_.cluster_id.push_back(v_[i].cluster_id[k]);
        }
    }

    c_.header.stamp = ros::Time::now();
    c_.header.frame_id = msg.header.frame_id;
    c_.factor = msg.factor ;
    c_.overlap = msg.overlap ;
    c_.num_scans = msg.num_scans ;
    c_.first_stamp = msg.first_stamp ;
    c_.angle_min = msg.angle_min;
    c_.angle_max = msg.angle_max;
    c_.angle_increment = msg.angle_increment;
    c_.range_min = msg.range_min;
    c_.range_max = msg.range_max;
    c_.scan_time = msg.scan_time;
    c_.rec_time = msg.rec_time;
    c_.middle_z = msg.middle_z;
    

    pub.publish(c_);

    if(marker_flag==true && t != NULL) {

        marker_pub.publish(marker);

        marker.markers.clear();
    }
}


int main(int argc, char** argv){

    ros::init(argc, argv, "pointcloud2_cluster_tracking");
    ros::NodeHandle n_;

    std::string out_topic;
    std::string input_topic;
    std::string marker_topic;


    n_.param("pointcloud2_cluster_tracking/size", size , 2);
    n_.param("pointcloud2_cluster_tracking/method", method , 1);
    n_.param("pointcloud2_cluster_tracking/overlap", overlap , 0.2);
    n_.param("pointcloud2_cluster_tracking/marker_flag", marker_flag , false);
    n_.param("pointcloud2_cluster_tracking/maxHungDist", maxHungDist , 1000);
    n_.param("pointcloud2_cluster_tracking/trackTheUntracked", trackTheUntracked , false);
    n_.param("pointcloud2_cluster_tracking/minMotionDist", minMotionDist , 15.0);
    n_.param("pointcloud2_cluster_tracking/max_dist", max_dist , 0.0);

    n_.param("pointcloud2_cluster_tracking/out_topic", out_topic , std::string("/pointcloud2_cluster_tracking/clusters"));
    n_.param("pointcloud2_cluster_tracking/input_topic", input_topic , std::string("pointcloud2_clustering/clusters"));

    if(marker_flag==true) {

        n_.param("pointcloud2_cluster_tracking/marker_topic", marker_topic , std::string("visualization_marker"));
        n_.param("pointcloud2_cluster_tracking/marker_frame_id", marker_frame_id , std::string("/base_link"));
        marker_pub = n_.advertise<visualization_msgs::MarkerArray>(marker_topic, 1);
    }

    sub = n_.subscribe( input_topic, 1 , callback);
    pub = n_.advertise<pointcloud_msgs::PointCloud2_Segments>( out_topic, 1);
    
    ros::spin();
}