#include <KrisLibrary/geometry/TSDFReconstruction.h>
#include <KrisLibrary/math3d/AABB3D.h>
#include <KrisLibrary/meshing/IO.h>
#include <KrisLibrary/meshing/TriMeshOperators.h>
#include <KrisLibrary/utils/ioutils.h>
#include <KrisLibrary/utils.h>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <ros/ros.h>
#include <shape_msgs/Mesh.h>
#include <geometry_msgs/Point.h>
#include <shape_msgs/MeshTriangle.h>
#include <rviz_visual_tools/rviz_visual_tools.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_listener.h>
#include <chrono>
#include <cinttypes>
using namespace Geometry;
using namespace Meshing;
using namespace Math3D;

#define CELL_SIZE 0.05
#define TRUNCATION_DISTANCE -0.1

rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
tf2_ros::Buffer tfBuffer;
ros::Publisher meshPub;
std::shared_ptr<tf2_ros::TransformListener> tfListener;
bool glbalMesh;
template <typename UNIT>
inline int64_t get_system_timestamp() {
  const auto tp = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration_cast<UNIT>(tp).count();
}

void tsdfCallback(const std_msgs::Float32MultiArray::Ptr& msg)
{
    static bool init = false;
    static Eigen::Isometry3d pose;
    static geometry_msgs::TransformStamped transformStamped;
    if(!init){
        try{
        transformStamped = tfBuffer.lookupTransform("world", "slam",
                                ros::Time(0));
        pose = tf2::transformToEigen(transformStamped);

        }
        catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        }
        init = true;
    }

    auto values = msg->data;
    ROS_INFO("I heard tsdf of size: ", msg->data.size());
    const auto st = get_system_timestamp<std::chrono::milliseconds>();  // nsec

    int numPoints = msg->data.size()/4;
    float minValue = 1e100, maxValue = -1e100;
    AABB3D bbox;
    int k=0;
    for(int i=0;i<numPoints;i++,k+=4) {
        minValue = Min(minValue,values[k+3]);
        maxValue = Max(maxValue,values[k+3]);
        bbox.expand(Vector3(values[k],values[k+1],values[k+2]));
    }
    printf("Read %d points with distance in range [%g,%g]\n",numPoints,minValue,maxValue);
    printf("   x range [%g,%g]\n",bbox.bmin.x,bbox.bmax.x);
    printf("   y range [%g,%g]\n",bbox.bmin.y,bbox.bmax.y);
    printf("   z range [%g,%g]\n",bbox.bmin.z,bbox.bmax.z);
    float truncation_distance = TRUNCATION_DISTANCE;
    if(TRUNCATION_DISTANCE < 0) {
        //auto-detect truncation distance
        truncation_distance = Max(-minValue,maxValue)*0.99;
        printf("Auto-detected truncation distance %g\n",truncation_distance);
    }
    printf("Using cell size %g\n",CELL_SIZE);
    SparseTSDFReconstruction tsdf(Vector3(CELL_SIZE),truncation_distance);
    tsdf.tsdf.defaultValue[0] = truncation_distance;
    k=0;
    Vector3 ofs(CELL_SIZE*0.5);
    for(int i=0;i<numPoints;i++,k+=4) {
        tsdf.tsdf.SetValue(Vector3(values[k],values[k+1],values[k+2])+ofs,values[k+3]);
    }

    printf("Extracting mesh\n");
    TriMesh mesh;
    tsdf.ExtractMesh(mesh);
    std::cout<<"Before Merge: vertsSize: "<<mesh.verts.size()<<std::endl;
    std::cout<<"Before Merge: trisSize: "<<mesh.tris.size()<<std::endl;

    MergeVertices(mesh, 0.05);

    int vertsSize = mesh.verts.size();
    int trisSize = mesh.tris.size();

    std::cout<<"vertsSize: "<<vertsSize<<std::endl;
    std::cout<<"trisSize: "<<trisSize<<std::endl;
    const auto end = get_system_timestamp<std::chrono::milliseconds>();
    std::cout<<"mesh processing time: "<<end-st<<" ms"<<std::endl;

    const auto st_msg = get_system_timestamp<std::chrono::milliseconds>();  
    shape_msgs::Mesh::Ptr mMeshMsg = boost::make_shared<shape_msgs::Mesh>();
    // geometry_msgs/Point[] 
    mMeshMsg->vertices.resize(vertsSize);

    // // // shape_msgs/MeshTriangle[] 
    mMeshMsg->triangles.resize(trisSize);
    

    for(int i = 0; i < vertsSize; i++){
        mMeshMsg->vertices[i].x = mesh.verts[i].x;
        mMeshMsg->vertices[i].y = mesh.verts[i].y;
        mMeshMsg->vertices[i].z = mesh.verts[i].z;
        // std::cout<<mesh.verts[i].x<<std::endl;
    }

    for(int i = 0; i < trisSize; i++){
        mMeshMsg->triangles[i].vertex_indices[0] = mesh.tris[i].a;
        mMeshMsg->triangles[i].vertex_indices[1] = mesh.tris[i].b;
        mMeshMsg->triangles[i].vertex_indices[2] = mesh.tris[i].c;
        // std::cout<<mesh.tris[i].a<<std::endl;
    }
    meshPub.publish(mMeshMsg);
    // Eigen::Isometry3d pose;
    // pose = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()); // rotate along X axis by 45 degrees
    // pose.translation() = Eigen::Vector3d( 0, 0, 0 ); // translate x,y,z
    // Publish arrow vector of pose
    visual_tools_->publishMesh(pose, *mMeshMsg, rviz_visual_tools::ORANGE, 1, "mesh", 1); // rviz_visual_tools::TRANSLUCENT_LIGHT

    const auto end_msg = get_system_timestamp<std::chrono::milliseconds>();  
    std::cout<<"Maker msg processing time: "<<end_msg-st_msg<<" ms"<<std::endl;


    // Don't forget to trigger the publisher!
    visual_tools_->trigger();


}

using namespace Geometry;
using namespace Meshing;
using namespace Math3D;
using namespace std;

#define CELL_SIZE 0.01
#define TRUNCATION_DISTANCE -1


int main(int argc, char** argv) {
    ros::init(argc, argv, "tsdf_node");
    ros::NodeHandle nh;
    nh.param("/tsdf2mesh/global_mesh", glbalMesh, true);

    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world","/mesh_reconst", nh));
    visual_tools_->setPsychedelicMode(false);
    visual_tools_->loadMarkerPub();

    // visual_tools_->enableBatchPublishing(false);
    tfListener= std::make_shared<tf2_ros::TransformListener>(tfBuffer);
    meshPub = nh.advertise<shape_msgs::Mesh>("/mesh", 1);
    ros::Subscriber subTsdf;
    
    if(glbalMesh){
        subTsdf = nh.subscribe("/tsdf_global", 1, tsdfCallback);
        std::cout<<"realtime global tsdf"<<std::endl;
    }
    else{
        subTsdf = nh.subscribe("/tsdf_local", 1, tsdfCallback);
        std::cout<<"realtime local tsdf"<<std::endl;
    }

    ros::spin();
    return 0;
}
