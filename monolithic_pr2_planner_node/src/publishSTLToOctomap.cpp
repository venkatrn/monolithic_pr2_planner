#include <sbpl_geometry_utils/Voxelizer.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32MultiArray.h>
#include <vector>
#include <boost/filesystem.hpp>
#include <leatherman/utils.h>
#include <leatherman/file.h>
#include <leatherman/binvox.h>
#include <ros/ros.h>
#include <monolithic_pr2_planner/LoggerNames.h>
#include <monolithic_pr2_planner/StatsWriter.h>
#include <cstdlib>

using namespace boost::filesystem;
using namespace std;

bool getVoxelsFromMesh(std::string resource,
        geometry_msgs::Pose pose,
        std::vector<std::vector<double> > &voxels){
    std::vector<int32_t> triangles;
    std::vector<geometry_msgs::Point> vertices, scaled_vertices;

    // get STL version of file
    std::string mesh_type = leatherman::getExtension(resource);
    std::string stl_resource = resource;
    if(!mesh_type.empty()){
        if(mesh_type.compare("dae") == 0)
            stl_resource = leatherman::replaceExtension(resource, "stl");
        ROS_DEBUG("Collada file found. Will try to use the STL version instead. (%s)",
                 resource.c_str());
        ROS_DEBUG("STL filename: %s", stl_resource.c_str());
    }

    // get triangles
    geometry_msgs::Vector3 temp_scale;
    temp_scale.x = 1.0;
    temp_scale.y = 1.0;
    temp_scale.z = 1.0;
    if(!leatherman::getMeshComponentsFromResource(stl_resource, temp_scale,
                                                  triangles, vertices))
        return false;

    // scale the vertices
    double scale = leatherman::getColladaFileScale(resource);
    leatherman::scaleVertices(vertices, scale, scale, scale, scaled_vertices);
    ROS_DEBUG("Scaled the vertices uniformly by a factor of %0.3f.", scale);

    // voxelize!
    sbpl::Voxelizer::voxelizeMesh(scaled_vertices, triangles, pose, 0.02, voxels, false);

    return true;
}

void addCuboid(pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud, double X, double Y, double Z, double dimX, double dimY, double dimZ, bool fill = true){
    // Add a cuboid of size dimX, dimY, dimZ, with the bottom left being the
    // reference point: X, Y, Z
    // double dimX, dimY, dimZ;
    // double X, Y, Z;

    std::vector<std::vector<double> > voxels;
    sbpl::Voxelizer::voxelizeBox(dimX, dimY, dimZ, 0.02, voxels, fill);
    size_t currentPCLCloudSize = pclCloud->points.size();
    pclCloud->points.resize(currentPCLCloudSize + voxels.size());
    for (size_t i = 0; i < voxels.size(); ++i)
    {
        pclCloud->points[currentPCLCloudSize + i].x = voxels[i][0] + (X +
            dimX/2);
        pclCloud->points[currentPCLCloudSize + i].y = voxels[i][1] + (Y +
            dimY/2);
        pclCloud->points[currentPCLCloudSize + i].z = voxels[i][2] + (Z +
            dimZ/2);
    }
}

void addRandomObstacles(pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud, int
    numSurfaces, int numObstaclesPerSurface, unsigned int seed){
    ros::NodeHandle nh;

    // Add the surface - these are generated only within these bounds.
    double surfaceBoundsXMin = 6.2;
    double surfaceBoundsXMax = 7;

    double surfaceBoundsYMin = 0.8;
    double surfaceBoundsYMax = 3.5;

    double surfaceSizeXMax = 0.65;
    double surfaceSizeXMin = 0.65;

    double surfaceSizeYMax = 1.3;
    double surfaceSizeYMin = 0.65;

    double surfaceSizeZ = 0.8;


    double obstacleBoundsXMin = 3.5;
    double obstacleBoundsXMax = 6;

    double obstacleBoundsYMin = 0;
    double obstacleBoundsYMax = 4;

    double obstacleSizeXMax = 0.3;
    double obstacleSizeXMin = 0.1;

    double obstacleSizeYMax = 0.3;
    double obstacleSizeYMin = 0.1;

    double obstacleSizeZMin = 0.1;
    double obstacleSizeZMax = 0.4;

    nh.setParam("/monolithic_pr2_planner_node/experiments/number_of_regions",
        numSurfaces);

    nh.setParam("/monolithic_pr2_planner_node/experiments/seed", int(seed));
    srand(seed);

    for (int i = 0, j = 0; i < numSurfaces; ++i, j+=2){

        // Generate position
        double X = surfaceBoundsXMin + static_cast<double>(rand())/RAND_MAX *
        (surfaceBoundsXMax - surfaceBoundsXMin);
        double Y = surfaceBoundsYMin + j*(surfaceBoundsYMax - surfaceBoundsYMin)/(numSurfaces+1) + static_cast<double>(rand())/RAND_MAX *
        (surfaceBoundsYMax - surfaceBoundsYMin)/(numSurfaces+1);

        double Z = 0.0;

        // Generate size
        double dimX = surfaceSizeXMin + static_cast<double>(rand())/RAND_MAX *
        (surfaceSizeXMax - surfaceSizeXMin);

        double dimY = surfaceSizeYMin + static_cast<double>(rand())/RAND_MAX *
        (surfaceSizeYMax - surfaceSizeYMin);

        double dimZ = surfaceSizeZ;

        // Add to Param server (not the best thing to do, but whatever)
        nh.setParam("/monolithic_pr2_planner_node/experiments/goal_region_x_" + std::to_string(i), X);
        nh.setParam("/monolithic_pr2_planner_node/experiments/goal_region_y_" + std::to_string(i), Y);
        nh.setParam("/monolithic_pr2_planner_node/experiments/goal_region_z_" + std::to_string(i), Z);
        nh.setParam("/monolithic_pr2_planner_node/experiments/goal_region_dimx_" + std::to_string(i),
            dimX);
        nh.setParam("/monolithic_pr2_planner_node/experiments/goal_region_dimy_" + std::to_string(i),
            dimY);
        nh.setParam("/monolithic_pr2_planner_node/experiments/goal_region_dimz_" + std::to_string(i),
            dimZ);

        // Now, we can add this surface.
        addCuboid(pclCloud, X, Y, Z, dimX, dimY, dimZ, false);

        obstacleBoundsXMin = X;
        obstacleBoundsXMax = X + (dimX - obstacleSizeXMax);

        obstacleBoundsYMin = Y;
        obstacleBoundsYMax = Y + (dimY - obstacleSizeYMax);

        // Add the obstacles on the surface
        for (int j = 0; j < numObstaclesPerSurface; ++j){
            double oX = obstacleBoundsXMin + static_cast<double>(rand())/RAND_MAX *
            (obstacleBoundsXMax - obstacleBoundsXMin);
            double oY = obstacleBoundsYMin + static_cast<double>(rand())/RAND_MAX *
            (obstacleBoundsYMax - obstacleBoundsYMin);

            double oZ = dimZ;

            // Generate size
            double odimX = obstacleSizeXMin + static_cast<double>(rand())/RAND_MAX *
            (obstacleSizeXMax - obstacleSizeXMin);

            double odimY = obstacleSizeYMin + static_cast<double>(rand())/RAND_MAX *
            (obstacleSizeYMax - obstacleSizeYMin);

            double odimZ = obstacleSizeZMin + static_cast<double>(rand())/RAND_MAX *
            (obstacleSizeZMax - obstacleSizeZMin);

            addCuboid(pclCloud, oX, oY, oZ, odimX, odimY, odimZ, true);
        }
    }


}

void addStartStateRegionToParamServer(){
    // Sets the start state regions to the param server for experiments.
    // Note: Setting only one start region for now.
    ros::NodeHandle nh;
    double X = 0.5;
    double Y = 0.5;
    double Z = 0.1;
    double dimX = 8;
    double dimY = 5;
    double dimZ = 0.0;
    nh.setParam("/monolithic_pr2_planner_node/experiments/start_region_x", X);
    nh.setParam("/monolithic_pr2_planner_node/experiments/start_region_y", Y);
    nh.setParam("/monolithic_pr2_planner_node/experiments/start_region_z", Z);
    nh.setParam("/monolithic_pr2_planner_node/experiments/start_region_dimx",
        dimX);
    nh.setParam("/monolithic_pr2_planner_node/experiments/start_region_dimy",
        dimY);
    nh.setParam("/monolithic_pr2_planner_node/experiments/start_region_dimz",
        dimZ);
}

vector<Eigen::Vector3d> getVoxelsFromFile(std::string filename){
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_environment", 1);
    path p(filename);
    if (!exists(p)){
        ROS_ERROR("Couldn't find %s to load in the collision map!",
                  filename.c_str());
    }

    ROS_DEBUG("Reading stl file %s",filename.c_str());

    geometry_msgs::Pose pose;
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = 0;
    pose.orientation.w = 1;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    std::vector<std::vector<double> > voxels;
    getVoxelsFromMesh(filename, pose, voxels);

    std::vector<Eigen::Vector3d> points;
    double maxx = -1;
    double maxy = -1;
    double maxz = -1;
    double minx = 100000;
    double miny = 100000;
    double minz = 100000;
    for(unsigned int i=0; i<voxels.size(); i++){
        if(voxels[i][0]>maxx)
            maxx = voxels[i][0];
        if(voxels[i][1]>maxy)
            maxy = voxels[i][1];
        if(voxels[i][2]>maxz)
            maxz = voxels[i][2];
        if(voxels[i][0]<minx)
            minx = voxels[i][0];
        if(voxels[i][1]<miny)
            miny = voxels[i][1];
        if(voxels[i][2]<minz)
            minz = voxels[i][2];
    }
    for(unsigned int i=0; i<voxels.size(); i++){
        //TODO: this offset should be read in from somewhere!
        Eigen::Vector3d p(voxels[i][0]-minx,
                voxels[i][1]-miny,
                voxels[i][2]-minz-0.32);
        points.push_back(p);
    }

    //mapsize_x = maxx-minx;
    //mapsize_y = maxy-miny;

    ROS_DEBUG("bounds (%f, %f, %f) to (%f, %f, %f)",
            minx,miny,minz,maxx,maxy,maxz);

    //make a point cloud for visualization
    ROS_INFO("visualizing point cloud of size %lu", points.size());
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pclCloud->points.resize(points.size());
    for(unsigned int i=0; i<points.size(); i++){
        pclCloud->points[i].x = points[i][0];
        pclCloud->points[i].y = points[i][1];
        pclCloud->points[i].z = points[i][2];
    }
    // addCuboid(pclCloud, 5.248199056716729, 1.7879581496994748, 0.0, 1.4, 1.4, 0.8,true);
    // addCuboid(pclCloud, 5.248199056716729, 2.5379581496994748, 0.8, 0.1, 0.4,
    //     0.15,true);
    // addCuboid(pclCloud, 5.348199056716729, 2.1079581496994748, 0.8, 0.1, 0.2, 0.3,true);

    // Doorway
    addCuboid(pclCloud, 5, 0, 0, 0.05, 2, 1.8, true);
    addCuboid(pclCloud, 5, 2.85, 0, 0.05, 2.2, 1.8, true);
    addCuboid(pclCloud, 5, 2.85, 0, 1.00, 0.05, 1.8, true);
    // head of doorway; don't add this.
    // addCuboid(pclCloud, 5, 2, 1.6, 0.05, 1, 1.8, true);

    // Small table
    addCuboid(pclCloud, 7.8, 2.3, 0, 0.4, 0.4, 0.8, true);


    bool addTableObstacles;
    ph.param("addTableObstacles",addTableObstacles,false);

    if(addTableObstacles){
      bool randomizeTableObstacles;
      ph.param("randomizeTableObstacles",randomizeTableObstacles,true);
      if(!randomizeTableObstacles){
        string pathToTableObstacleParamFile;
        ph.param<string>("pathToTableObstacleParamFile",pathToTableObstacleParamFile,"");
        if(pathToTableObstacleParamFile.empty()){
          ROS_ERROR("rosparam randomizeTableObstacles was set to false, but then pathToTableObstacleParamFile was not set!");
          exit(1);
        }
        FILE* fin = fopen(pathToTableObstacleParamFile.c_str(), "r");
        if(!fin){
          ROS_ERROR("pathToTableObstacleParamFile did not lead to a file");
          exit(1);
        }
        unsigned int seed;
        int numSurfaces, numObstaclesPerSurface;
        bool success = true;
        success &= fscanf(fin,"randomSeed: %u\n",&seed) == 1;
        success &= fscanf(fin,"numSurfaces: %d\n",&numSurfaces) == 1;
        success &= fscanf(fin,"numObstaclesPerSurface: %d\n",&numObstaclesPerSurface) == 1;
        if(!success){
          ROS_ERROR("table obstacle param file formatted incorrectly");
          exit(1);
        }
        addRandomObstacles(pclCloud, numSurfaces, numObstaclesPerSurface, seed);
        ROS_WARN("loaded environment from file (seed=%d) %d surfaces and %d obstacles on each",seed,numSurfaces,numObstaclesPerSurface);
      }
      else{
        int numSurfaces = 2;
        int numObstaclesPerSurface = 5;
        unsigned int seed = clock();
        ROS_WARN("generating random environment (seed=%d) with %d surfaces and %d obstacles on each",seed,numSurfaces,numObstaclesPerSurface);
        addRandomObstacles(pclCloud, numSurfaces, numObstaclesPerSurface, seed);
        ROS_WARN("writing tableObstacles.yaml file!");
        FILE* fout = fopen("tableObstacles.yaml", "w");
        fprintf(fout,"randomSeed: %u\n",seed);
        fprintf(fout,"numSurfaces: %d\n",numSurfaces);
        fprintf(fout,"numObstaclesPerSurface: %d\n",numObstaclesPerSurface);
        fclose(fout);
      }
    }

    addStartStateRegionToParamServer();

    sensor_msgs::PointCloud2 pc;
    pcl::toROSMsg (*pclCloud, pc);
    pc.header.frame_id = "/map";
    pc.header.stamp = ros::Time::now();
    sleep(1);
    pcl_pub.publish(pc);

    return points;
}


bool isValidPath(std::string filename){
    path input_path(filename.c_str());
    if (exists(input_path)){
        ROS_DEBUG("Pulling in data from %s", filename.c_str());
        return true;
    } else {
        ROS_ERROR("Failed to find file '%s'", filename.c_str());
        return false;
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "stl_to_octomap");
    if (argc != 2){
        ROS_ERROR("missing filename for map!");
        exit(1);
    }
    //if (isValidPath(string(argv[1]))){
        getVoxelsFromFile(string(argv[1]));
    //}
    return 0;
}
