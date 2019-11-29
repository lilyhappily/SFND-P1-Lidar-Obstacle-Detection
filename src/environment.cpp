/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // Create lidar sensor
    Lidar * lidar =  new Lidar(cars, 0.0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    // renderRays(viewer, lidar->position, inputCloud);
    // renderPointCloud(viewer, inputCloud, "inputCloud", Color(1, 0, 0));
    // renderPointCloud(viewer, inputCloud, "inputCloud");

    // Create point processor
    // on the heap
    ProcessPointClouds<pcl::PointXYZ> * cloudProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    // on the stack
    // ProcessPointClouds<pcl::PointXYZ> cloudProcessor;
    std::pair< pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr > segmentCloud = cloudProcessor->SegmentPlane(inputCloud, 100, 0.2);
    //renderPointCloud(viewer, segmentCloud.first, "planeCloud", Color(0, 1, 0));
    //renderPointCloud(viewer, segmentCloud.second, "obstCloud", Color(1, 0, 0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters= cloudProcessor->Clustering(segmentCloud.second, 1.0, 3, 30);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};

    bool render_clusters = true;
    bool render_boxes = true;
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        if(render_clusters)
        {
            std::cout << "Cluster Size: ";
            cloudProcessor->numPoints(cluster);
            renderPointCloud(viewer, cluster, "ObstCloud " + std::to_string(clusterId), colors[clusterId]);
        }

        if(render_boxes)
        {
            Box box = cloudProcessor->BoundingBox(cluster);
            renderBox(viewer, box, clusterId);
            ++clusterId;
        }

    }

}

void CityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> * pointProcessI, pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud)
{
    // ProcessPointClouds<pcl::PointXYZI> * pointProcessI = new ProcessPointClouds<pcl::PointXYZI>();
    // pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    // filter
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFiltered = pointProcessI->FilterCloud(inputCloud, 0.3, Eigen::Vector4f (-10, -5, -2, 1), Eigen::Vector4f (30, 8, 1, 1));
    // segment
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessI->SegmentPlane(cloudFiltered, 50, 0.3);
    renderPointCloud(viewer, segmentCloud.first, "palneCloud", Color(0, 1, 0));
    renderPointCloud(viewer, segmentCloud.second, "obstCloud", Color(1, 0, 0));

    // cluster 
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessI->Clustering(segmentCloud.second, 0.8, 10, 500);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0),Color(1, 1, 0), Color(0, 0, 1)};
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "Cluster Size: ";
        pointProcessI->numPoints(cluster);
        renderPointCloud(viewer, cluster, "ObstCloud" + std::to_string(clusterId), colors[clusterId%colors.size()]);
        
        Box box = pointProcessI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }


    // //render the car roof box
    // Box roofBox;
    // roofBox.x_max = pointProcessI->maxPointRoof[0];
    // roofBox.y_max = pointProcessI->maxPointRoof[1];
    // roofBox.z_max = pointProcessI->maxPointRoof[2];

    // roofBox.x_min = pointProcessI->minPointRoof[0];
    // roofBox.y_min = pointProcessI->minPointRoof[1];
    // roofBox.z_min = pointProcessI->minPointRoof[2];

    // renderBox(viewer, roofBox, 1, Color(0.5,0.,0.5));


}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    // simpleHighway(viewer);
    // CityBlock(viewer);
    ProcessPointClouds<pcl::PointXYZI> * pointProcessI = new ProcessPointClouds<pcl::PointXYZI>;
    std::vector<boost::filesystem::path> stream = pointProcessI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        inputCloudI = pointProcessI->loadPcd((*streamIterator).string());
        // renderPointCloud(viewer,inputCloudI, "raw");
        CityBlock(viewer, pointProcessI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    } 
}