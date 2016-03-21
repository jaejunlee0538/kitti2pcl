//
// Created by ub1404 on 16. 3. 21.
//

#ifndef KITTI2PCL_READKITTI_H
#define KITTI2PCL_READKITTI_H
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/filesystem.hpp>

typedef pcl::PointCloud<pcl::PointXYZI> CloudXYZI;

void readKittiVelodyne(const boost::filesystem::path& fileName, CloudXYZI& cloud){
    std::ifstream input(fileName.c_str(), std::ios_base::binary);
    if(!input.good()){
        std::cerr<<"Cannot open file : "<<fileName<<std::endl;
        return;
    }

    cloud.clear();
    cloud.height = 1;

    for (int i=0; input.good() && !input.eof(); i++) {
        pcl::PointXYZI point;
        input.read((char *) &point.x, 3*sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));
        cloud.push_back(point);
    }
    std::cerr<<fileName.filename()<<":"<<cloud.width<<" points"<<std::endl;
    input.close();
}


#endif //KITTI2PCL_READKITTI_H
