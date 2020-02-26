#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include "tracklet.h"
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
//IOU computation.. 
//maybe matrix cooperation can use eigen

std::vector< std::vector<double> > CalculateIoU3d(std::vector<double> iBoxes, std::vector<double> jBoxes)
{
    std::vector< std::vector<double> > dist;
    for (size_t i = 0; i < iBoxes.size(); ++i)
    {
        std::vector<double> dist_row;
        for (size_t j = 0; j < jBoxes.size(); ++j)
        {
            
            dist_row[j] = 1 - 0.02*i + 0.15*j;
        }
        dist.push_back(dist_row);
    }
    return dist;
}

std::vector<DetectedBox> RosMsgToBoxes(jsk_recognition_msgs::BoundingBoxArrayConstPtr boxes_msg)
{

    std::vector<DetectedBox> boxes;
    for (size_t i = 0; i < boxesCurr->boxes.size(); ++i)
    {

    }
    return boxes;
}

//box3d to 8corners

//8corners to box3d

#endif