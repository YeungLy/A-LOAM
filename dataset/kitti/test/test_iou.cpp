#include "box.h"
#include <iostream>

typedef kitti::Box3D DetectedBox;

int main(int argc, char * argv[])
{

    // test case 1
    /* 
*/
    DetectedBox a1(0.0, 1.0, 0.0, 3.0, 4.0, 2., 0.);
    DetectedBox b1(0.0, 0.0, 0.0, 4.0, 3.0, 1., 0.);
    
    double iou1 = a1.iou(b1);
    
    //output:
    //       vertexes: (1.5, -1), (-1.5, -1), (-1.5, 1.5), (1.5, 1.5)
    //       intersect = 7.5, union = 24-7.5, iou = 0.454545

    //test case 2
//    DetectedBox a2(0.0, 1.0, 0.0, 3.0, 4.0, 2., 0.);
//    DetectedBox b2(0.0, 0.0, 0.0, 4.0, 4.0, 1., M_PI/4.);
//    double iou2 = a2.iou(b2);
    
    //output:
    //       intersect = 9.23528, union = 28-intersect, iou = 0.492162

    //test case 3, for repeat vertexes
    DetectedBox a3(0.0, 0.0, 0.0, 4.0, 4.0, 2., 0.);
    DetectedBox b3(0.0, 0.0, 0.0, 2.0*1.414, 2.0*1.414, 1., M_PI/4.);
    double iou3 = a3.iou(b3);
    
    //output:
    //       intersect = 7.99758, union = 28-intersect, iou = 0.498849
    
    //test case 4, for non intersection
    DetectedBox a4(0.0, 0.0, 0.0, 4.0, 4.0, 2., 0.);
    DetectedBox b4(8.0, 5.0, 0.0, 2.0*1.414, 2.0*1.414, 1., M_PI/4.);
    double iou4 = a4.iou(b4);

    //DetectedBox a5(32.547063, 8.547366, -1.437311, 3.874000, 1.648000, 1.43, -3.1389);
    //DetectedBox b5(32.546772, 8.546250, -1.149843, 4.020460, 1.603115, 1.447647, -2.989162);



    //double iou5 = a5.iou(b5);

    

    DetectedBox a6(15.209687, 5.867436, -1.638654, 4.261270, 1.697110, 1.445860, -3.131309);
    DetectedBox b6(15.225699, 5.886409, -1.671138, 4.173050, 1.721551, 1.453356, -3.126798);
    std::cout << b6.bev_corners() << std::endl;

    std::cout << a6.iou(b6) << std::endl;

    return 0;
}
