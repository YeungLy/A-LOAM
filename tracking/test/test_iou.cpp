#include "iou.h"

int main(int argc, char * argv[])
{

    // test case 1
    /* 
    DetectedBox a1(0.0, 1.0, 0.0, 3.0, 4.0, 2., 0.);
    DetectedBox b1(0.0, 0.0, 0.0, 4.0, 3.0, 1., 0.);
    
    double iou1 = BoxIoUBev(a1, b1);
    
    //output:
    //       vertexes: (1.5, -1), (-1.5, -1), (-1.5, 1.5), (1.5, 1.5)
    //       intersect = 7.5, union = 24-7.5, iou = 0.454545
    */

    //test case 2
    DetectedBox a2(0.0, 1.0, 0.0, 3.0, 4.0, 2., 0.);
    DetectedBox b2(0.0, 0.0, 0.0, 4.0, 4.0, 1., M_PI/4.);
    double iou2 = BoxIoUBev(a2, b2);
    
    /*
    //output:
    //       intersect = 9.23528, union = 28-intersect, iou = 0.492162
    */

    //test case 3, for repeat vertexes
    DetectedBox a3(0.0, 0.0, 0.0, 4.0, 4.0, 2., 0.);
    DetectedBox b3(0.0, 0.0, 0.0, 2.0*1.414, 2.0*1.414, 1., M_PI/4.);
    double iou3 = BoxIoUBev(a3, b3);
    
    /*
    //output:
    //       intersect = 7.99758, union = 28-intersect, iou = 0.498849
    */
    
    //test case 4, for non intersection
    DetectedBox a4(0.0, 0.0, 0.0, 4.0, 4.0, 2., 0.);
    DetectedBox b4(8.0, 5.0, 0.0, 2.0*1.414, 2.0*1.414, 1., M_PI/4.);
    double iou4 = BoxIoUBev(a4, b4);
    /*
    */

    //DetectedBox a5(32.547063, 8.547366, -1.437311, 3.874000, 1.648000, 1.43, -3.1389);
    //DetectedBox b5(32.546772, 8.546250, -1.149843, 4.020460, 1.603115, 1.447647, -2.989162);



    //double iou5 = BoxIoUBev(a5, b5);



    return 0;
}
