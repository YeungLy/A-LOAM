#include "iou.h"

int main(int argc, char * argv[])
{

    /* 
    // test case 1
    DetectedBox a(0.0, 1.0, 0.0, 3.0, 4.0, 2., 0.);
    DetectedBox b(0.0, 0.0, 0.0, 4.0, 3.0, 1., 0.);
    
    //output:
    //       vertexes: (1.5, -1), (-1.5, -1), (-1.5, 1.5), (1.5, 1.5)
    //       intersect = 7.5, union = 24-7.5, iou = 0.454545
    */

    /*
    //test case 2
    DetectedBox a(0.0, 1.0, 0.0, 3.0, 4.0, 2., 0.);
    DetectedBox b(0.0, 0.0, 0.0, 4.0, 4.0, 1., M_PI/4.);
    
    //output:
    //       intersect = 9.23528, union = 28-intersect, iou = 0.492162
    */

    /*
    //test case 3, for repeat vertexes
    DetectedBox a(0.0, 0.0, 0.0, 4.0, 4.0, 2., 0.);
    DetectedBox b(0.0, 0.0, 0.0, 2.0*1.414, 2.0*1.414, 1., M_PI/4.);
    
    //output:
    //       intersect = 7.99758, union = 28-intersect, iou = 0.498849
    */
    //test case 4, for non intersection
    DetectedBox a(0.0, 0.0, 0.0, 4.0, 4.0, 2., 0.);
    DetectedBox b(8.0, 5.0, 0.0, 2.0*1.414, 2.0*1.414, 1., M_PI/4.);

    double iou = BoxIoUBev(a, b);



    return 0;
}
