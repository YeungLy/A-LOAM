#include "iou.h"

int main(int argc, char * argv[])
{

    DetectedBox a(0.0, 1.0, 0.0, 3.0, 4.0, 2., 0.);
    DetectedBox b(0.0, 0.0, 0.0, 4.0, 3.0, 1., 0.);
    
    double iou = BoxIoUBev(a, b);

    return 0;
}
