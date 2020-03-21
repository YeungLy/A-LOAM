#ifndef POLYGON_INTERSECT_H
#define POLYGON_INTERSECT_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>

class Line 
{
    public:
    Line(Eigen::Vector2d v1, Eigen::Vector2d v2)
    {
        a = v2.y() - v1.y();
        b = v1.x() - v2.x();
        c = v2.x() * v1.y() - v1.x() * v2.y(); //v2.cross(v1)
    }
    double operator() (Eigen::Vector2d p)
    {
        //it is direction specified. same point, opposite direction line has oppsite symbol.
        //e.g. v1:(0.0,  1.0), v2( 1.0, 0.0) => l1 : -x+y+1=0, direction is from v1 to v2, pointing down.
        //     v1:(0.0, -1.0), v2(-1.0, 0.0) => l2 : -x-y+1=0, direction is from v1 to v2, pointing up.
        //     p: (0.0, 3.0), the point is above l1 and l2, but l1(p) = -2 < 0, l2(p) = 4 > 0,
        //     in clockwise rectangle edge, la, lb, lc, ld, the point p is inside of this rectangle, means all the l(p) should be positive.
        return a * p.x() + b * p.y() + c;
    }
    bool intersect(Line other, Eigen::Vector2d & inter_p)
    {
        //using homogeneouse coordinate find two line's intersection point
        double w = a * other.b - b * other.a;
        if (w == 0)
            return false;
        inter_p << (b * other.c - c *other.b) / w, (c * other.a - a * other.c) / w; 
        return true;

    }
    double a, b, c;
};

void test_line()
{
    Eigen::Vector2d v1(0.0, 1.0);
    Eigen::Vector2d v2(1.0, 0.0);
    Line l(v1, v2);
    Eigen::Vector2d p(0.0, 3.0);
    std::cout << "a: " << l.a << ", b: " << l.b <<", c: " << l.c << std::endl;
    std::cout << l(p) <<std::endl;

    v1 << 0.0, -1.0;
    v2 << -1.0, 0.0;
    Line l2(v1, v2);
    std::cout << "a: " << l2.a << ", b: " << l2.b <<", c: " << l2.c << std::endl;
    std::cout << l2(p) << std::endl;


}

double intersect_area(Eigen::Matrix<double, 2, 4> rec1, Eigen::Matrix<double, 2, 4> rec2)
{
    test_line();

    std::vector<Eigen::Vector2d> vertexes;
    //initiliaze vertexes with all rec1 corners
    for (int i = 0; i < 4; ++i)
        vertexes.push_back(rec1.col(i));
    //find each edge of rec2 intersect with rec1, 
    for (int p = 0; p < 4; ++p)
    {
        //no intersection
        if (vertexes.size() < 3)
            break;
        int q = (p + 1) % 4;
        Line line(rec2.col(p), rec2.col(q));
        std::vector<Eigen::Vector2d> new_vertexes;
        for (int s = 0; s < vertexes.size(); ++s)
        {
            int t = ( s + 1 ) % vertexes.size();
            // >= 0 means inside of this clockwise rec. 
            if (line(vertexes[s]) >= 0)
            {
                //std::cout << "new vertexes inside of this line: " << vertexes[s] << std::endl;
                new_vertexes.push_back(vertexes[s]);
            }
            if (line(vertexes[s]) * line(vertexes[t]) < 0)
            {
                //line_pq intersect with edge_st.
                Eigen::Vector2d inter_p;
                Line st(vertexes[s], vertexes[t]);
                line.intersect(st, inter_p);
                //std::cout << "intersect! " << inter_p << std::endl;
                new_vertexes.push_back(inter_p);
            }
        }
        vertexes = new_vertexes;
        
    }
    //area
    if (vertexes.size() < 3)
        return 0;
    double area = 0.0;
    for (int s = 0; s < vertexes.size(); ++s)
    {
       int t = (s + 1) % vertexes.size();
       area += vertexes[s].x() * vertexes[t].y() - vertexes[s].y() *vertexes[t].x();
    }
    area = 0.5*std::abs(area);
    return area;
}

#endif
