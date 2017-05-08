#ifndef CGAL_TOOLS_H_
#define CGAL_TOOLS_H_

#include <CGAL/Cartesian.h>
#include <iostream>
#include <geometry_msgs/Point32.h>
#include <math.h>

typedef CGAL::Cartesian<double>     Kernel;
typedef Kernel::Line_2              Line_2;
typedef Kernel::Line_3              Line_3;
typedef Kernel::Point_2             Point_2;
typedef Kernel::Point_3             Point_3;
typedef Kernel::Triangle_2          Triangle_2;
typedef Kernel::Triangle_3          Triangle_3;
typedef CGAL::Cartesian<double>     Kernel;
typedef Kernel::Intersect_3         Intersect_3;

bool doTriangleInteresect2(std::vector<geometry_msgs::Point32>& vec1, std::vector<geometry_msgs::Point32>& vec2);

bool doTriangleInteresect3(std::vector<geometry_msgs::Point32>& vec1, std::vector<geometry_msgs::Point32>& vec2);

geometry_msgs::Point32 linesInteraction(Line_3 l1, Line_3 l2);

geometry_msgs::Point32 linesInteraction(std::vector<geometry_msgs::Point32>& line1, std::vector<geometry_msgs::Point32>& line2);

void extendTriangleMargin(std::vector<geometry_msgs::Point32>& vec1, std::vector<geometry_msgs::Point32>& vec2, double extend_radius);

Line_3 getExtendLine(Point_3 vert1, Point_3 vert2, Point_3 vert3, double extend_radius);
#endif
