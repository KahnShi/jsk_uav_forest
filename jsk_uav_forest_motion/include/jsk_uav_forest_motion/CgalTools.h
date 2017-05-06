#ifndef CGAL_TOOLS_H_
#define CGAL_TOOLS_H_

#include <CGAL/Cartesian.h>
#include <iostream>
#include <geometry_msgs/Point32.h>

typedef CGAL::Cartesian<double>     Kernel;
typedef Kernel::Line_2              Line_2;
typedef Kernel::Line_3              Line_3;
typedef Kernel::Point_3             Point_3;
typedef Kernel::Triangle_3          Triangle_3;
typedef CGAL::Cartesian<double>     Kernel;
typedef Kernel::Intersect_3         Intersect_3;

bool doTriangleInteresect(std::vector<geometry_msgs::Point32>& vec1, std::vector<geometry_msgs::Point32>& vec2);

geometry_msgs::Point32 linesInteraction(std::vector<geometry_msgs::Point32>& line1, std::vector<geometry_msgs::Point32>& line2);

#endif
