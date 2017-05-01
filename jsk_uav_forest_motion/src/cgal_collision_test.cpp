#include <CGAL/Cartesian.h>
#include <iostream>
typedef CGAL::Cartesian<double>     Kernel;
typedef Kernel::Point_3             Point_3;
typedef Kernel::Triangle_3          Triangle_3;
typedef CGAL::Cartesian<double>     Kernel;

int main()
{
  // Point_3 p0(-21, -72, 63), p1(-78, 99, 40), p2(-19, -78, -83);
  // Point_3 q0(96, 77, -51), q1(-95, -1, -16), q2(9, 5, -21);
  Point_3 p0(0.0, 0.0, 0), p1(2, 0, 0), p2(0, 1, 0);
  Point_3 q0(-5, 0, 0), q1(-1, 0, 0), q2(0, 4, 0);
  Triangle_3 t1(p0, p1, p2), t2(q0, q1, q2);
  bool result = CGAL::do_intersect(t1, t2);
  if (result)
    std::cout << "In collision\n";
  else
    std::cout << "No collisiion\n";
  return 0;
}
