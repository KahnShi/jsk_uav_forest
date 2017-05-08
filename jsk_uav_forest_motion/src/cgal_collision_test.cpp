#include <CGAL/Cartesian.h>
#include <iostream>
typedef CGAL::Cartesian<double>     Kernel;
typedef Kernel::Point_2             Point_2;
typedef Kernel::Point_3             Point_3;
typedef Kernel::Line_3              Line_3;
typedef Kernel::Triangle_2          Triangle_2;
typedef Kernel::Triangle_3          Triangle_3;
typedef CGAL::Cartesian<double>     Kernel;
typedef Kernel::Intersect_3         Intersect_3;

int main()
{
  // Point_3 p0(-21, -72, 63), p1(-78, 99, 40), p2(-19, -78, -83);
  // Point_3 q0(96, 77, -51), q1(-95, -1, -16), q2(9, 5, -21);
  Point_3 p0(-7.99979, -0.000196484, 0), p1(-8.00075, 0.000715654, 0), p2(-8.00266, 0.00253993, 0);
  Point_3 q0(-11.9741, -4.95182, 0), q1(6.1569, -4.94446, 0), q2(-6.60335, -4.95213, 0);
  Triangle_3 t1(p0, p1, p2), t2(q0, q1, q2);
  bool result = CGAL::do_intersect(t1, t2);
  if (result)
    std::cout << "In collision\n";
  else
    std::cout << "No collisiion\n";

  /* Output the intersect point of 2 lines */
  Line_3 l1(Point_3(0, 1, 1), Point_3(1, 2, 1)), l2(Point_3(3, 0, 1), Point_3(5, 0, 1));
  CGAL::cpp11::result_of<Intersect_3(Line_3, Line_3)>::type result2 = intersection(l1, l2);
  if (result2){
    std::cout << "In collision\n";
    const Point_3* p = boost::get<Point_3 >(&*result2);
    std::cout << p->x() << ", " << p->y() << std::endl;
  }
  else
    std::cout << "No collisiion\n";

  return 0;
}
