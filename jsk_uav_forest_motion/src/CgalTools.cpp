#include <jsk_uav_forest_motion/CgalTools.h>

bool doTriangleInteresect(std::vector<geometry_msgs::Point32>& vec1, std::vector<geometry_msgs::Point32>& vec2)
{
  std::vector<Point_3> pts1, pts2;
  for (int i = 0; i < 3; ++i){
    Point_3 pt(vec1[i].x, vec1[i].y, vec1[i].z);
    pts1.push_back(pt);
  }
  for (int i = 0; i < 3; ++i){
    Point_3 pt(vec2[i].x, vec2[i].y, vec2[i].z);
    pts2.push_back(pt);
  }
  Triangle_3 t1(pts1[0], pts1[1], pts1[2]);
  Triangle_3 t2(pts2[0], pts2[1], pts2[2]);
  bool result = CGAL::do_intersect(t1, t2);
  if (result)
    return true;
  else
    return false;
}

geometry_msgs::Point32 linesInteraction(std::vector<geometry_msgs::Point32>& line1, std::vector<geometry_msgs::Point32>& line2)
{
  std::vector<Point_3> pts1, pts2;
  for (int i = 0; i < 2; ++i){
    Point_3 pt(line1[i].x, line1[i].y, line1[i].z);
    pts1.push_back(pt);
  }
  for (int i = 0; i < 2; ++i){
    Point_3 pt(line2[i].x, line2[i].y, line2[i].z);
    pts2.push_back(pt);
  }
  geometry_msgs::Point32 pt32;
  pt32.x = -10000.0; pt32.y = -10000.0; pt32.z = -10000.0;
  Line_3 l1(pts1[0], pts1[1]), l2(pts2[0], pts2[1]);
  CGAL::cpp11::result_of<Intersect_3(Line_3, Line_3)>::type result = intersection(l1, l2);
  if (result){
    const Point_3* p = boost::get<Point_3 >(&*result);
    pt32.x = p->x(); pt32.y = p->y(); pt32.z = p->z();
  }
  return pt32;
}
