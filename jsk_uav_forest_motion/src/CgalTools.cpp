#include <jsk_uav_forest_motion/CgalTools.h>

bool doTriangleInteresect2(std::vector<geometry_msgs::Point32>& vec1, std::vector<geometry_msgs::Point32>& vec2)
{
  std::vector<Point_2> pts1, pts2;
  for (int i = 0; i < 3; ++i){
    Point_2 pt(vec1[i].x, vec1[i].y);
    pts1.push_back(pt);
  }
  for (int i = 0; i < 3; ++i){
    Point_2 pt(vec2[i].x, vec2[i].y);
    pts2.push_back(pt);
  }
  Triangle_2 t1(pts1[0], pts1[1], pts1[2]);
  Triangle_2 t2(pts2[0], pts2[1], pts2[2]);
  // std::cout << "before\n";
  // for (int i = 0; i < 3; ++i){
  //   std::cout << "[" << pts1[i].x() << ", " << pts1[i].y() << "]  ";
  // }
  // std::cout << "\n";
  // for (int i = 0; i < 3; ++i){
  //   std::cout << "[" << pts2[i].x() << ", " << pts2[i].y() << "]  ";
  // }
  // std::cout << "\n";
  bool result = CGAL::do_intersect(t1, t2);
  if (result)
    return true;
  else
    return false;
}

bool doTriangleInteresect3(std::vector<geometry_msgs::Point32>& vec1, std::vector<geometry_msgs::Point32>& vec2)
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
  // std::cout << "before\n";
  // for (int i = 0; i < 3; ++i){
  //   std::cout << "[" << pts1[i].x() << ", " << pts1[i].y() << ", " << pts1[i].z() << "]  ";
  // }
  // std::cout << "\n";
  // for (int i = 0; i < 3; ++i){
  //   std::cout << "[" << pts2[i].x() << ", " << pts2[i].y() << ", " << pts2[i].z() << "]  ";
  // }
  // std::cout << "\n";
  bool result = CGAL::do_intersect(t1, t2);
  if (result)
    return true;
  else
    return false;
}

geometry_msgs::Point32 linesInteraction(Line_3 l1, Line_3 l2)
{
  geometry_msgs::Point32 pt32;
  pt32.x = -10000.0; pt32.y = -10000.0; pt32.z = -10000.0;
  CGAL::cpp11::result_of<Intersect_3(Line_3, Line_3)>::type result = intersection(l1, l2);
  if (result){
    const Point_3* p = boost::get<Point_3 >(&*result);
    pt32.x = p->x(); pt32.y = p->y(); pt32.z = p->z();
  }
  return pt32;
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
  Line_3 l1(pts1[0], pts1[1]), l2(pts2[0], pts2[1]);
  geometry_msgs::Point32 pt32 = linesInteraction(l1, l2);
  return pt32;
}

void extendTriangleMargin(std::vector<geometry_msgs::Point32>& tri, std::vector<geometry_msgs::Point32>& tri_extend, double extend_radius)
{
  Point_3 verticles[3];
  for (int i = 0; i < 3; ++i){
    verticles[i] = Point_3(tri[i].x, tri[i].y, tri[i].z);
  }
  Triangle_3 tri_3(verticles[0], verticles[1], verticles[2]);
  // if 3 verticles are colinear, ignore
  if (tri_3.is_degenerate()){
    for (int i = 0; i < 3; ++i)
      tri_extend[i] = tri[i];
    return;
  }

  Line_3 l01, l12, l02;
  l01 = getExtendLine(verticles[0], verticles[1], verticles[2], extend_radius);
  l12 = getExtendLine(verticles[1], verticles[2], verticles[0], extend_radius);
  l02 = getExtendLine(verticles[0], verticles[2], verticles[1], extend_radius);
  tri_extend[0] = linesInteraction(l01, l02);
  tri_extend[1] = linesInteraction(l01, l12);
  tri_extend[2] = linesInteraction(l12, l02);
  // if(!tri_extend.empty())
  //   tri_extend.clear();
  // tri_extend.push_back(linesInteraction(l01, l02));
  // tri_extend.push_back(linesInteraction(l01, l12));
  // tri_extend.push_back(linesInteraction(l12, l02));
}

Line_3 getExtendLine(Point_3 vert1, Point_3 vert2, Point_3 vert3, double extend_radius)
{
  // todo: current 2d extend
  double k = (vert2.y() - vert1.y()) / (vert2.x() - vert1.x());
  double sin_k = k / sqrt(1 + k*k);
  double factor = 1;
  if (k * (vert3.x() - vert1.x()) - (vert3.y() - vert1.y()) > 0){
    factor = -1;
  }
  // todo: z = 0.0?
  /* in case k = 0, then 1/0 will cause error */
  if (fabs(k) > 0.1){
    Point_3 new_vert1(vert1.x() + factor*extend_radius/sin_k, vert1.y(), 0.0);
    Point_3 new_vert2(vert2.x() + factor*extend_radius/sin_k, vert2.y(), 0.0);
    return Line_3(new_vert1, new_vert2);
  }
  else{
    Point_3 new_vert1(vert1.x(), vert1.y() - factor*extend_radius, 0.0);
    Point_3 new_vert2(vert2.x(), vert2.y() - factor*extend_radius, 0.0);
    return Line_3(new_vert1, new_vert2);
  }
}
