#include <jsk_uav_forest_motion/TriangleGenerator.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "spline_path");

  triangleGenerator safe_path;
  safe_path.onInit();

  ros::spin();
  return 0;
}
