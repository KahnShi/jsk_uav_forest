#include <jsk_uav_forest_motion/ActivePlannar.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "active_plannar");
  ActivePlannar active_plannar;
  active_plannar.onInit();

  ros::spin();
  return 0;
}
