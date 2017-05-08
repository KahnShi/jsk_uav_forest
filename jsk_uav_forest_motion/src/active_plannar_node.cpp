#include <jsk_uav_forest_motion/ActivePlannar.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "active_plannar");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");
  ActivePlannar* active_plannar = new ActivePlannar(nh, nhp);

  ros::spin();
  delete active_plannar;
  return 0;
}
