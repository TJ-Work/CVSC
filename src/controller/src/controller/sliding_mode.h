
#include <Eigen/Dense>
#include <chrono>  
#include <fstream>
#include <string>
#include <vector>

#include "rambot_controller/src/controller/adp_learning/action_network.cpp"

#include "rambot_controller/src/common/control_config.h"

#include "rambot_controller/src/controller/base_controller.h"
#include "rambot_controller/src/path_proxy/trajectory_parser.h"
#include "rambot_controller/src/data_proxy/lower_proxy.h"

#include "rambot_controller/src/tools/math.hpp"

#include "rambot_controller/src/common/logging.h"
#include "src/data_types/path_point.pb.h"
#include "src/data_types/chassis.pb.h"
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>


using Matrix = Eigen::MatrixXd;
namespace rambot {
namespace control {
class SlidingMode : public BaseController {
 public:
  SlidingMode() = default;

  virtual ~SlidingMode() = default;
 
  void Init(ros::NodeHandle*);

  void ComputeControlCommand() override;

  void Reset() override;

  void Stop() override;

 private:
  //sliding surface parameters 
  double k = 0.5;
  double k0 = 0.03;

  //distance of looking ahead
  //double k_l = 0.2;

  //specify different reaching rate and structure
  double q = 5.0;
  double p = 10.0;
  double kd = 1.0;
  double boundary = 1;

  bool is_init = false;
  bool is_start = false;
  double last_steering_angle = 0;
  double last_steering_angle1 = 0;
  double last_s_error = 0;
  double last_reference_point_x = 0;
  double last_reference_point_y = 0;
  double last_reference_point_theta = 0;
  double init_speed_threshold = 0.001;
  double steer_lock = 0.52;

  double ts_ ;
  // TODO: 使用config变成变量�?  double wheelbase_length ;
  double unit_trans ;
  double J_cost;
  double last_output;
  double last_act;
  double act;
  double output;
  double loop_count;

  action_network *action_network1 = new action_network();
  action_network *action_network_learn_smc = new action_network();
  critic_network *critic_network1 = new critic_network();
  
  double a = 1;
  double b = 1;
  double c = 1;

  ros::NodeHandle* nh_ptr_;
  ros::Publisher adp_pub_;
  visualization_msgs::Marker adp_msg_;

};
}  // namespace control
}  