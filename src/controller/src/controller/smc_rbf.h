

#include <Eigen/Dense>
#include <chrono> // NOLINT
#include <fstream>
#include <string>
#include <iostream>
#include <vector>

#include "rambot_controller/src/common/control_config.h"

#include "rambot_controller/src/controller/base_controller.h"
#include "rambot_controller/src/path_proxy/trajectory_parser.h"
#include "rambot_controller/src/data_proxy/lower_proxy.h"
#include "rambot_controller/src/common/logging.h"
#include "rambot_controller/src/tools/math.hpp"
#include "rambot_controller/src/tools/matrix_operation.h"

#include "src/data_types/path_point.pb.h"
#include "src/data_types/chassis.pb.h"
#include "rambot_controller/src/common/logging.h"
#include "rambot_controller/src/common/numpy.hpp"
#include "rbfn.cpp"
#include <string>
#include <ros/ros.h>
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;

namespace rambot{
namespace control{
class SMC_RBFController : public BaseController
{
public:
    SMC_RBFController() = default;

    virtual ~SMC_RBFController() = default;

    void Init(ros::NodeHandle*) override;

    void ComputeControlCommand() override;

    void Reset() override;

    void Stop() override;

private:
    //sliding surface parameters 
  double k = 0.5;
  double k0 = 0.05;

  //distance of looking ahead
  //double Lh = 1.0;
  double k_l = 0.2;

  //specify different reaching rate and structure
  double q = 10.0;
  double p = 10.0;
  double boundary = 0.5;

  float last_theta = 0;  // car status
  bool is_init = false;
  double last_steering_angle = 0;

  double init_speed_threshold = 1;

  ControlConfig &config = ControlConfig::GetControlConfig();
  double ts_ = config.Config().common_conf().ts();
  // TODO: 使用config变成变量�?  double wheelbase_length = config.Config().common_conf().wheel_base();
  double unit_trans = config.Config().lat_controller_conf().steer_unit_trans();
    // RBFN
  
    RBFN *s_rbf = new RBFN(2,5);

};
} // namespace control
} 