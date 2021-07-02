#include "controller/src/controller/smc_rbf.h"
#include <ros/package.h>
#include <ros/ros.h>
#include <cmath>

const double PI = 3.141592653589793238463;

using namespace std;

namespace rambot{
namespace control{
void SMC_RBFController::Init(ros::NodeHandle* nh_ptr)
{
    ControlConfig &config = ControlConfig::GetControlConfig();
    ts_ = config.Config().common_conf().ts();
          // TODO: 使用config变成变量�?    wheelbase_length = config.Config().common_conf().wheel_base();
    unit_trans = config.Config().lat_controller_conf().steer_unit_trans();
    
}

void SMC_RBFController::ComputeControlCommand()
{
  TrajectoryParser &trajectory = TrajectoryParser::GetTrajectoryParser();
  LowerProxy &lower_proxy = LowerProxy::GetLowerProxy();
  Logging &logging = Logging::GetLogging();
  
  rambot::chassis::Chassis chassis = lower_proxy.Chassis();
  double x = chassis.x();
  double y = chassis.y();
  double linear_v = chassis.v();
  double theta = chassis.theta();

  //looking ahead
  double Lh = 0.8;
  //double Lh = 0.0;

  double control_point_x = x + Lh * cos(theta);
  double control_point_y = y + Lh * sin(theta);

  auto reference_point = trajectory.GetNearestPointInGlobal(control_point_x, control_point_y);

  double dx = control_point_x - reference_point.x();
  double dy = control_point_y - reference_point.y();

  if (!is_init) {
    is_init = true;
    last_theta = chassis.theta();
    last_steering_angle = 0;
  }
  
  double angle_output = 0;

  //calculate the control variables
  float lateral_error = -dx * sin(reference_point.theta()) + dy * cos(reference_point.theta());
  double orientational_error = math::NormalizeAngle(theta - reference_point.theta());

  double current_omega = tan(last_steering_angle) * linear_v / wheelbase_length;

  double diff_lateral_error = linear_v * sin(orientational_error) + Lh * current_omega * cos(orientational_error);
  double diff_orientational_error = current_omega;

  //1. sgn() : sign function
  //double  sgn_lateral_error = fabs(lateral_error) / lateral_error;
  //2. saturation function
  double k_s = lateral_error / boundary ;
  double sat_lateral_error = 0.0;
  if (fabs(k_s) < 1)
      sat_lateral_error = k_s;
  else
     sat_lateral_error = fabs( k_s ) / k_s;
  

  //define the sliding surface where lateral error and orientional error 
  //are internally coupled with each other
  float s = diff_lateral_error + k * lateral_error + k0 * sat_lateral_error * orientational_error;

  //the rate is forced to approach the switching manifolds faster when s is large
  double diff_s = -q * s - p * sat_lateral_error;
  //define RBF neural network k1 segma_1//input last[s1,Derivative_s1]
  double teacher_E = 0.5 * pow(s,2);
  double Derivative_teacher_E = s;
  Eigen::MatrixXd s_input(1,2);
  s_input << s , diff_s;
  s_rbf->training(s_input,Derivative_teacher_E);

  // redefine the next k1,k2,segma_1,segma_2
  //q = s_rbf->output();
  //cout <<"q:          " <<q<<endl;
    //k_2 = s2_rbf->output();
  float steering_speed;

  steering_speed = wheelbase_length * pow(cos(last_steering_angle),2)
                         / (linear_v * Lh * cos(orientational_error))
                         * (diff_s - k * diff_lateral_error 
                         - linear_v * diff_orientational_error * cos(orientational_error)
                         + Lh * sin(orientational_error) * pow(diff_orientational_error,2)
                         - k0 * sat_lateral_error * diff_orientational_error);

  
  if(linear_v > init_speed_threshold)
     angle_output = last_steering_angle + steering_speed * ts_ ;

  angle_output = rambot::control::math::Clamp(angle_output, 0.52, -0.52);

  const float MAX_STEERING_SPEED = 0.5025; 
  // clamp from changing too fast
  angle_output = rambot::control::math::Clamp(angle_output, 
                       last_steering_angle + MAX_STEERING_SPEED * ts_,
                       last_steering_angle - MAX_STEERING_SPEED * ts_ );
  
  lower_proxy.Command()->set_steer_angle(angle_output * unit_trans);
  last_steering_angle = angle_output;

  //Visualizor &visualizor = Visualizor::GetVisualizor();
  //visualizor.Visualize_user(s,orientational_error,reference_point.kappa());
  
  cout << "\nsliding_mode control information:\n";
  cout << left << setw(20) << "[ angle_output: " << right
       << setw(20) << angle_output << "]" << endl;
  cout << left << setw(20) << "[ steering_speed: " << right << setw(20)
       << steering_speed << "]" << endl;
  cout << left << setw(20) << "[ s: " << right << setw(20)
       << s << "]" << endl;
  cout << left << setw(20) << "[ lateral_error: " << right << setw(20)
       << lateral_error << "]" << endl;  
    


   
}

void SMC_RBFController::Reset() { is_init = false; }

void SMC_RBFController::Stop(){
    LowerProxy &lower_proxy = LowerProxy::GetLowerProxy();
    lower_proxy.Command()->set_linear_v(0.0);
    lower_proxy.Command()->set_steer_angle(0.0);
}

} 
}
