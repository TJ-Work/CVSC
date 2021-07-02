#include "controller/src/controller/sliding_mode.h"
#include <ros/package.h>
#include <ros/ros.h>
#include <cmath>
#include <visualization_msgs/Marker.h>

using namespace std;

namespace rambot {
namespace control {
void SlidingMode::Init(ros::NodeHandle* nh_ptr) {   
  ControlConfig &config = ControlConfig::GetControlConfig();
  ts_ = config.Config().common_conf().ts();
  // TODO: 使用config变成变量�?  wheelbase_length = config.Config().common_conf().wheel_base();
  unit_trans = config.Config().lat_controller_conf().steer_unit_trans();
  nh_ptr_ = nh_ptr;
  adp_pub_ =
      nh_ptr_->advertise<visualization_msgs::Marker>("/adp_indicator", 1);

}

void SlidingMode::ComputeControlCommand() {
  
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
  //double Lh = k_l * linear_v;
  if (!is_init) {
    is_init = true;
    last_steering_angle = 0;
    last_s_error = 0;
    J_cost = 0;
    output = 0;
    last_output = 0;
    //last_reference_point = trajectory.GetNearestPointInGlobal(x,y)
  }

  double control_point_x = x + Lh * cos(theta);
  double control_point_y = y + Lh * sin(theta);

  auto reference_point = trajectory.GetNearestPointInGlobal(control_point_x, control_point_y);
  double dx1 = reference_point.x()-last_reference_point_x;
  double dy1 = reference_point.y()-last_reference_point_y;
  double distance = sqrt(pow(dx1,2)+pow(dy1,2));
  double deriva = (reference_point.theta()-last_reference_point_theta)/(distance);
  last_reference_point_x = reference_point.x();
  last_reference_point_y = reference_point.y();
  last_reference_point_theta = reference_point.theta();
  
  double dx = control_point_x - reference_point.x();
  double dy = control_point_y - reference_point.y();
  last_reference_point_x = reference_point.x();
  last_reference_point_y = reference_point.y();
  last_reference_point_theta = reference_point.theta();

  
  double angle_output = 0;

  double lateral_error = -dx * sin(reference_point.theta()) + dy * cos(reference_point.theta());
  double orientational_error = math::NormalizeAngle(theta - reference_point.theta());

  double current_omega = tan(last_steering_angle) * linear_v / wheelbase_length;

  double diff_lateral_error = linear_v * sin(orientational_error) + Lh * current_omega * cos(orientational_error);
  double diff_orientational_error = current_omega;

  //saturation function
  double k_s = lateral_error / boundary ;
  double sat_lateral_error = 0.0;
  
  if (fabs(k_s) < 0.5)
      sat_lateral_error = k_s;
  else
     sat_lateral_error = 0.5 * fabs( k_s ) / k_s;
  

  //define the sliding surface where lateral error and orientional error 
  //are internally coupled with each other

  double s = diff_lateral_error + k * lateral_error + k0 * sat_lateral_error * orientational_error;
  
  // small pid controller
  
  //double desired_s = 0; 
  //double s_error = s - desired_s;
  //double s_diff_error = s_error - last_s_error;

  //the rate is forced to approach the switching manifolds faster when s is large
  //double diff_s = -(q * (s - desired_s) + kd * s_diff_error) - p * sat_lateral_error;

  //last_s_error = s_error;
  double sat_s = 0.0;
  if(fabs(s) < 0.5)
    sat_s = s;
  else
    sat_s = 0.5 * fabs(s)/s;
  

  double diff_s = -q * s - p * sat_s;


  // adaptive dynamic programming
  // input s and diff_S

  
  double last_J_cost = J_cost;
 
  cout << "lateral_error: " << lateral_error << " orientational_error: " << orientational_error <<endl; 
  
  double reference_signal =  (pow(lateral_error,2) + pow(orientational_error,2)+pow(last_steering_angle, 2));
  cout << "jcost-refer" << 0.5*pow(J_cost - reference_signal,2) << endl;
  loop_count = 0;
  while(0.5*pow(J_cost - reference_signal,2)>0.000000001 && loop_count < 50)
  {
    J_cost = critic_network1->output_J_cost(lateral_error,orientational_error,act);
    critic_network1->update_weight(J_cost, last_J_cost, reference_signal);
    loop_count += 1;
    cout<<loop_count<<endl;
  }
  last_output = output;
  last_act = act;
  loop_count = 0;
  while (J_cost < 0.1 && loop_count < 200)
  {
    act = action_network1->output_cmd(lateral_error, orientational_error); 
    action_network1->update_weight(J_cost, critic_network1);
    J_cost = critic_network1->output_J_cost(lateral_error,orientational_error,act);
    loop_count += 1;
  }
  if (J_cost < 0.1 )
      output =  0.1 * steer_lock * act;
  else
      output = 0;
  //limit the delta_output;
  // double MAX_CHANGE = 0.5075 * ts_;
  // double delta_output = rambot::control::math::Clamp(output-last_output, MAX_CHANGE, -MAX_CHANGE);
  // output = last_output + delta_output;
 
  //double diff_s1 = -10 * pow(s, 0.5)
  double diff_s1 = diff_s + act * 10 ;
 
  double  steering_speed1 = wheelbase_length * pow(cos(last_steering_angle),2)
                         / (linear_v * Lh * cos(orientational_error))
                         * (diff_s - k * diff_lateral_error 
                         - linear_v * diff_orientational_error * cos(orientational_error)
                         + Lh * sin(orientational_error) * pow(diff_orientational_error,2)
                         - k0 * sat_lateral_error * diff_orientational_error);

  //critic_network1->update_weight(J_cost, last_J_cost, a, b, c); 
  double  steering_speed = wheelbase_length * pow(cos(last_steering_angle),2)
                         / (linear_v * Lh * cos(orientational_error))
                         * (diff_s - k * diff_lateral_error 
                         - linear_v * diff_orientational_error * cos(orientational_error)
                         + Lh * sin(orientational_error) * pow(diff_orientational_error,2)
                         - k0 * sat_lateral_error * diff_orientational_error);
  

  double MAX_STEERING_SPEED = 0.5075; 
  // clamp from changing too fast
  steering_speed = rambot::control::math::Clamp(steering_speed, MAX_STEERING_SPEED, - MAX_STEERING_SPEED);
  steering_speed1 = rambot::control::math::Clamp(steering_speed, MAX_STEERING_SPEED, - MAX_STEERING_SPEED);

  double angle_output1;
  if(linear_v > init_speed_threshold)
  {
    angle_output = last_steering_angle + steering_speed * ts_ + output;
    angle_output1 = last_steering_angle1 + steering_speed1 * ts_;
    //angle_output = angle_output1 + output;
  }
  //output = steering_speed -steering_speed1 ;


  angle_output = rambot::control::math::Clamp(angle_output1, steer_lock, -steer_lock);
  // if(!is_start)
  // {
  //   if(linear_v < 50)
  //   {
  //     angle_output = 0;
  //   }
  //   else
  //   {
  //     angle_output = 0.087*6;
  //     is_start = true;
  //   }
    

  // }
  // else
  // {
  //   angle_output = 0.087 * 6;
  // }
  
  
  lower_proxy.Command()->set_steer_angle(angle_output * unit_trans);
  last_steering_angle = angle_output;
  last_steering_angle1 = angle_output1;
  
  cout << "\nsliding_mode control information:\n";
  cout << left << setw(20) << "[ angle_output: " << right
       << setw(20) << angle_output << "]" << endl;
  cout << left << setw(20) << "[ s: " << right
       << setw(20) << s << "]" << endl;
  cout << left << setw(20) << "[ diff_s: " << right
       << setw(20) << diff_s << "]" << endl;
  cout << left << setw(20) << "[ output: " << right
       << setw(20) << output << "]" << endl;
  //cout << left << setw(20) << "[ J_cost: " << right
  //     << setw(20) << J_cost << "]" << endl;  
  adp_msg_.scale.x = J_cost;
  adp_msg_.scale.y = reference_signal;
  adp_msg_.scale.z = (57.3 * output);
  adp_msg_.pose.orientation.x = lateral_error;
  adp_msg_.pose.position.x = lateral_error;
  adp_msg_.pose.position.y = orientational_error;
  adp_msg_.pose.position.z = theta;
  adp_msg_.pose.orientation.x = reference_point.theta();
  adp_msg_.pose.orientation.y = s;
  adp_msg_.pose.orientation.z = diff_s;
  adp_msg_.pose.orientation.w = deriva;
  adp_pub_.publish(adp_msg_);   
}

void SlidingMode::Reset() { is_init = false; }

void SlidingMode::Stop() {
  LowerProxy &lower_proxy = LowerProxy::GetLowerProxy();
  lower_proxy.Command()->set_linear_v(0.0);
  lower_proxy.Command()->set_steer_angle(0.0);
}
}  // namespace control
}  













