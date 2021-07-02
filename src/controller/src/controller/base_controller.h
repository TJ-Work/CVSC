#include <ros/ros.h>
namespace rambot {
namespace control {
class BaseController {
 public:
  BaseController() = default;

  virtual ~BaseController() = default;

  virtual void Init(ros::NodeHandle*) = 0;

  virtual void ComputeControlCommand() = 0;

  virtual void Reset() = 0;

  virtual void Stop() = 0;
};
}  
}  
#endif 
