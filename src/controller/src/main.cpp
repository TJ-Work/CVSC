#include "controller/src/main.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "rambot_conroller");
  GlogInit(argv[0]);
  google::InstallFailureSignalHandler();

  ros::NodeHandle n("~");
  ControlConfig& config = ControlConfig::GetControlConfig();
  LowerProxy& lower_proxy = LowerProxy::GetLowerProxy();
  UpperProxy& upper_proxy = UpperProxy::GetUpperProxy();
  ControllerMonitor& monitor = ControllerMonitor::GetStateMonitor();
  TrajectoryParser& trajectory_parser = TrajectoryParser::GetTrajectoryParser();
  Visualizor& visualizor = Visualizor::GetVisualizor();
  Logging& logging = Logging::GetLogging();

  config.LoadParams(argv[1], argv[2]);
  LOG(INFO) << argv[2];
  logging.Init();
  monitor.Init(&n);
  lower_proxy.Init(&n);
  upper_proxy.Init(&n);
  visualizor.Init(&n);
  trajectory_parser.Init();
  ControlProcessor processor(StopState::GetStopState());

  ros::Rate rate(config.Config().common_conf().loop_rate());
  while (ros::ok()) {
    rate.sleep();
    ros::spinOnce();
    upper_proxy.UpdataPathInfo();
    trajectory_parser.GenerateTrajectory();
    monitor.Monitor();
    processor.ControlCommand();
    lower_proxy.PublishControlCommand();
    visualizor.Visualize();
  }
  return 0;
}
