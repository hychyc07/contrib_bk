namespace yarp iCub

service dmpManagerInterface
{
  bool train(1:string action, 2:string target, 3:string tool, 4:string hand);
  bool stop_training();
  bool s();
  bool test(1:string action, 2:string target, 3:string tool, 4:string hand);
  string observe_state();
  oneway void go_home();
  oneway void quit();
}

