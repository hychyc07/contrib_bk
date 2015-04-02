namespace yarp iCub

enum Hand
{
  RIGHT = 0;
  LEFT=1;
  INDIFF=2;
}
service dmpExecutorInterface
{
  bool run();
  bool is_running();
  bool stop();
  bool s();
  bool execute_OPC(1:i32 id);
  bool waitMotionDone(1: double period = 0.5, 2:double timeout=0.0);
  bool set_hand(1:Hand newHand);
  Hand get_hand();
  bool teach_start(1:string actionName, 2:Hand handToUse=0);
  oneway void quit();

//  bool execute_DMP(1:DMP dmp);
}

