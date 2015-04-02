struct Vector 
{
  1: list<double> content;
} 
(
  yarp.name = "yarp::sig::Vector"
  yarp.includefile="yarp/sig/Vector.h"
)

struct ClassScore
{
  1:string className;
  2:double score;
}

struct ClassScoreList
{
 1:list<ClassScore> elements;
}

service GurlsClassificationInterface
{
  bool add_sample(1: string className, 2:Vector sample);
  bool save(1: string className);
  bool train();
  bool forget(1:string className);
  list<string> classList();
  bool stop();
  bool recognize();
  string classify_sample(1:Vector sample);
  list<ClassScore> get_scores_for_sample(1:Vector sample);
  string get_parameter(1:string parameterName);
  bool set_parameter(1:string parameterName, 2:string parameterValue);
}
