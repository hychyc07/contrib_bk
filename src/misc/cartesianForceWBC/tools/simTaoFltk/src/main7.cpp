
/**
   \file main7.cpp
   \author by Juan G Victores
   \author contrib Roland Philippsen (tut00_test.cpp of WBC)
   
   A quick test to see whether the simulator works as expected.

*/

#include "tutsim7.hpp"
#include <jspace/test/sai_util.hpp>
#include <boost/shared_ptr.hpp>
#include <err.h>

// #define REAL_ROBOT  // DO NOT USE for testing, it SENDS torque commands! (not implemented i believe...)
//#define DO_NOT_CONNECT

static boost::shared_ptr<jspace::Model> model;
static IPositionControl *pos;
static double trg_deg[7];
static BufferedPort<yarp::sig::Vector> qPort;

class TrgPort : public BufferedPort<Bottle> {
  public:
    TrgPort() {}
  private:
    virtual void onRead(Bottle& b) {
        printf("[toolPort::callback] Got %s\n", b.toString().c_str());
        if ((b.get(0).asString()=="poss")&&(b.size() == 6)) {
            trg_deg[0] = b.get(1).asDouble();
            trg_deg[1] = b.get(2).asDouble();
            trg_deg[2] = b.get(3).asDouble();
            trg_deg[3] = b.get(4).asDouble();
            trg_deg[4] = b.get(5).asDouble();
        } else if ((b.get(0).asString()=="rels")&&(b.size() == 6)) {
            trg_deg[0] += b.get(1).asDouble();
            trg_deg[1] += b.get(2).asDouble();
            trg_deg[2] += b.get(3).asDouble();
            trg_deg[3] += b.get(4).asDouble();
            trg_deg[4] += b.get(5).asDouble();
        } else if ((b.get(0).asString()=="pos")&&(b.size() == 3)) {
            trg_deg[b.get(1).asInt()] = b.get(2).asDouble();
        } else if ((b.get(0).asString()=="rel")&&(b.size() == 3)) {
            trg_deg[b.get(1).asInt()] += b.get(2).asDouble();
        } else if ((b.get(0).asString()=="get")&&(b.size() == 1)) {
            printf("[%f, %f, %f, %f, %f]\n",trg_deg[0],trg_deg[1],trg_deg[2],trg_deg[3],trg_deg[4]);
        } else printf("[error] command not understood\n");
//        if (b.get(0).getCode() == BOTTLE_TAG_DOUBLE) {
//            target = b.get(0).asDouble();
//        }
    }
};

static bool servo_cb(size_t toggle_count,
		     double wall_time_ms,
		     double sim_time_ms,
		     jspace::State & state,
		     jspace::Vector & command)
{
  model->update(state);
  jspace::Matrix mass_inertia;
  model->getMassInertia(mass_inertia);
  jspace::Vector gravity;
  model->getGravity(gravity);
  
  static size_t counter(0);
  if (0 == (counter % 50)) {  // This outputs data every 50 iterations
    std::cerr << "wall: " << wall_time_ms << "  sim: " << sim_time_ms << "\n";
    jspace::pretty_print(state.position_, std::cerr, "jpos", "  ");

    double jointPos[7];

    for (int i=0;i<7;i++)
//        jointPos[i] = state.position_[i]*180.0/M_PI;
          jointPos[i] = trg_deg[i];
//          jointPos[i] = state.position_[i]*180.0/M_PI;
     yarp::sig::Vector& output = qPort.prepare();
     output.resize(16);
     output.zero();
    for (int i=0;i<7;i++)
          output[i] = state.position_[i]*180.0/M_PI;
    
/*    jointPos[0] = -90.0 + state.position_[0]*180.0/M_PI;
    jointPos[1] = 90.0 + state.position_[1]*180.0/M_PI;
    jointPos[2] = -75.0 + state.position_[2]*180.0/M_PI;
    jointPos[3] = 0 + state.position_[3]*180.0/M_PI;
    jointPos[4] = 90.0 + state.position_[4]*180.0/M_PI;*/
#ifndef DO_NOT_CONNECT
          pos->positionMove(jointPos);
          qPort.write();
#endif

    // this was just a hack, the mass inertia will be wrong because tao doesn't know where we are
    jspace::pretty_print(mass_inertia, std::cerr, "mass_inertia", "  ");
    jspace::pretty_print(gravity, std::cerr, "gravity", "  ");
  }
  ++counter;
  
//  if (0 == (toggle_count % 2)) {
//    static double const aa(M_PI / 2);
    static double const aa(0);
    for (int ii(0); ii < state.position_.rows(); ++ii) {
//      state.position_[ii] = aa * sin((1.0 + 0.1 * ii) * 1e-3 * wall_time_ms);
      if (ii==0)
          state.position_[ii] = (trg_deg[ii]) * M_PI / 180.0;
      else if (ii==1)
          state.position_[ii] = (trg_deg[ii]) * M_PI / 180.0;
      else if (ii==2)
          state.position_[ii] = (0+trg_deg[ii]) * M_PI / 180.0;
      else if (ii==3)
          state.position_[ii] = (0 + trg_deg[ii]) * M_PI / 180.0;
      else if (ii==4)
          state.position_[ii] = (0 + trg_deg[ii]) * M_PI / 180.0;
      else if (ii==5)
          state.position_[ii] = (0 + trg_deg[ii]) * M_PI / 180.0;
      else if (ii==6)
          state.position_[ii] = (0 + trg_deg[ii]) * M_PI / 180.0;
      else
/*      if (ii==0)
          state.position_[ii] = trg_deg[ii] * M_PI / 180.0;
      else if (ii==1)
          state.position_[ii] = trg_deg[ii] * M_PI / 180.0;
      else if (ii==2)
          state.position_[ii] = trg_deg[ii] * M_PI / 180.0;
      else if (ii==3)
          state.position_[ii] = trg_deg[ii] * M_PI / 180.0;
      else if (ii==4)
          state.position_[ii] = trg_deg[ii] * M_PI / 180.0;
      else*/
          state.position_[ii] = aa;
      state.velocity_[ii] = 0.0;
    }
//    return false;
//  } // returns, so the following is a kind of // } else {
  
  command = jspace::Vector::Zero(state.position_.rows());
  return true;
}


int main(int argc, char ** argv) {
/*  trg_deg[0] = -28.0;
  trg_deg[1] = 80.0;
  trg_deg[2] = 15.0;
  trg_deg[3] = 50.0;
  trg_deg[4] = 0.0;*/
  trg_deg[0] = 0.0;
  trg_deg[1] = 0.0;
  trg_deg[2] = 0.0;
  trg_deg[3] = 0.0;
  trg_deg[4] = 0.0;
  trg_deg[5] = 0.0;
  trg_deg[6] = 0.0;

  ResourceFinder rf;
  rf.setVerbose(true);
  rf.setDefaultContext("cartesianForceWBC/conf");
  rf.setDefaultConfigFile("cartesianForceWBC.ini");
  rf.configure("ICUB_ROOT",argc,argv);


  Network yarp;
  if (!Network::checkNetwork()) {
      printf("Please start a yarp name server first\n");
      return(-1);
  }
  
  TrgPort trgPort;
  trgPort.open("/trgPort");
  trgPort.useCallback();

  qPort.open("/tao/state:o");

  Property options;
  options.put("device","remote_controlboard");
#ifdef REAL_ROBOT
  options.put("remote","/icub/left_arm");
#else
  options.put("remote","/icubSim/left_arm");
#endif
  options.put("local","/local");

#ifndef DO_NOT_CONNECT
  PolyDriver dd(options);
  if(!dd.isValid()) {
    printf("ravebot device not available.\n");
    dd.close();
    Network::fini();
    return 1;
  }

  dd.view(pos);
  double sp[7] = {100,100,100,100,100,100,100};
  pos->setRefSpeeds(sp);
#endif

  ConstString saixml=rf.findFile("saixml");
  const char *c_saixml(saixml.c_str());
  printf("Loading SAIxml from file '%s'...\n",c_saixml);
  try {
    model.reset(jspace::test::parse_sai_xml_file(c_saixml, true));
  }
  catch (std::runtime_error const & ee) {
    errx(EXIT_FAILURE, "%s", ee.what());
  }
  tutsim::set_robot_filename(c_saixml);
  return tutsim::run(servo_cb);
}
