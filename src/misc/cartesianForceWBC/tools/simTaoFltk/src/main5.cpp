
/**
   \file main5.cpp
   \author by Juan G Victores
   \author contrib Roland Philippsen (tut00_test.cpp of WBC)
   
   A quick test to see whether the simulator works as expected.

*/

#include "tutsim5.hpp"
#include <opspace/task_library.hpp>
#include <opspace/skill_library.hpp>
#include <opspace/ClassicTaskPostureController.hpp>
#include <jspace/test/sai_util.hpp>
#include <boost/shared_ptr.hpp>
#include <FL/fl_draw.H>
#include <err.h>

static boost::shared_ptr<jspace::Model> model;
static boost::shared_ptr<opspace::ClassicTaskPostureController> controller;
static boost::shared_ptr<opspace::GenericSkill> skill;
static boost::shared_ptr<opspace::CartPosTask> eetask;
//static boost::shared_ptr<opspace::JPosTask> jtask;
static boost::shared_ptr<opspace::JointLimitTask> jltask;
static opspace::Parameter * eegoalpos;
static opspace::Parameter * eegoalvel;
//static opspace::Parameter * jgoalpos;
//static opspace::Parameter * jgoalvel;
static opspace::Parameter * jldt_seconds;
static size_t mode;
static IPositionControl *ipos;

static bool servo_cb(size_t toggle_count,
		     double wall_time_ms,
		     double sim_time_ms,
		     jspace::State & state,
		     jspace::Vector & command) {

  static size_t iteration(0);

  model->update(state);  // Update the model to reflect the current robot state.
  jspace::pretty_print(model->getState().position_, std::cerr, "model.getState().position_", " ");

  jldt_seconds->set(1.0+sim_time_ms/1000.0);    //sim_time_ms

//    ++iteration;
//    printf("iteration: %d\n",iteration);
  static size_t prev_toggle(1234);
  mode = toggle_count % 5;
  if (prev_toggle != toggle_count) {  // ONLY ON CLICK
    printf("toggle_count: %d\n",toggle_count);
    prev_toggle = toggle_count;
    printf("mode: %d\n",mode);
    if (toggle_count == 1) {
      jspace::Status st(skill->init(*model));
      if ( ! st) {
        errx(EXIT_FAILURE, "skill->init() failed: %s", st.errstr.c_str());
      } else printf("[success] skill->init()\n");
      st = jltask->init(*model);
      if ( ! st) {
        errx(EXIT_FAILURE, "jltask->init() failed: %s", st.errstr.c_str());
      } else printf("[success] jltask->init()\n");
      st = controller->init(*model);
      if ( ! st) {
        errx(EXIT_FAILURE, "controller->init() failed: %s", st.errstr.c_str());
      } else printf("[success] controller->init()\n");
    }
  }

  if(mode==1) {
    static jspace::Vector pos(3), vel(3);
    pos << 0.1, 0.1, -0.05;  // pos << 0.0, amp * sin(py), amp * sin(pz);
    vel << 0.0,	0.0, 0.0;  // vel << 0.01 * amp * cos(py), 0.01 * amp * cos(py), 0.01 * amp * cos(pz);
    if ( ! eegoalpos->set(pos)) {
      errx(EXIT_FAILURE, "failed to set end-effector goal position");
    }
    if ( ! eegoalvel->set(vel)) {
      errx(EXIT_FAILURE, "failed to set end-effector goal velocity");
    }

/*    jspace::Vector jpos(state.position_.rows());
    jspace::Vector jvel(state.velocity_.rows());
    for (int ii(0); ii < state.position_.rows(); ++ii) {
      jvel[ii]=0;
      jpos[ii]=0;
    }*/
  }

  if(mode>0){

    if ( ! skill->update(*model)) {
      errx(EXIT_FAILURE, "skill update failed");
    }

    jltask->dbg(std::cout,"DBG","    ");
    jltask->update(*model);

//    model->computeJacobian(model->getNode(4),jltask->jacobian_);

    jltask->dbg(std::cout,"DBG","    ");
    printf("HERE1\n");
    if ( ! controller->computeCommand(*model, *skill, command)) {
      errx(EXIT_FAILURE, "controller update failed");
    }
    printf("HERE2\n");
  } else command = jspace::Vector::Zero(5);

  return true;
}


static void draw_cb(double x0, double y0, double scale) {
  if (0 != mode) {
    
/*    tutsim::draw_delta_jpos(*jgoalpos->getVector(), 1, 100, 80, 80, x0, y0, scale);
    
    //////////////////////////////////////////////////
    // Remember: we plot the YZ plane, X is sticking out of the screen
    // but the robot is planar anyway.
    
    fl_color(255, 100, 100);
    fl_line_style(FL_SOLID, 1, 0);
    
    double const gx(eegoalpos->getVector()->y());
    double const gy(eegoalpos->getVector()->z());
    int const rr(ceil(0.15 * scale/9.0));
    int const dd(2 * rr);
    fl_arc(int(x0 + gx * scale) - rr, int(y0 - gy * scale) - rr, dd, dd, 0.0, 360.0);
    
    double const vx(eegoalvel->getVector()->x());
    double const vy(eegoalvel->getVector()->y());
    double const px(gx + vx * 0.2);
    double const py(gy + vy * 0.2);
    fl_line(x0 + (gx + 0.02) * scale, y0 - gy * scale,
	    x0 + (gx - 0.02) * scale, y0 - gy * scale);
    fl_line(x0 + gx * scale, y0 - (gy + 0.02) * scale,
	    x0 + gx * scale, y0 - (gy - 0.02) * scale);
    fl_color(255, 255, 100);
    fl_line(x0 + gx * scale, y0 - gy * scale,
	    x0 + px * scale, y0 - py * scale);*/
  }
}

int main(int argc, char ** argv) {
  
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
  
/*  Property options;
  options.put("device","remote_controlboard");
  options.put("remote","/icubSim/left_arm");
  options.put("local","/local");

  PolyDriver dd(options);
  if(!dd.isValid()) {
    printf("robot device not available.\n");
    dd.close();
    Network::fini();
    return 1;
  }

  dd.view(ipos);*/
  ConstString saixml=rf.findFile("saixml-5dof");
  const char *c_saixml(saixml.c_str());
  printf("Loading SAIxml from file '%s'...\n",c_saixml);

  try {

    model.reset(jspace::test::parse_sai_xml_file(c_saixml, true));

    eetask.reset(new opspace::CartPosTask("jtut2-eepos"));
    jspace::Vector kp(1), kd(1), maxvel(1), ctrlpt(3);
    kp << 0.2;  // was 400.0
    kd << 0.1;
    maxvel << 0.10;
    ctrlpt << 0.0, 0.0, 0.0;  // to set extra ee tool length // was << 0.0, 0.0, -1.0;
    eetask->quickSetup(kp, kd, maxvel, "link5", ctrlpt);
    eegoalpos = eetask->lookupParameter("goalpos", opspace::PARAMETER_TYPE_VECTOR);
    if ( ! eegoalpos) {
      errx(EXIT_FAILURE, "failed to find appropriate end-effector goalpos parameter");
    } 
    eegoalvel = eetask->lookupParameter("goalvel", opspace::PARAMETER_TYPE_VECTOR);
    if ( ! eegoalvel) {
      errx(EXIT_FAILURE, "failed to find appropriate end-effector goalvel parameter");
    }

    jltask.reset(new opspace::JointLimitTask("jtut2-jltask"));
    jldt_seconds = jltask->lookupParameter("dt_seconds", opspace::PARAMETER_TYPE_REAL);
    if ( ! jldt_seconds) {
      errx(EXIT_FAILURE, "failed to find appropriate jl dt_seconds parameter");
    }
    opspace::Parameter * jlparam;
    jspace::Vector jll,jlu;
    model->getJointLimits(jll,jlu);
    jlu = (180.0/M_PI)*jlu;  // Because later we need upper_stop_deg, etc.
    jll = (180.0/M_PI)*jll;  // Because later we need lower_stop_deg, etc.
    jspace::pretty_print(jlu, std::cerr, "jlu", " ");  // Tested, works to multiply all elements
    jspace::pretty_print(jll, std::cerr, "jll", " ");  // Tested, works to multiply all elements

    jlparam = jltask->lookupParameter("upper_stop_deg", opspace::PARAMETER_TYPE_VECTOR);
    if ( ! jlparam) {
      errx(EXIT_FAILURE, "failed to find appropriate jl upper_stop_deg parameter");
    }
    jlparam->set(jlu);
    jlparam = jltask->lookupParameter("upper_trigger_deg", opspace::PARAMETER_TYPE_VECTOR);
    if ( ! jlparam) {
      errx(EXIT_FAILURE, "failed to find appropriate jl upper_trigger_deg parameter");
    }
    jlparam->set(jlu);
    jlparam = jltask->lookupParameter("lower_stop_deg", opspace::PARAMETER_TYPE_VECTOR);
    if ( ! jlparam) {
      errx(EXIT_FAILURE, "failed to find appropriate jl lower_stop_deg parameter");
    }
    jlparam->set(jll);
    jlparam = jltask->lookupParameter("lower_trigger_deg", opspace::PARAMETER_TYPE_VECTOR);
    if ( ! jlparam) {
      errx(EXIT_FAILURE, "failed to find appropriate jl lower_trigger_deg parameter");
    }
    jlparam->set(jll);

    jspace::Vector jlmaxvel(5), jlmaxacc(5), jlkp(5), jlkd(5);
    jlparam = jltask->lookupParameter("maxvel", opspace::PARAMETER_TYPE_VECTOR);
    if ( ! jlparam) {
      errx(EXIT_FAILURE, "failed to find appropriate jl maxvel parameter");
    }
    jlmaxvel<<1.0,1.0,1.0,1.0,1.0;
    jlparam->set(jlmaxvel); 
    jlparam = jltask->lookupParameter("maxacc", opspace::PARAMETER_TYPE_VECTOR);
    if ( ! jlparam) {
      errx(EXIT_FAILURE, "failed to find appropriate jl maxacc parameter");
    }
    jlmaxacc<<1.0,1.0,1.0,1.0,1.0;
    jlparam->set(jlmaxacc); 
    jlparam = jltask->lookupParameter("kp", opspace::PARAMETER_TYPE_VECTOR);
    if ( ! jlparam) {
      errx(EXIT_FAILURE, "failed to find appropriate jl kp parameter");
    }
    jlkp<<1.0,1.0,1.0,1.0,1.0;
    jlparam->set(jlkp); 
    jlparam = jltask->lookupParameter("kd", opspace::PARAMETER_TYPE_VECTOR);
    if ( ! jlparam) {
      errx(EXIT_FAILURE, "failed to find appropriate jl kd parameter");
    }
    jlkd<<1.0,1.0,1.0,1.0,1.0;
    jlparam->set(jlkd);

//    jldt_seconds->set(1.0);    //sim_time_ms

    skill.reset(new opspace::GenericSkill("jtut2-skill"));
    skill->appendTask(eetask);
    skill->appendTask(jltask);

    controller.reset(new opspace::ClassicTaskPostureController("jtut2-ctrl"));

  } catch (std::runtime_error const & ee) {
    errx(EXIT_FAILURE, "%s", ee.what());
  }
  tutsim::set_robot_filename(c_saixml);
  tutsim::set_draw_cb(draw_cb);
  return tutsim::run(servo_cb);
}

