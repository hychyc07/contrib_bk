
/**
   \file main7.cpp
   \author by Juan G Victores
   \author contrib Roland Philippsen (tut06_eepos.cpp of WBC)
   
   A quick test to see whether the simulator works as expected.

*/

#include "tutsim7.hpp"
#include <opspace/task_library.hpp>
#include <opspace/skill_library.hpp>
#include <opspace/ClassicTaskPostureController.hpp>
#include <jspace/test/sai_util.hpp>
#include <boost/shared_ptr.hpp>
#include <FL/fl_draw.H>
#include <err.h>


static std::string model_filename;
static boost::shared_ptr<jspace::Model> model;
static boost::shared_ptr<opspace::ClassicTaskPostureController> controller;
static boost::shared_ptr<opspace::GenericSkill> skill;
static boost::shared_ptr<opspace::CartPosTask> eetask;
static boost::shared_ptr<opspace::JPosTask> jtask;
static opspace::Parameter * eegoalpos;
static opspace::Parameter * eegoalvel;
static opspace::Parameter * jgoalpos;
static opspace::Parameter * jgoalvel;
static size_t mode;
static IPositionControl *ipos;


static bool servo_cb(size_t toggle_count,
		     double wall_time_ms,
		     double sim_time_ms,
		     jspace::State & state,
		     jspace::Vector & command) {

  mode = toggle_count % 2;
  static size_t prevmode(42);
  static size_t iteration(0);
  
  model->update(state);

  if (0 == mode) {
    
    prevmode = mode;
    return false;
    
  }
    
  if ((1 == mode) && (1 != prevmode)) {
    jspace::Status st(skill->init(*model));
    if ( ! st) {
      errx(EXIT_FAILURE, "skill->init() failed: %s", st.errstr.c_str());
    }
    st = controller->init(*model);
    if ( ! st) {
      errx(EXIT_FAILURE, "controller->init() failed: %s", st.errstr.c_str());
    }
  }

  // Try to maintain positions and velocities of the joint space
/*  jspace::Vector jpos(state.position_.rows());
  jpos << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  if ( ! jgoalpos->set(jpos)) {
    errx(EXIT_FAILURE, "failed to set joint goal position");
  }
  jspace::Vector jvel(state.velocity_.rows());
  jvel << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  if ( ! jgoalvel->set(jvel)) {
    errx(EXIT_FAILURE, "failed to set joint goal velocity");
  }*/

  jspace::Vector pos(3), vel(3);
  pos << 0.0577, 0.1239, -0.12;
//    pos << -0.1036, -0.0148, 0.0;
  if ( ! eegoalpos->set(pos)) {
    errx(EXIT_FAILURE, "failed to set end-effector goal position");
  }
  vel << 0, 0, 0;
  if ( ! eegoalvel->set(vel)) {
    errx(EXIT_FAILURE, "failed to set end-effector goal velocity");
  }


  if ( ! skill->update(*model)) {
    errx(EXIT_FAILURE, "skill update failed");
  }
  
  if ( ! controller->computeCommand(*model, *skill, command)) {
    errx(EXIT_FAILURE, "controller update failed");
  }
  
  if (0 == (iteration % 100)) {
    controller->dbg(std::cerr, "**************************************************", "");
  }
  
  ++iteration;
  prevmode = mode;
  
  return true;
}


static void draw_cb(double x0, double y0, double scale)
{
  if (0 != mode) {
    
    tutsim::draw_delta_jpos(*jgoalpos->getVector(), 1, 100, 80, 80, x0, y0, scale);
    
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
	    x0 + px * scale, y0 - py * scale);
  }
}


int main(int argc, char ** argv) {

  ResourceFinder rf;
  rf.setVerbose(true);
  rf.setDefaultContext("cartesianForceWBC/conf");
  rf.setDefaultConfigFile("cartesianForceWBC.ini");
  rf.configure("ICUB_ROOT",argc,argv);

  ConstString saixml=rf.findFile("saixml");
  const char *c_saixml(saixml.c_str());
  printf("Loading SAIxml from file '%s'...\n",c_saixml);

  try {
    
    model.reset(jspace::test::parse_sai_xml_file(c_saixml, true));
    
    eetask.reset(new opspace::CartPosTask("tut06-eepos"));
    jspace::Vector kp(1), kd(1), maxvel(1), ctrlpt(3);
    kp << 400.0;
    kd << 4.0;
    maxvel << 1.0;
    ctrlpt << 0.0, 0.0, 0.0;  // to set extra ee tool length // was << 0.0, 0.0, -1.0;
    eetask->quickSetup(kp, kd, maxvel, "link7", ctrlpt);
    eegoalpos = eetask->lookupParameter("goalpos", opspace::PARAMETER_TYPE_VECTOR);
    if ( ! eegoalpos) {
      errx(EXIT_FAILURE, "failed to find appropriate end-effector goalpos parameter");
    } 
    eegoalvel = eetask->lookupParameter("goalvel", opspace::PARAMETER_TYPE_VECTOR);
    if ( ! eegoalvel) {
      errx(EXIT_FAILURE, "failed to find appropriate end-effector goalvel parameter");
    }

    jtask.reset(new opspace::JPosTask("tut06-jtask"));
    kp << 10.0;
    kd << 0.01;
    maxvel << M_PI/5.0;
    jtask->quickSetup(kp, kd, maxvel);
    jgoalpos = jtask->lookupParameter("goalpos", opspace::PARAMETER_TYPE_VECTOR);
    if ( ! jgoalpos) {
      errx(EXIT_FAILURE, "failed to find appropriate joint-posture goalpos parameter");
    }
    jgoalvel = jtask->lookupParameter("goalvel", opspace::PARAMETER_TYPE_VECTOR);
    if ( ! jgoalvel) {
      errx(EXIT_FAILURE, "failed to find appropriate joint-posture goalvel parameter");
    }

    skill.reset(new opspace::GenericSkill("tut06-skill"));
    skill->appendTask(eetask);
    skill->appendTask(jtask);

    controller.reset(new opspace::ClassicTaskPostureController("tut06-ctrl"));
    
  }
  catch (std::runtime_error const & ee) {
    errx(EXIT_FAILURE, "%s", ee.what());
  }
  tutsim::set_draw_cb(draw_cb);
  tutsim::set_robot_filename(c_saixml);
  return tutsim::run(servo_cb);
}

