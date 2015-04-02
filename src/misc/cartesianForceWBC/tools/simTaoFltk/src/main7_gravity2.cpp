
/**
   \file main7.cpp
   \author by Juan G Victores
   \author contrib Roland Philippsen (tut03_gravity_compensation.cpp of WBC)

   A quick test to see whether the simulator works as expected.

*/

#include "tutsim7.hpp"
#include <opspace/Task.hpp>
#include <jspace/test/sai_util.hpp>
#include <boost/shared_ptr.hpp>
#include <err.h>

static IPositionControl *pos;


namespace tut03 {

class JTask : public opspace::Task {
public:
    JTask() : opspace::Task("tut03::JTask"), mode_(0) {}

    virtual jspace::Status init(jspace::Model const & model) {
        //////////////////////////////////////////////////
        // The Jacobian is not really required for pure jspace control,
        // but will become very important for operational-space control.

        jacobian_ = jspace::Matrix::Identity(model.getNDOF(), model.getNDOF());

        //////////////////////////////////////////////////
        // Initialize our PD parameters.

        kp_ = jspace::Vector::Zero(model.getNDOF());
        kd_ = jspace::Vector::Zero(model.getNDOF());

        kp_[0] = 0.7;
        kp_[1] = 0.7;
        kp_[2] = 0.7;
        kp_[3] = 0.7;
        kp_[4] = 0.7;
        kp_[5] = 0.7;
        kp_[6] = 0.7;

        kd_[0] = 0.04;
        kd_[1] = 0.04;
        kd_[2] = 0.04;
        kd_[3] = 0.04;
        kd_[4] = 0.04;
        kd_[5] = 0.04;
        kd_[6] = 0.04;


        kp_M_ = jspace::Vector::Zero(model.getNDOF());
        kd_M_ = jspace::Vector::Zero(model.getNDOF());

        // No NOT ever go above 2.0 in the following
        kp_M_[0] = 0.5;
        kp_M_[1] = 0.5;
        kp_M_[2] = 0.5;
        kp_M_[3] = 0.5;
        kp_M_[4] = 0.5;
        kp_M_[5] = 0.5;
        kp_M_[6] = 0.5;

        // Do NOT move the following values from 0.01
        kd_M_[0] = 1.200;
        kd_M_[1] = 1.200;
        kd_M_[2] = 1.200;
        kd_M_[3] = 1.200;
        kd_M_[4] = 1.200;
        kd_M_[5] = 1.200;
        kd_M_[6] = 1.200;

        //////////////////////////////////////////////////
        // Initialize our goal to the current configuration.

        goal1_ = model.getState().position_;
        goal1_[0] = -28.0 * M_PI / 180.0;
        goal1_[1] = 80.0 * M_PI / 180.0;
        goal1_[2] = 15.0 * M_PI / 180.0;
        goal1_[3] = 50.0 * M_PI / 180.0;
        goal1_[4] = 0.0 * M_PI / 180.0;
        goal1_[5] = 0.0 * M_PI / 180.0;
        goal1_[6] = 0.0 * M_PI / 180.0;

        goal2_ = model.getState().position_;
        goal2_[0] = -32.0 * M_PI / 180.0;
        goal2_[1] = 20.0 * M_PI / 180.0;
        goal2_[2] = 25.0 * M_PI / 180.0;
        goal2_[3] = 45.0 * M_PI / 180.0;
        goal2_[4] = 0.0 * M_PI / 180.0;
        goal2_[5] = 0.0 * M_PI / 180.0;
        goal2_[6] = 0.0 * M_PI / 180.0;

        //////////////////////////////////////////////////
        // No initialization problems to report: the default constructor
        // of jspace::Status yields an instance that signifies success.

        jspace::Status ok;
        return ok;
    }


    virtual jspace::Status update(jspace::Model const & model) {

        //////////////////////////////////////////////////
        // Update the state of our task. Again, this is not critical
        // here, but important later when we want to integrate several
        // operational space tasks into a hierarchy.

        actual_ = model.getState().position_;

        //////////////////////////////////////////////////
        // Compute PD control torques and store them in command_ for
        // later retrieval. If enabled, add the estimated effect of
        // gravity in order to make the robot behave as if was
        // weightless.

        // Just a test for now
        if ( ! model.getMassInertia(mass_inertia_)) {
            return jspace::Status(false, "failed to retrieve mass matrix");
        }

        if (mode_ == 0) {
            command_ = jspace::Vector::Zero(model.getNDOF());
        } else if (mode_ == 1) {
            command_ = kp_.cwise() * (goal1_ - actual_) - kd_.cwise() * model.getState().velocity_;
        } else if (mode_ == 2) {
            command_ = kp_.cwise() * (goal2_ - actual_) - kd_.cwise() * model.getState().velocity_;
        } else if (mode_ == 3) {
            command_ = mass_inertia_ * (kp_M_.cwise() * (goal1_ - actual_) - kd_M_.cwise() * model.getState().velocity_);
        } else if (mode_ == 4) {
            command_ = mass_inertia_ * (kp_M_.cwise() * (goal2_ - actual_) - kd_M_.cwise() * model.getState().velocity_);
        } else {
            command_ = jspace::Vector::Zero(model.getNDOF());
            printf("ERROR UNKNOWN MODE, fallback to gravity compensation\n");
        }

        jspace::Vector gg;
        if ( ! model.getGravity(gg)) {
            return jspace::Status(false, "failed to retrieve gravity torque");
        }

        command_ += gg;
//        command_[5]=0;
//        command_[6]=0;

        jspace::Status ok;
        return ok;
    }

    int mode_;

    jspace::Vector kp_;
    jspace::Vector kd_;

    jspace::Vector kp_M_;
    jspace::Vector kd_M_;
    jspace::Vector goal1_;
    jspace::Vector goal2_;

    jspace::Matrix mass_inertia_;

};

}

static boost::shared_ptr<jspace::Model> model;
static boost::shared_ptr<tut03::JTask> jtask;
static size_t mode(0);

static bool servo_cb(size_t toggle_count, double wall_time_ms,
                     double sim_time_ms, jspace::State & state,
                     jspace::Vector & command) {

    mode = toggle_count % 5;
    static bool initialized(false);
    static size_t iteration(0);

    if (!initialized) {
        state.position_[0] = -28.0 * M_PI / 180.0;
        state.position_[1] = 80.0 * M_PI / 180.0;
        state.position_[2] = 15.0 * M_PI / 180.0;
        state.position_[3] = 50.0 * M_PI / 180.0;
        state.position_[4] = 0.0 * M_PI / 180.0;
        state.position_[5] = 0.0 * M_PI / 180.0;
        state.position_[6] = 0.0 * M_PI / 180.0;
        state.velocity_ = jspace::Vector::Zero(state.position_.rows());
        command = jspace::Vector::Zero(state.position_.rows());
        model->update(state);
        jtask->init(*model);
        initialized=true;
        return false;
    }

    //////////////////////////////////////////////////
    // Update the model to reflect the current robot state.
        state.position_[5] = 0.0 * M_PI / 180.0;
        state.position_[6] = 0.0 * M_PI / 180.0;

    model->update(state);

    //////////////////////////////////////////////////
    // Run the jtask, but re-initialize it whenever we start a new
    // cycle of trials, and switch gravity compensation on/off to
    // illustrate its effects.

    jtask->mode_ = mode;

    jtask->update(*model);
    command = jtask->getCommand();

    //////////////////////////////////////////////////
    // Print debug info from time to time.

    if (0 == (iteration % 100)) {

        switch (mode) {
        case 0:
            std::cerr << "mode: pure gravity compensation\n";
            jspace::pretty_print(jtask->goal1_, std::cerr, "  goal", "    ");
            break;
        case 1:
            std::cerr << "mode: jtask with goal1\n";
            jspace::pretty_print(jtask->goal1_, std::cerr, "  goal", "    ");
            break;
        case 2:
            std::cerr << "mode: jtask with goal2\n";
            jspace::pretty_print(jtask->goal2_, std::cerr, "  goal", "    ");
            break;
        case 3:
            std::cerr << "mode: jtask with goal1 using M\n";
            jspace::pretty_print(jtask->goal1_, std::cerr, "  goal", "    ");
            break;
        case 4:
            std::cerr << "mode: jtask with goal2 using M\n";
            jspace::pretty_print(jtask->goal2_, std::cerr, "  goal", "    ");
            break;
        default:
            std::cerr << "[WARNING] Non-contemplated case\n";
            break;
        }
        jspace::pretty_print(state.position_, std::cerr, "  jpos", "    ");
        jspace::pretty_print(state.velocity_, std::cerr, "  jvel", "    ");

        if ((mode==3)||(mode==4))
            jspace::pretty_print(jtask->mass_inertia_, std::cerr, "  mass matrix", "    ");

        jspace::pretty_print(command, std::cerr, "  command", "    ");
    }
    ++iteration;

    return true;
}


static void draw_cb(double x0, double y0, double scale) {
    if ((mode==1)||(mode==3)) {
        tutsim::draw_robot(jtask->goal1_, 2, 100, 80, 80, x0, y0, scale);
        tutsim::draw_delta_jpos(jtask->goal1_, 1, 120, 120, 80, x0, y0, scale);
    } else if ((mode==2)||(mode==4)) {
        tutsim::draw_robot(jtask->goal2_, 2, 100, 80, 80, x0, y0, scale);
        tutsim::draw_delta_jpos(jtask->goal2_, 1, 120, 120, 80, x0, y0, scale);
    }
}


int main(int argc, char ** argv) {

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("cartesianForceWBC/conf");
    rf.setDefaultConfigFile("cartesianForceWBC.ini");
    rf.configure("ICUB_ROOT",argc,argv);

    /*
      Network yarp;
      if (!Network::checkNetwork()) {
          printf("Please start a yarp name server first\n");
          return(-1);
      }

      Property options;
      options.put("device","remote_controlboard");
      options.put("remote","/icubSim/left_arm");
      options.put("local","/local");

      PolyDriver dd(options);
      if(!dd.isValid()) {
        printf("ravebot device not available.\n");
        dd.close();
        Network::fini();
        return 1;
      }

      dd.view(pos);
    */
    ConstString saixml=rf.findFile("saixml");
    const char *c_saixml(saixml.c_str());
    printf("Loading SAIxml from file '%s'...\n",c_saixml);

    try {
        model.reset(jspace::test::parse_sai_xml_file(c_saixml, true));
        jtask.reset(new tut03::JTask());
    }
    catch (std::runtime_error const & ee) {
        errx(EXIT_FAILURE, "%s", ee.what());
    }
    tutsim::set_draw_cb(draw_cb);
    tutsim::set_robot_filename(c_saixml);
    return tutsim::run(servo_cb);
}
