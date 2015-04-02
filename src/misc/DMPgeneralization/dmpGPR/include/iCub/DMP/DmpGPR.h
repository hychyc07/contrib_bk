/* 
 * Copyright (C) 2012 Istituto Italiano di Tecnologia
 * Author: Elena Ceseracciu
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include"iCub/DMP/DMP.h"
//#include "iCub/DMP/DMPstructure.h"
#include "iCub/learningMachine/GPR.h"

class DmpGPR {
private:
    std::vector<iCub::learningmachine::GPRLearner> gprs;

    int dof_; // degrees of freedom for each DMP
    int N_; // number basis functions for non-linear component of DMPs
    int m_; // degrees of freedom for the target
    bool init;
    bool verbose;
    yarp::sig::Vector dmpConstants;
    std::string dmpType;
    int sampleCount;
    bool checkConstants(const dmp::DMP* y);
public:
    /**
     *  Create a new Gaussian Regression Process object for DMP generalization
     * \param[in] N number of basis functions (therefore, of weights) for non-linear component of DMPs
     * \param[in] dof number of degrees of freedom of the original trajectory
     * \param[in] m number of degrees of freedom of the target
     * \param[in] sf2 "prior" standard deviation of noise
     */
    DmpGPR(int N, int dof, int m, double sf2=1.0); //should add names or smt for covariance/mean functions?
    
//     /** 
//      * Default class constructor
//      */
//     DmpGPR();
    
    /**
     *  Class destructor
     */
    ~DmpGPR();
    
    /**
     *  Feed a new DMP example and relative target to the DmpGPR generalizer
     * \param[in] x "target" vector, with respect to which DMP parameters are learned
     * \param[in] y DMP the parameters of which are used as examples
     * \return true if example was successfully added to the training set (basically, sizes were consistent with DmpGPR parameters)
     */
    bool feedSample(const yarp::sig::Vector &x, const dmp::DMP* y);

    /**
    *  Reset the DmpGRP generalizer ( reset DMP time constants and remove all previous examples)
    */
    void reset();
    
    /**
     *  Feed DMP examples and relative targets to the DmpGPR generalizer, then train the learning machine.
     * \param[in] x list of "target" vector, with respect to which DMP parameters are learned
     * \param[in] y list of DMPs, the parameters of which are used as examples
     * \param[in] optimizeHyperparameters true if hyperparameters should be optimized, minimizing the log negative marginal likelihood
     * \return true if training process was completed successfully
     */
    bool inference(const std::vector <yarp::sig::Vector> & x, const std::vector<dmp::DMP*> & y, bool optimizeHyperparameters=false);
    
    /**
     * Train the learning machines with the examples provided so far
     * \return true if training process was completed successfully
     */
    bool inference();
    
    /**
     * Predict DMP parameters for each provided target
     * \param[in] xs list of targets
     * \return list of DMP with predicted parameters (weights & tau) //should add goal as well...?
     */
    dmp::DMP* generalize(const yarp::sig::Vector & xs);
    
    /**
     * Predict DMP parameters for each provided target
     * \param[in] xs list of targets
     * \return list of DMP with predicted parameters (weights & tau) //should add goal as well...?
     */
    std::vector<dmp::DMP*> generalize(const std::vector <yarp::sig::Vector> & xs);
    
    /**
     * Print information for each GPR learner to screen
     */
    void print();
    
    int getNumberOfSamples();

    friend class dmp::DMP;
};