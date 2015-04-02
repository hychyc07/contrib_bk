// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
/**
* \defgroup icub_controlbasis_libraries Control Basis Libraries
* @ingroup icub_controlbasis_libraries
*
*/
#ifndef _HEADING_SAMPLE__H_
#define _HEADING_SAMPLE__H_

#include <vector>
#include <deque>

#include "Heading.h"

#include "GaussianProbabilityDistribution.h"
#include "NonParametricProbabilityDistribution.h"

namespace CB {
    
    /**
     * Implements the Heading abstract interface to return 
     * a sampled (\gamma_u,\gamma_v) coordinates drawn from 
     * a distibution of the type specified
     **/
    class HeadingSample : public Heading {

    public:
        /**
         * distribution types
         **/
        enum distributionType {
            GAUSSIAN=0,
            NONPARAMETRIC
        };
        
    protected:
        
        /**
         * the type of distribution
         **/
        distributionType distType;

        /**
         * the distribution
         **/
        ProbabilityDistribution *dist;

    public:
        
        /**
         * Constructor
         */
        HeadingSample(std::string name, distributionType distT, std::string fName="") :
            distType(distT),
            dist(NULL)
        {
            
            deviceName=name;
            numInputs=0;
            setRate(100);
            //            updateDelay=0.1;
            valid=false;
            values.resize(2);
            
            switch(distType) {
            case GAUSSIAN:
                dist = new GaussianProbabilityDistribution(2);
                break;
            case NONPARAMETRIC:
                dist = new NonParametricProbabilityDistribution(2);
                break;            
            default:
                std::cout << "unkown probability distribution type!!" << std::endl;
                break;
            }
            
            if(fName != "") {
                if(dist->loadDistribution(fName)) {
                    int s = dist->getDimensionCount();
                    if(s!=2) {
                        std::cout << "probability distribution loaded from file \'" << fName.c_str() << "\' of incorrect size: " << s << std::endl;
                    }
                }
            }

        }

        /**
         * Constructor
         */
        HeadingSample(std::string name, std::string fName) :
            dist(NULL)
        {
            
            deviceName=name;
            numInputs=0;
            setRate(100);
            //            updateDelay=0.1;
            valid=false;
            values.resize(2);
            
            distType = getDistributionTypeFromFile(fName);
                
            switch(distType) {
            case GAUSSIAN:
                dist = new GaussianProbabilityDistribution(2);
                break;
            case NONPARAMETRIC:
                dist = new NonParametricProbabilityDistribution(2);
                break;            
            default:
                std::cout << "unkown probability distribution type!!" << std::endl;
                break;
            }

            if(dist->loadDistribution(fName)) {
                int s = dist->getDimensionCount();
                if(s!=2) {
                    std::cout << "probability distribution loaded from file \'" << fName.c_str() << "\' of incorrect size: " << s << std::endl;
                }
            }           

        }

        /** 
         * Destructor
         **/
        ~HeadingSample() { }
        
        /**
         * Inherited update function.
         * \returns success on update
         **/
        bool updateResource();

        /**
         * Inherited start function.
         **/
        void startResource();

        /**
         * Inherited stop function.
         **/
        void stopResource();
        
        /**
         * A functon that draws a new sample from the distribution
         **/
        bool drawSample();    

        /**
         * load distribution from file
         **/
        bool loadDistributionFromFile(std::string f);

        /**
         * retrieves distribution type from file
         **/
        distributionType getDistributionTypeFromFile(std::string f);


    };


}

#endif
