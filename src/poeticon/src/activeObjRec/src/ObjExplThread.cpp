#include "iCub/ObjExplThread.h"
#include "iCub/HandPoseUtil.h"
#include "iCub/Util.h"
#include "iCub/ObjRecModule.h"

bool ObjExplThread::threadInit() 
{
    if (! armCartDriver.view(armCart))
    {
        std::cout <<  "Cannot get cartesian interface to the arm" << std::endl;  
        armCartDriver.close();
        return false;
    }

    newObject = Object();

    explProgress = 0;
    keyframeFeatures.clear();
    explorationMap.clear();
    explViewpoints.clear();
    explKeyframes.clear();

    moduleName = rf->check("name", Value("activeObjRec"), "module name (string)").asString().c_str();

    DISTINCT_VIEW_THRESH = rf->check("distinctViewThreshold", Value(92.0), "max distance for new object candidate").asDouble();
    stopExplorationThresh = rf->check("stopExplorationThresh", Value(0.7), "objec  exploration is stopped when below threshold").asDouble();
    explorationGain = rf->check("explorationGain", Value(0.4)).asDouble();
    explorationGain = std::min(1.0, std::max(0.0, explorationGain));
    
    settings.minElevation = rf->check("minElevation", Value(0), "minimal elevation angle on view sphere").asInt();
    settings.maxElevation = rf->check("maxElevation", Value(90), "maximal elevation angle on view sphere").asInt();
    settings.minRotation  = rf->check("minRotation", Value(140), "minimal Rotation angle on view sphere").asInt();
    settings.maxRotation  = rf->check("maxRotation", Value(280), "maximal Rotation angle on view sphere").asInt();
    
    std::string defaultDataRoot = "./data";

    std::string defaultObjectViewDir = defaultDataRoot+"/objectViewDir";
    objectViewDir = rf->check("objectViewDir", Value(defaultObjectViewDir.c_str()), "Directory for storing object views").asString().c_str();
    std::cout << "Object view directory: " << objectViewDir << std::endl;
    
    // create output directory
    try
    {
        fs::create_directories(objectViewDir);
    }
    catch(std::exception &e)
    {
        std::cerr << "ERROR - ObjRecThread::setAction: Could not create object view directory " << objectViewDir << std::endl; 
        std::cerr << "\t ... " << e.what() << std::endl; 
        return false;
    }

    if (! isSimulation)
    {
        std::string explorationPortName = "/" + moduleName + "/";
        explorationPortName += rf->check("explorationPort", Value("explorationStatus:o"), 
                "sends 1 when object exploration completed").asString().c_str();
        if (! explorationPort.open(explorationPortName.c_str())) 
        {
            std::cout << "unable to open port " << explorationPortName << std::endl;
            return false;
        }
    }

    keyframeExtractor.init(); 

    module->handGazeControl->stopControl();

    // Definition of exploration path for object learning
    explPath.push_back(cv::Vec2f(0,0));
    explPath.push_back(cv::Vec2f(90,160));
    explPath.push_back(cv::Vec2f(90,210));
    explPath.push_back(cv::Vec2f(90,240));
    explPath.push_back(cv::Vec2f(90,260));
    explPath.push_back(cv::Vec2f(90,280));
    explPath.push_back(cv::Vec2f(70,280));
    explPath.push_back(cv::Vec2f(70,260));
    explPath.push_back(cv::Vec2f(70,240));
    explPath.push_back(cv::Vec2f(70,210));
    explPath.push_back(cv::Vec2f(70,180));
    explPath.push_back(cv::Vec2f(50,180));
    explPath.push_back(cv::Vec2f(50,210));
    explPath.push_back(cv::Vec2f(50,240));
    explPath.push_back(cv::Vec2f(50,260));
    explPath.push_back(cv::Vec2f(50,280));
    explPath.push_back(cv::Vec2f(40,280));
    explPath.push_back(cv::Vec2f(40,260));
    explPath.push_back(cv::Vec2f(40,240));
    explPath.push_back(cv::Vec2f(40,210));
    explPath.push_back(cv::Vec2f(40,180));
    explPath.push_back(cv::Vec2f(30,180));
    explPath.push_back(cv::Vec2f(30,210));
    explPath.push_back(cv::Vec2f(30,240));
    explPath.push_back(cv::Vec2f(30,260));
    explPath.push_back(cv::Vec2f(30,280));
    explPath.push_back(cv::Vec2f(0,0));

    CV_Assert(DISTINCT_VIEW_THRESH >= 0);

    return true;
}


void ObjExplThread::threadRelease() 
{
    explorationPort.interrupt();
    explorationPort.close();
}


void ObjExplThread::run() 
{
    exploreObject();
}



void ObjExplThread::exploreObject()
{
    // get current view
    cv::Mat currentImage = module->getCameraImage();
    if (currentImage.empty())
        return;

    double currentElevation, currentRotation;
    module->getHandGazeAngles(currentElevation, currentRotation);

    explorationMap.markGaze(currentElevation, currentRotation);
    explorationMap.show("Exploration Map", 5);

    //int width = 14;
    //std::cout << std::setw(width) << std::left << "e/r:" 
       // << currentElevation << "°\t" << currentRotation << "°" << std::endl;

    // record keyframes
    if (keyframeExtractor.isKeyframe(currentImage))
    {
        // check if view in different enough from previous views
        cv::Mat ftFourier  = FeatureExtractor::extract(currentImage, FeatureExtractor::FOURIER);
        for (int i = 0; i < keyframeFeatures.size(); i++)
        {
            double sim = FeatureExtractor::compareFeatures(ftFourier, keyframeFeatures[i]);
            //std::cout << "Keyframe similarity: " << sim << std::endl;
            if (sim > DISTINCT_VIEW_THRESH)
            {
                // too similar -> dont save
                std::cout << "Discarding keyframe. Similarity = " << sim << std::endl;
                return; 
            }
        }

        std::cout << "Found keyview " << keyframeFeatures.size()  << std::endl;

        keyframeFeatures.push_back(ftFourier);

        // get current configuration
        double e, r;
        module->getHandGazeAngles(e, r);

        newObject.addKeyview(currentImage, e, r);

        cv::Mat ftComposite  = FeatureExtractor::extract(currentImage, FeatureExtractor::COMPOSITE);
        explKeyframes.push_back(ftComposite);
   
        explViewpoints.push_back(cv::Vec2f(currentElevation, currentRotation));

        int nKf = (int)explKeyframes.size();

        cv::Mat distMat(nKf, nKf, CV_32F);
        for (int i = 0; i < nKf; i++)
        {
            for (int j = i; j < nKf; j++)
            {
                double d = cv::norm(explKeyframes[i], explKeyframes[j], cv::NORM_L2)/(explKeyframes[i].cols);
                //d = std::exp(-10*d);
                distMat.at<float>(i,j) = d;
                distMat.at<float>(j,i) = d;
            }
        }

        cv::Mat saliency(nKf,1, CV_32F);
        for (int i = 0; i < nKf; i++)
        {
            saliency.at<float>(i) = cv::sum(distMat.row(i))[0];
        }

        cv::Mat plot(settings.maxElevation, settings.maxRotation, CV_32F, cv::Scalar::all(0));

        // set point radius proportional to particle weight 
        double maxRadius = 3; // best particle will be drawn that large
        //cv::Mat particleRadius;
        cv::normalize(saliency, saliency, 0, 1, cv::NORM_MINMAX);
        saliency *= maxRadius;

        for (int i = 0; i < nKf; i++)
        {
            cv::Point pt(explViewpoints[i][1], explViewpoints[i][0]);
            //cv::Scalar color = cv::Scalar(rand()%255, rand()%255, rand()%255);
            cv::Scalar color = cv::Scalar(255,0,0); // blue
            cv::circle(plot, pt, saliency.at<float>(i), color, -1, CV_AA);
        }

        cv::imshow("kf saliency", plot);
        //cv::waitKey();


        double avg = cv::sum(saliency.rowRange(0,nKf-1))[0]/(nKf-1);
        if (saliency.at<float>(nKf-1) > avg)
            explorationGain = 0.50;
        else
            explorationGain = 0.65;
        
        explorationGain = std::min(std::max(explorationGain, 0.0), 1.0);
        std::cout << "expGain: " <<  explorationGain << std::endl;
    }
    
    // find new postion to go to
    if (!module->handGazeControl->targetReached())
    {
        return;
    }

    int nextElevation = 0;
    int nextRotation = 0;

    int stepE = 1;
    int stepR = 1;
    double bestTarget = 1000;
    double minExpValue = 1000;
    for (int e = settings.minElevation; e < settings.maxElevation; e += stepE)
    {
        for (int r = settings.minRotation; r < settings.maxRotation; r += stepR)
        {
            double expValue = explorationMap.getViewValue(e, r);
            if (expValue < minExpValue) 
            {
                minExpValue = expValue;
            }
            double dist = Util::calcCentralAngle(currentElevation, e, currentRotation, r);
            //if (dist < 10) // turn at least 10 deg
                //continue;
            double f = explorationGain*expValue + (1.0-explorationGain)*(dist/180.0);
            if (f == 0.0)
                continue;
            if (f < bestTarget)
            {
                bestTarget = f;
                nextElevation = e;
                nextRotation = r;
            }
        }
    }

    std::cout << "minExpValue/stop: " << minExpValue << " / " << stopExplorationThresh << std::endl;

#ifdef PATH_EXPLORE
    // stop exploration if there are no unexplored regions left
    if (explProgress >= explPath.size())
    //if (minExpValue > stopExplorationThresh)
    {
        std::cout << "\nExploration of object " << objectName << " completed." << std::endl;
       
        // send message 
        Bottle& msg = explorationPort.prepare();
        msg.clear();
        msg.addInt(1);
        explorationPort.write();
        setAction(IDLE);
        return;
    }
    else
    {
        nextElevation = explPath[explProgress][0];
        nextRotation = explPath[explProgress][1];
        explProgress++;
    }
#endif

    if (minExpValue > stopExplorationThresh)
    {
        std::cout << "\nExploration of object " << objectName << " completed." << std::endl;
        std::cout << newObject.getKeyviewCount() << " keyviews recorded." << std::endl;

        // save data
        fs::path objectDir = fs::path(objectViewDir) / objectName;
        if (! newObject.save(objectDir.string()))
            std::cerr << "ERROR: Object data could not be saved!" << std::endl;

        // send message 
        Bottle& msg = explorationPort.prepare();
        msg.clear();
        msg.addInt(1);
        explorationPort.write();

        // done, stop thread
        stop();

        return;
    }

    //cout << "bestTarget: " << bestTarget << endl;
    //cout << "next target: " << nextElevation << " " << nextRotation << endl;
    //cout << "dist: " << std::sqrt(SQR(currentElevation-nextElevation)+SQR(currentRotation-nextRotation));
    //cout << "expValue: " << explorationMap.getViewValue(nextElevation, nextRotation) << endl;

    // goto position
    std::cout << "lookat  " << nextElevation << " " << nextRotation << std::endl;
    std::cout << "current " << currentElevation << " " << currentRotation << std::endl;
    module->handGazeControl->lookAtViewpoint(nextElevation, nextRotation);

    //std::cout << "markGaze" << std::endl;
    explorationMap.markGaze(nextElevation, nextRotation);
}

