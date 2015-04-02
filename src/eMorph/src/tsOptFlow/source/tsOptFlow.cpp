/* 
 * Copyright (C) <year> RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author Charles Clercq
 * email:   charles.clercq@robotcub.org
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/
#include "tsOptFlow.hpp"

using namespace std; 
using namespace emorph::ebuffer;
using namespace yarp::sig;
using namespace yarp::os;
tsOptFlow::tsOptFlow(uint &_h, uint &_w, string &_src, uint &_type, uint &_acc, uint &_bin, double &_th, uint &_nn, uint &_ssz, uint &_tsval, double &_a, double &_td, int &_pol, std::string &_eye, bool &_ori, bool &_save, yarp::os::BufferedPort<VelocityBuffer>* _port)
:polarity(_pol)
{
    vxMat = new Matrix(_h, _w);
    vyMat = new Matrix(_h, _w);
    mutex = new Semaphore();
    velBuf= new VelocityBuffer();

    if(polarity!=-1)
    {
        if(polarity==2)
            _pol=1;
        tsofThreadPos=new tsOptFlowThread(_h, _w, _src, _type, _acc, _bin, _th, _nn, _ssz, _tsval, _a, _td, _pol, _eye, _ori, _save, vxMat, vyMat, mutex, velBuf);
    }
    if(polarity==-1 || polarity==2)
    {
        if(polarity==2)
            _pol=-1;
        tsofThreadNeg=new tsOptFlowThread(_h, _w, _src, _type, _acc, _bin, _th, _nn, _ssz, _tsval, _a, _td, _pol, _eye, _ori, _save, vxMat, vyMat, mutex, velBuf);
    }
    else
        tsofThreadPos=new tsOptFlowThread(_h, _w, _src, _type, _acc, _bin, _th, _nn, _ssz, _tsval, _a, _td, _pol, _eye, _ori, _save, vxMat, vyMat, mutex, velBuf);
        
    sendvelbuf=new sendVelBuf(velBuf, mutex, _port, _acc);
    if(polarity!=-1)
        tsofThreadPos->start();
    if(polarity==-1 || polarity==2)
        tsofThreadNeg->start();
    sendvelbuf->start();
}

tsOptFlow::~tsOptFlow()
{
    if(polarity!=-1)
    {
        tsofThreadPos->stop();
        delete tsofThreadPos;
    }    
    if(polarity==-1 || polarity==2)
    {
        tsofThreadNeg->stop();
        delete tsofThreadNeg;
    }

    delete vxMat;
    delete vyMat;
    delete mutex;
    delete velBuf;
}

void tsOptFlow::onRead(eventBuffer& _buf)
{
#ifdef _DEBUG
    cout << "[tsOptFlow] Buffer to forward received" << endl;
#endif
    if(polarity!=-1)
        tsofThreadPos->setBuffer(_buf);
    if(polarity==-1 || polarity==2)
        tsofThreadNeg->setBuffer(_buf);
}

