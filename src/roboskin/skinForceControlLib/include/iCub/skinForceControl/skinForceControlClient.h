#ifndef __ICUB_SFC_CLIENT_H__
#define __ICUB_SFC_CLIENT_H__

#include <yarp/os/Port.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/BufferedPort.h>
#include <iCub/skinDynLib/common.h>
#include <iCub/skinDynLib/skinContactList.h>
#include <iCub/skinForceControl/skinForceControlLib.h>
#include <iCub/skinForceControl/controlLaws.h>
#include <map>
#include <vector>
#include <fstream>

namespace iCub{

namespace skinForceControl{

    class SfcMonitorData
    {
        yarp::sig::Vector data;
        unsigned int size;
        unsigned int extraDataSize;

    public:
        SfcMonitorData(){
            // torques+torque ref: 2*dof
            // force+force ref: 6
            // pos+pos ref: 6
            // feasibility+contact number+force norm+pwm norm: 4
            // ctrlPnt+realCtrlPnt: 6
            // pos2: 3
            // ref cont link: 1
            // joint ang+vel+acc: 3*dof
            // vel+refVel+acc+refAcc: 12
            // pos desired: 3
            // pwm: dof
            // qRef: dof
            // xOrtho: 3
            // xdEnv: 3
            // contact CoP: 3
            // extra data: 50
            // skinContact: 20+?
            extraDataSize=50;
            //size = 10+6+6+3+3+3+3+4+10+12+6+3+20+10+10+3+extraDataSize;
            size = 150 + extraDataSize;
            data.resize(size);
            data.zero(); 
        }

        yarp::sig::Vector& getData(){           return data; }
        yarp::sig::Vector getTorques()
        {
            return yarp::math::cat(
                        yarp::math::cat(data[61],data[63],data[65]),
                        yarp::math::cat(data[1],data[3],data[5],data[7],data[9])
                    );
        }
        //vector of reference command (either torque or velocity, depending on the current control law)
        yarp::sig::Vector getRefCommand()
        {
            return yarp::math::cat(
                        yarp::math::cat(data[60],data[62],data[64]), 
                        yarp::math::cat(data[0],data[2],data[4],data[6],data[8])
                    );
        }
        yarp::sig::Vector getForce(){           return yarp::math::cat(data[11],data[13],data[15]); }
        yarp::sig::Vector getRefForce(){        return yarp::math::cat(data[10],data[12],data[14]); }
        yarp::sig::Vector getX(){               return yarp::math::cat(data[17],data[19],data[21]); }
        yarp::sig::Vector getRefX(){            return yarp::math::cat(data[16],data[18],data[20]); }
        double getFeasibility(){                return data[22]; }
        unsigned int getContactNumber(){        return (unsigned int)data[23]; }
        double getForceNorm(){                  return data[24]; }
        double getPwmNorm(){                    return data[25]; }
        // 26-27 free
        yarp::sig::Vector getRealCtrlPoint(){   return data.subVector(28,30); }
        yarp::sig::Vector getXc(){              return data.subVector(31,33); }
        // 34-37 free
        yarp::sig::Vector getJointAng(){        return data.subVector(38,47); }
        yarp::sig::Vector getRefDx(){           return data.subVector(48,50); }
        yarp::sig::Vector getDx(){             return data.subVector(51,53); }
        yarp::sig::Vector getRefDdx(){          return data.subVector(54,56); }
        yarp::sig::Vector getDdx(){             return data.subVector(57,59); }
        // 66-68 free
        yarp::sig::Vector getJointVel(){        return data.subVector(69,78); }
        yarp::sig::Vector getJointAcc(){        return data.subVector(79,88); }
        yarp::sig::Vector getPwm(){             return data.subVector(89,98); }
        yarp::sig::Vector getJointAngRef(){     return data.subVector(99,108); }
        // 108-111 free
        yarp::sig::Vector getXd(){              return data.subVector(112,114); }
        // 115-149 free
        yarp::sig::Vector getExtraData(){       return data.subVector(size-extraDataSize, data.size()-1); }
        iCub::skinDynLib::skinContact getContact(){ 
            iCub::skinDynLib::skinContact c;
            yarp::sig::Vector v=data.subVector(size, data.size()-1);
            if(!c.fromVector(v))
                printf("Error converting vector to skinContact: %s\n", v.toString(2).c_str());
            return c;
        }

        void setData(const yarp::sig::Vector &d){               data=d; }
        //void resizeExtraData(unsigned int d){                   data.resize(size+d); }
        void setTorques(const yarp::sig::Vector &d)
        {
            data[61]=d[0]; data[63]=d[1]; data[65]=d[2];
            data[1]=d[3]; data[3]=d[4]; data[5]=d[5]; data[7]=d[6]; data[9]=d[7]; 
        }
        void setRefCommand(const yarp::sig::Vector &d)
        {
            data[60]=d[0]; data[62]=d[1]; data[64]=d[2];
            data[0]=d[3]; data[2]=d[4]; data[4]=d[5]; data[6]=d[6]; data[8]=d[7]; 
        }
        void setForce(const yarp::sig::Vector &d){              data[11]=d[0]; data[13]=d[1]; data[15]=d[2]; }
        void setRefForce(const yarp::sig::Vector &d){           data[10]=d[0]; data[12]=d[1]; data[14]=d[2]; }
        // Same as ctrlPoint, but in root ref frame
        void setX(const yarp::sig::Vector &d){                  data[17]=d[0]; data[19]=d[1]; data[21]=d[2]; }
        // Reference position, output of the min jerk trajectory generator
        void setRefX(const yarp::sig::Vector &d){               data[16]=d[0]; data[18]=d[1]; data[20]=d[2]; }
        // Contact point, in root ref frame
        void setXc(const yarp::sig::Vector &d){                 data.setSubvector(31,d); }
        void setFeasibility(double d){                          data[22]=d; }
        void setContactNumber(unsigned int d){                  data[23]=d; }
        void setForceNorm(double d){                            data[24]=d; }
        void setPwmNorm(double d){                              data[25]=d; }
        void setJointAng(const yarp::sig::Vector &d){           data.setSubvector(38,d); }
        void setRefDx(const yarp::sig::Vector &d){              data.setSubvector(48,d); }
        void setDx(const yarp::sig::Vector &d){                 data.setSubvector(51,d); }
        void setRefDdx(const yarp::sig::Vector &d){             data.setSubvector(54,d); }
        void setDdx(const yarp::sig::Vector &d){                data.setSubvector(57,d); }
        void setJointVel(const yarp::sig::Vector &d){           data.setSubvector(69,d); }
        void setJointAcc(const yarp::sig::Vector &d){           data.setSubvector(79,d); }
        void setPwm(const yarp::sig::Vector &d){                data.setSubvector(89,d); }
        void setJointAngRef(const yarp::sig::Vector &d){        data.setSubvector(99,d); }
        void setXd(const yarp::sig::Vector &d){                 data.setSubvector(112,d); }
        void setExtraData(const yarp::sig::Vector &d){          data.setSubvector(size-extraDataSize,d); }
        void setContact(const iCub::skinDynLib::skinContact &d){
            yarp::sig::Vector temp = d.toVector();
            //printf("Contact \n%s\nconverted into vector\n%s\n", d.toString(2).c_str(), temp.toString(2).c_str());
            data.resize(size+temp.size());
            data.setSubvector(size, temp); 
        }
    };

    const std::string SfcCommand_s[]  = {
        "stop",                 "float ctrl",           "jpos ctrl",            "pos ctrl", 
        "torque ctrl",          "force ctrl",           "paral ctrl",           "dparal ctrl",
        "cont ctrl",            "press ctrl",           "reach ctrl",           "get ctrl",
        "set qd",               "get qd",
        "set ctrlPnt",          "get ctrlPnt",          "get x",
        "set ctrlLink",         "get ctrlLink",
        "set xd",               "get xd",
        "set taud",             "set taudj",            "get taud",             "get tau",
        "set fd",               "get fd",               "get f",
        "set alphaf",           "get alphaf",           "set alphafd",          "get alphafd",
        "set alphaTaod",        "get alphaTaod",        "set traj time",        "get traj time",
        "sim on",               "sim off",
        "block joint",          "unblock joint",        "is active",
        "set pinv damp",        "get pinv damp" 
    };

	class skinForceControlClient: 
        public yarp::os::TypedReaderCallback<yarp::sig::Vector>, 
        public yarp::os::TypedReaderCallback<iCub::skinDynLib::skinContactList> 
    { 
        enum SfcCommand{
            stop_ctrl,      float_ctrl,     jpos_ctrl,      pos_ctrl,       torque_ctrl,    
            force_ctrl,     paral_ctrl,     dparal_ctrl,    cont_ctrl,      press_ctrl,     
            reach_ctrl,     get_ctrl,
            set_qd,         get_qd,
            set_ctrlPnt,    get_ctrlPnt,    get_x,
            set_ctrlLink,   get_ctrlLink,
            set_xd,         get_xd,
            set_taud,       set_taud_j,     get_taud,       get_tau,
            set_fd,         get_fd,         get_f,
            set_alphaF,     get_alphaF,     set_alphaFd,    get_alphaFd,    
            set_alphaTaod,  get_alphaTaod,  set_trajTime,   get_trajTime,
            sim_on,         sim_off,        block_joint,    unblock_joint,  is_blocked,
            set_damp,       get_damp,       SfcCommandSize};

        std::string sfcName;
        std::string name;
		yarp::os::Port  rpcPort;
        yarp::os::BufferedPort<yarp::sig::Vector> streamPort;

        yarp::os::Semaphore mutex;
        SfcMonitorData streamData;
        double lastReadTimestamp;

        virtual void onRead(yarp::sig::Vector& v);

        bool sendCommand(SfcCommand c);
        bool sendCommand(SfcCommand c, int d);
        bool sendCommand(SfcCommand c, double d);
        bool sendCommand(SfcCommand c, const yarp::sig::Vector &v);

        yarp::sig::Vector sendCommandThatReturnsVector(SfcCommand c);
        double sendCommandThatReturnsDouble(SfcCommand c);
        int sendCommandThatReturnsInt(SfcCommand c);
			
	public:
		skinForceControlClient(const char* _name, const char* _sfcName=NULL);
		~skinForceControlClient();
        bool init();
        
        bool getStreamData(SfcMonitorData &data);

        // *** SET COMMANDS (return true if succeeded, false otherwise)
        bool stop(){                return sendCommand(stop_ctrl); }
        bool startControl(ControlLaw cl);
        bool setQd(const yarp::sig::Vector &qd){    return sendCommand(set_qd, qd); }
        bool setTaud(const yarp::sig::Vector &taud){return sendCommand(set_taud, taud); }
        bool setCtrlPnt(unsigned int linkNum, const yarp::sig::Vector &p);
        /**
        * Set the destination point in root reference frame.
        */
        bool setXd(const yarp::sig::Vector &xd){ return sendCommand(set_xd, xd); }
        /** 
        * Set the destination point in link reference frame. Note that xd is not 
        * attached to the specified robot link but it is first projected in root 
        * reference frame and then set. 
        */
		bool setXd(unsigned int linkNum, const yarp::sig::Vector &xd);
        bool setFd(const yarp::sig::Vector &fd){    return sendCommand(set_fd, fd); }
        bool setTrajectoryTime(double t){           return sendCommand(set_trajTime, t); }
        bool setSimulationMode(bool on){            return on?sendCommand(sim_on):sendCommand(sim_off); }

        // *** GET COMMANDS
        unsigned int getCtrlPntLink(){  return sendCommandThatReturnsInt(get_ctrlLink); }
        yarp::sig::Vector getCtrlPnt(){ return sendCommandThatReturnsVector(get_ctrlPnt); }
        yarp::sig::Vector getFd(){      return sendCommandThatReturnsVector(get_fd); }
        double getTrajectoryTime(){     return sendCommandThatReturnsDouble(get_trajTime); }
	};

}

}

#endif //__ICUB_SFC_CLIENT_H__
