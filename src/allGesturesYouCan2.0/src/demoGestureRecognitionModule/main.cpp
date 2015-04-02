#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <deque>
#include <stdio.h>
#include <stdlib.h>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/math/Rand.h>

#define VOCAB_CMD_UPDATE_INITIAL VOCAB3('p','o','s')
#define VOCAB_CMD_REC            VOCAB3('r','e','c')
#define VOCAB_CMD_STOP           VOCAB4('s','t','o','p')
#define VOCAB_CMD_TRAIN          VOCAB4('t','r','a','i')
#define VOCAB_CMD_SAVE           VOCAB4('s','a','v','e')

using namespace std;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp::math;

class DemoModule: public RFModule
{
protected:

    Port rpc;
    Port outExecModule;
    Port outspeak;
    Port outGestRecModule;
    BufferedPort<Bottle> inSequence;
    string sequence;
    string currentSequence;
    bool myturn;
    bool yourturn;
    bool gameEnding;
    int numActions;
    int count;
    int temp;

    RandScalar randGen;

    bool close()
    {
        rpc.close();
        return true;
    }

    bool interruptModule()
    {
        rpc.interrupt();
        temp=-1;
        return true;
    }

    double getPeriod()
    {
        return 0.01;
    }

    bool respond(const Bottle& cmd, Bottle& reply) 
    {
        if (cmd.get(0).asString()=="save")
        {
            Bottle bot1,bot2;
            bot1.addVocab(VOCAB_CMD_STOP);
            outGestRecModule.write(bot1,bot2);
            string a="Ok, show me this gesture";
            Bottle w;
            w.addString(a.c_str());
            outspeak.write(w);
            Bottle bot5,bot6;
            bot5.addString("execute");
            bot5.addInt(cmd.get(1).asInt());
            outExecModule.write(bot5,bot6);
            Bottle bot7,bot8;
            bot7.addVocab(VOCAB_CMD_UPDATE_INITIAL);
            outGestRecModule.write(bot7,bot8);
            Bottle bot3,bot4;
            bot3.addVocab(VOCAB_CMD_SAVE);
            stringstream ss;
            ss << cmd.get(1).asInt();
            string str = "action"+ss.str();
            bot3.addString(str.c_str());
            outGestRecModule.write(bot3,bot4);
            reply.addString("Starting saving features");
            return true;
        }
        if (cmd.get(0).asString()=="trained")
        {
            string a="Training";
            Bottle w;
            w.addString(a.c_str());
            outspeak.write(w);
            Bottle bot1,bot2;
            bot1.addVocab(VOCAB_CMD_TRAIN);
            outGestRecModule.write(bot1,bot2);
            string b="Thanks, now I know the gesture";
            Bottle w1;
            w1.addString(b.c_str());
            outspeak.write(w1);
            reply.addString("Training done");
            return true;
        }
        if (cmd.get(0).asString()=="start")
        {
            count=1;
            Bottle bot1,bot2;
            bot1.addVocab(VOCAB_CMD_UPDATE_INITIAL);
            outGestRecModule.write(bot1,bot2);
            Bottle bot3,bot4;
            bot3.addVocab(VOCAB_CMD_REC);
            outGestRecModule.write(bot3,bot4);
            double number=randGen.get(0,1);
            string turn;
            if (number<=0.5)
                turn="your turn";
            else
                turn="my turn";
            string a="Ok, let's start the game, it is "+turn;
            reply.addString("Ok, let's start the game ");
            reply.addString(yourturn?"your turn":"my turn");
            Bottle w;
            w.addString(a.c_str());
            outspeak.write(w);
            if(number>0.5)
                Time::delay(3.5);
            int pending=inSequence.getPendingReads();
            for (int i=0; i<pending; i++)
                inSequence.read(false);
            gameEnding=false;
            if (number>0.5)
                myturn=true;
            else
                yourturn=true;
            return true;
        }
        else if (cmd.get(0).asString()=="win"||cmd.get(0).asString()=="lose"||cmd.get(0).asString()=="over")
        {
            reply.addString("Quitting game");
            Bottle bot1,bot2;
            bot1.addString("stop");
            outExecModule.write(bot1,bot2);
            Bottle bot3,bot4;
            bot3.addVocab(VOCAB_CMD_STOP);
            outGestRecModule.write(bot3,bot4);
            sequence="";
            currentSequence="";
            gameEnding=true;
            myturn=false;
            yourturn=false;
            fprintf(stdout, "Quitting game\n");

            if (cmd.get(0).asString()=="win")
            {
                Bottle w;
                w.addString("Yay! I'm happy!");
                outspeak.write(w);
                fprintf(stdout, "yay! I'm happy!\n");
            }
            else if (cmd.get(0).asString()=="lose")
            {
                Bottle w;
                w.addString("I'm really sad!");
                outspeak.write(w);
                fprintf(stdout, "Oh No!\n");
            }
            else
            {
                Bottle w;
                w.addString("Game over!");
                outspeak.write(w);
                fprintf(stdout, "Quitting!\n");
            }

            return true;
        }
        else if (cmd.get(0).asString()=="done"&&!gameEnding)
        {
            Bottle bot1,bot2;
            bot1.addVocab(VOCAB_CMD_UPDATE_INITIAL);
            outGestRecModule.write(bot1,bot2);
            Bottle bot3,bot4;
            bot3.addVocab(VOCAB_CMD_REC);
            outGestRecModule.write(bot3,bot4);
            yourturn=true;
            int pending=inSequence.getPendingReads();
            for (int i=0; i<pending; i++)
                inSequence.read(false);
            reply.addString("ack");
            Bottle w;
            w.addString("It's your turn");
            outspeak.write(w);
            return true;
        }
        else if(cmd.get(0).asString()=="turn")
        {
             Bottle bot3,bot4;
             bot3.addVocab(VOCAB_CMD_STOP);
             outGestRecModule.write(bot3,bot4);
             if(!gameEnding)
             {
                myturn=true;
                yourturn=false;
                reply.addString("It is my turn now");
             }
             return true;
        }

        else
        {
            reply.addString("command not recognized");
            return true;
        }
    }

    bool updateModule()
    {
        if (!gameEnding && myturn)
        {
            //int number=rand() % numActions+1;
            int number=(int) randGen.get(1,numActions+0.99);
            ostringstream action;
            action << number;
            sequence=sequence+action.str();
            currentSequence=currentSequence+action.str();

            Bottle cmd,reply;
            cmd.addString("recognize");
            cmd.addInt(atoi(sequence.c_str()));
            outExecModule.write(cmd,reply);
            myturn=false;
            sequence="";
            count++;
        }
        if (!gameEnding && yourturn)
        {
            bool right=true;
            double t=Time::now();
            temp=0;
            while(!gameEnding /*&& temp<count && (Time::now()-t<4 || temp==0)*/ &&(yourturn && !myturn))
            {
                Bottle* bot=inSequence.read(false);
                if (bot!=NULL)
                {
                    if (bot->get(0).asInt()>numActions || temp>currentSequence.size())
                    {
                        right=false;
                        break;
                    }
                    //int sub=atoi(currentSequence.substr(temp,1).c_str());
                    /*if (bot->get(0).asInt()!=sub && count!=1 && temp!=count-1)
                    {
                        fprintf(stdout, "Entrato, %d %d %d\n", count, sub, bot->get(0).asInt());
                        right=false;
                        break;
                    }*/
                    ostringstream buffer;
                    buffer << bot->get(0).asInt();
                    sequence=sequence+buffer.str();
                    t=Time::now();
                    temp++;
                }
            }
            if (temp==-1)
                return true;

            fprintf(stdout, "sequence: %s\n", sequence.c_str());
            if (!right)
            {
                if(!gameEnding)
                {
                    Bottle w;
                    w.addString("I think you're wrong");
                    fprintf(stdout, "I think you're wrong\n");
                    outspeak.write(w);
                    myturn=false;
                    yourturn=false;
                    count=0;
                    currentSequence="";
                    sequence="";
                    Bottle bot1,bot2;
                    bot1.addString("stop");
                    outExecModule.write(bot1,bot2);
                    Bottle bot3,bot4;
                    bot3.addVocab(VOCAB_CMD_STOP);
                    outGestRecModule.write(bot3,bot4);
                    return true;
                }
                return true;
            }
            else if (sequence!="" && sequence.substr(0,sequence.size()-1)==currentSequence && !gameEnding)
                currentSequence=currentSequence+sequence.substr(sequence.size()-1,1);
            else
            {
                if(!gameEnding)
                {
                    Bottle w;
                    w.addString("I think you're wrong");
                    fprintf(stdout, "I think you're wrong\n");
                    outspeak.write(w);
                    myturn=false;
                    yourturn=false;
                    count=0;
                    currentSequence="";
                    sequence="";
                    Bottle bot1,bot2;
                    bot1.addString("stop");
                    outExecModule.write(bot1,bot2);
                    Bottle bot3,bot4;
                    bot3.addVocab(VOCAB_CMD_STOP);
                    outGestRecModule.write(bot3,bot4);
                    return true;
                }
            }

            if(!gameEnding)
            {
                count++;
                myturn=true;
                yourturn=false;
            }
        }

        return true;
    }

public:
    bool configure(ResourceFinder &rf)
    {
        string name="demoActionRecognition";
        string rpcName="/"+name+"/rpc";
        string outExecName="/"+name+"/exec:o";
        string inName="/"+name+"/scores:i";
        string outGestRecModName="/"+name+"/gestRec:o";
        rpc.open(rpcName.c_str());
        attach(rpc);
        outExecModule.open(outExecName.c_str());
        inSequence.open(inName.c_str());
        string outspeakname="/"+name+"/outspeak";
        outspeak.open(outspeakname.c_str());
        outGestRecModule.open(outGestRecModName.c_str());

        string filename=rf.findFile("actions").c_str();
        Property config; config.fromConfigFile(filename.c_str());

        Bottle& bgeneral=config.findGroup("general");
        numActions=bgeneral.find("numActions").asInt();

        sequence="";
        currentSequence="";
        myturn=false;
        yourturn=false;
        gameEnding=true;
        count=0;
        temp=0;

        return true;
    }
};

int main(int argc, char *argv[])
{
    Network yarp;

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("allGesturesYouCan2.0/conf");

    rf.setDefault("from","config.ini");
    rf.setDefault("actions","actions.ini");
    rf.configure("ICUB_ROOT",argc,argv);

    DemoModule mod;
    mod.runModule(rf);

    return 0;
}

