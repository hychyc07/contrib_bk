#include <string>
#include <yarp/os/Property.h>
#include <yarp/os/Port.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Vocab.h>

class Communicator
{

private:
    yarp::os::Port mil;
    yarp::os::Port opc;
    bool isOpen;

public:

    Communicator();
    ~Communicator();

    bool retrieveTableHeight(double &tableHeight);
    bool open(const yarp::os::Property &options);
    bool disableAttention();
    bool enableAttention();
    bool retrieveBoundingBox(const std::string &object, yarp::sig::Vector &leftTopVertex, yarp::sig::Vector &rightDownVertex);
    void close();
    bool checkPorts();

};


