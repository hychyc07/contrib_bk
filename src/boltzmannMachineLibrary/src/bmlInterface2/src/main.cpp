// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/all.h>
#include <iCub/BMLInterface.h>


using namespace yarp::os;

int main(int argc, char *argv[]) {
    std::string fname;

    if(argc<2){
        //there are no parameters
        // litte help on how to use
        printf("______ HELP ________ \n");
        printf(" \n");
        printf("USER COMMANDS: \n");
        printf("--name (XXXX): defines the name of the module and the rootname of every port \n");
        printf("e.g bmlInterface --name /bmlInterface \n");
        printf(" \n");
        printf(" \n");
        return 0;
    }
    else{
        //estracts the command from command line
        for (int i=1;i<argc;i++) {
            if ((strcmp(argv[i],"--name")==0)||(strcmp(argv[i],"-n")==0)) {
                fname = argv[++i];
                printf("file name:%s \n",fname.c_str());
            }
            if ((strcmp(argv[i],"--help")==0)||(strcmp(argv[i],"-h")==0)) {
                printf("______ HELP ________ \n");
                printf(" \n");
                printf("USER COMMANDS: \n");
                printf("--name (XXXX): defines the name of the module and the rootname of every port \n");
                printf("e.g bmlInterface --name /bmlInterface \n");
                printf(" \n");
                printf(" \n");
                
                return 0;
            }
            //options.fromCommand(argc,argv);
        }
    }
      // Create and run processor module
    BMLInterface *module=new BMLInterface();
    //module->processor1=new ImageProcessor();
    //module->processor2=new ImageProcessor();
    //module->processor3=new ImageProcessor();
    //module->currentProcessor=module->processor1;
    module->setName(fname.c_str());
    
    
    //initialise Yarp Network
    Network yarp;
    // This is called in all GTK applications. Arguments are parsed
    // from the command line and are returned to the application.
    gtk_init (&argc, &argv);

    
    // Get command line options
    //Property options;
    //options.fromCommand(argc,argv);
    //module.setOptions(options);
    
    

    return module->runModule(argc,argv);
    //return 0;
}

