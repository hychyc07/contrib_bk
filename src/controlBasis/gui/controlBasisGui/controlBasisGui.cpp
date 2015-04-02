// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#include "controlBasisGui.h"

#include <string.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <yarp/os/Bottle.h>
//#include <yarp/os/impl/NameClient.h>
//#include <yarp/os/impl/NameConfig.h>

#include <controlBasis/PotentialFunctionRegister.h>
#include <controlBasis/JacobianRegister.h>

using namespace CB;
using namespace std;
using namespace yarp::os;
using namespace yarp::os::impl;

CBAPIWindow::CBAPIWindow() :
    cbapiTable(2,2,true),
    controlTabTable(1,2,true),
    sequenceTabTable(1,2,true),
    controlResourcesTable(6,9,true),
    controlDefinitionTable(8,12,true),
    sequenceControllersTable(9,12,true),
    controlOutputTable(9,12,true),
    optionsTable(3,1,true),
    sensorList("sensors"),
    referenceList("reference"),
    potentialFunctionList("potential functions"),
    effectorList("effectors"),
    controlDefinitionText("blue","white"),
    controlOutputText("green","black"),
    sequenceControllersText("black","gray"),
    sequenceOutputText("yellow","black"),
    addControllerButton(Gtk::Stock::ADD),
    clearControllerButton(Gtk::Stock::CLEAR),
    runControllerButton(Gtk::Stock::MEDIA_PLAY),
    stopControllerButton(Gtk::Stock::MEDIA_STOP),
    refreshButton(Gtk::Stock::REFRESH),
    addControllerToSequenceButton(Gtk::Stock::ADD),
    addSecondaryControllerToSequenceButton("Nullspace"),
    clearSequenceButton(Gtk::Stock::CLEAR),
    runSequenceButton(Gtk::Stock::MEDIA_PLAY),
    stopSequenceButton(Gtk::Stock::MEDIA_STOP),
    fwdSequenceButton(Gtk::Stock::MEDIA_FORWARD),
    bkSequenceButton(Gtk::Stock::MEDIA_REWIND),
    saveControllerOutputButton("Save Output"),
    setParametersButton("Set"),
    allowVirtualEffectorsBox("allow virtual effectors"),
    useJacobianTransposeBox("use Jacobian Transpose"),
    usePDControlBox("use PD-Control"),
    usePIDControlBox("use PID-Control"),
    controllerGainPLabel("Kp:"),
    controllerGainILabel("Ki:"),
    controllerGainDLabel("Kd:"),
    sequenceGainPLabel("Kp:"),
    sequenceGainILabel("Ki:"),
    sequenceGainDLabel("Kd:"),
    nullspaceFactorLabel("NF:")
{

    registerPotentialFunctions();
    registerJacobians();

    controlLawRunning = false;
    sequenceRunning = false;
    showVirtualEffectors = false;
    useJacobianTranspose = true;
    usePDControl = true;
    usePIDControl = false;
    useJacobianTransposeBox.set_active(useJacobianTranspose);
    usePDControlBox.set_active(usePDControl);
    usePIDControlBox.set_active(usePIDControl);
    cbapi.useTranspose(useJacobianTranspose);
    cbapi.usePDControl(usePDControl);
    cbapi.usePIDControl(usePIDControl);

    setPIDTextFields();

    set_title("Control Basis API GUI");
    set_size_request(1200,800);
    set_border_width(6);
    //set_icon_from_file("cb.png");

    addControllerButton.set_border_width(4);
    clearControllerButton.set_border_width(4);
    runControllerButton.set_border_width(4);
    stopControllerButton.set_border_width(4);
    refreshButton.set_border_width(2);

    addControllerToSequenceButton.set_border_width(4);
    addSecondaryControllerToSequenceButton.set_border_width(4);
    clearSequenceButton.set_border_width(4);
    runSequenceButton.set_border_width(4);
    stopSequenceButton.set_border_width(4);
    fwdSequenceButton.set_border_width(4);
    bkSequenceButton.set_border_width(4);
    setParametersButton.set_border_width(4);

    addControllerButton.signal_clicked().connect( sigc::mem_fun(*this,
                                                                &CBAPIWindow::on_add_button_clicked) );
    clearControllerButton.signal_clicked().connect( sigc::mem_fun(*this,
                                                                  &CBAPIWindow::on_clear_button_clicked) );
    runControllerButton.signal_clicked().connect( sigc::mem_fun(*this,
                                                                &CBAPIWindow::on_run_button_clicked) );
    stopControllerButton.signal_clicked().connect( sigc::mem_fun(*this,
                                                                 &CBAPIWindow::on_stop_button_clicked) );
    refreshButton.signal_clicked().connect( sigc::mem_fun(*this,
                                                          &CBAPIWindow::on_refresh_button_clicked) ); 

    allowVirtualEffectorsBox.signal_clicked().connect( sigc::mem_fun(*this,
                                                                  &CBAPIWindow::on_virtual_effector_toggle) );
    useJacobianTransposeBox.signal_clicked().connect( sigc::mem_fun(*this,
                                                                 &CBAPIWindow::on_use_jacobian_transpose) );
    usePDControlBox.signal_clicked().connect( sigc::mem_fun(*this,
                                                                 &CBAPIWindow::on_use_pd_control) );
    usePIDControlBox.signal_clicked().connect( sigc::mem_fun(*this,
                                                                 &CBAPIWindow::on_use_pid_control) );

    addControllerToSequenceButton.signal_clicked().connect( sigc::mem_fun(*this,
                                                                          &CBAPIWindow::on_add_to_sequence_button_clicked) );
    addSecondaryControllerToSequenceButton.signal_clicked().connect( sigc::mem_fun(*this,
                                                                                   &CBAPIWindow::on_add_secondary_to_sequence_button_clicked) );
    clearSequenceButton.signal_clicked().connect( sigc::mem_fun(*this,
                                                                &CBAPIWindow::on_clear_sequence_button_clicked) );
    runSequenceButton.signal_clicked().connect( sigc::mem_fun(*this,
                                                                &CBAPIWindow::on_run_sequence_button_clicked) );
    stopSequenceButton.signal_clicked().connect( sigc::mem_fun(*this,
                                                                 &CBAPIWindow::on_stop_sequence_button_clicked) );
    fwdSequenceButton.signal_clicked().connect( sigc::mem_fun(*this,
                                                                 &CBAPIWindow::on_fwd_sequence_button_clicked) );
    bkSequenceButton.signal_clicked().connect( sigc::mem_fun(*this,
                                                                 &CBAPIWindow::on_bk_sequence_button_clicked) );

    setParametersButton.signal_clicked().connect( sigc::mem_fun(*this,
                                                                       &CBAPIWindow::on_set_parameters_button_clicked) );
    saveControllerOutputButton.signal_clicked().connect( sigc::mem_fun(*this,
                                                                       &CBAPIWindow::on_save_controller_output_button_clicked) );

    add(cbapiVBox);
    cbapiVBox.pack_start(cbapiTable);
    cbapiTable.set_border_width(5);

    cbapiTable.attach(controlResourcesFrame,0,2,0,1);
    cbapiTable.attach(cbapiNotebook,0,2,1,2);

    cbapiNotebook.set_border_width(5);
    cbapiNotebook.set_tab_pos(Gtk::POS_BOTTOM);
    
    controlTabTable.set_border_width(5);
    sequenceTabTable.set_border_width(5);

    controlDefinitionTable.set_border_width(5);
    sequenceControllersTable.set_border_width(5);

    controlResourcesFrame.set_label("Controller Resources");
    controlResourcesFrame.set_shadow_type(Gtk::SHADOW_ETCHED_IN);

    controlOutputTable.set_border_width(5);

    sensorList.set_border_width(3);
    referenceList.set_border_width(3);
    potentialFunctionList.set_border_width(3);
    effectorList.set_border_width(3);

    controlResourcesTable.attach(sensorList,0,3,0,3);
    controlResourcesTable.attach(referenceList,0,3,3,6);
    controlResourcesTable.attach(potentialFunctionList,3,6,0,6);
    controlResourcesTable.attach(effectorList,6,9,0,5);
    controlResourcesTable.attach(refreshButton,6,7,5,6);

    optionsTable.attach(allowVirtualEffectorsBox,0,1,0,1);
    optionsTable.attach(useJacobianTransposeBox,0,1,1,2);
    optionsTable.attach(usePDControlBox,0,1,2,3);
    optionsTable.attach(usePIDControlBox,0,1,3,4);
    controlResourcesTable.attach(optionsTable,7,9,5,6);

    controllerGainPEntry.set_max_length(5);
    controllerGainIEntry.set_max_length(5);
    controllerGainDEntry.set_max_length(5);
    controllerGainPEntry.set_text("5");
    controllerGainIEntry.set_text("5");
    controllerGainDEntry.set_text("2.5");
    controllerGainIEntry.set_editable(false);

    nullspaceFactorEntry.set_max_length(5);
    nullspaceFactorEntry.set_text("0.5");

    sequenceGainPEntry.set_max_length(5);
    sequenceGainIEntry.set_max_length(5);
    sequenceGainDEntry.set_max_length(5);
    sequenceGainPEntry.set_text("1");
    sequenceGainIEntry.set_text("1");
    sequenceGainDEntry.set_text("1");
    sequenceGainIEntry.set_editable(false);
    
    controlResourcesFrame.add(controlResourcesTable);
   
    // for the control law tab
    controlDefinitionFrame.set_label("Controller Definition");
    controlDefinitionFrame.set_shadow_type(Gtk::SHADOW_ETCHED_IN);
    controlDefinitionText.set_border_width(5);
    
    controlDefinitionTable.attach(controlDefinitionText,0,8,0,8);
    controlDefinitionTable.attach(addControllerButton,8,10,0,1); 
    controlDefinitionTable.attach(runControllerButton,8,10,1,2);
    controlDefinitionTable.attach(stopControllerButton,8,10,2,3);
    controlDefinitionTable.attach(clearControllerButton,8,10,3,4);

    controlDefinitionTable.attach(controllerGainPLabel,10,11,0,1); 
    controlDefinitionTable.attach(controllerGainPEntry,11,12,0,1); 

    controlDefinitionTable.attach(controllerGainILabel,10,11,1,2); 
    controlDefinitionTable.attach(controllerGainIEntry,11,12,1,2); 

    controlDefinitionTable.attach(controllerGainDLabel,10,11,2,3); 
    controlDefinitionTable.attach(controllerGainDEntry,11,12,2,3); 

    controlDefinitionTable.attach(nullspaceFactorLabel,10,11,3,4); 
    controlDefinitionTable.attach(nullspaceFactorEntry,11,12,3,4); 

    controlDefinitionTable.attach(setParametersButton,8,12,7,8);

    controlDefinitionFrame.add(controlDefinitionTable);
    controlTabTable.attach(controlDefinitionFrame,0,1,0,2);

    controlOutputFrame.set_label("Controller Output");
    controlOutputFrame.set_shadow_type(Gtk::SHADOW_ETCHED_IN);
    controlOutputText.set_border_width(5);
    controlOutputTable.attach(controlOutputText,0,12,0,8);
    controlOutputTable.attach(saveControllerOutputButton,10,12,8,9);
    controlOutputFrame.add(controlOutputTable);
    controlTabTable.attach(controlOutputFrame,1,2,0,2);
    //    controlOutputFrame.add(controlOutputText);
    //controlTabTable.attach(controlOutputFrame,1,2,0,2);

    // for the sequence tab
    sequenceControllersFrame.set_label("Control Sequence");
    sequenceControllersFrame.set_shadow_type(Gtk::SHADOW_ETCHED_IN);
    sequenceControllersText.set_border_width(5);

    sequenceControllersTable.attach(sequenceControllersText,0,8,0,9);
    sequenceControllersTable.attach(addControllerToSequenceButton,8,10,0,1); 
    sequenceControllersTable.attach(addSecondaryControllerToSequenceButton,8,10,1,2); 
    sequenceControllersTable.attach(runSequenceButton,8,10,3,4);
    sequenceControllersTable.attach(fwdSequenceButton,8,10,4,5);
    sequenceControllersTable.attach(bkSequenceButton,8,10,5,6);
    sequenceControllersTable.attach(stopSequenceButton,8,10,6,7);
    sequenceControllersTable.attach(clearSequenceButton,8,10,8,9);

    sequenceControllersTable.attach(sequenceGainPLabel,10,11,0,1); 
    sequenceControllersTable.attach(sequenceGainPEntry,11,12,0,1); 
    sequenceControllersTable.attach(sequenceGainILabel,10,11,1,2); 
    sequenceControllersTable.attach(sequenceGainIEntry,11,12,1,2); 
    sequenceControllersTable.attach(sequenceGainDLabel,10,11,2,3); 
    sequenceControllersTable.attach(sequenceGainDEntry,11,12,2,3); 

    sequenceControllersFrame.add(sequenceControllersTable);
    sequenceTabTable.attach(sequenceControllersFrame,0,1,0,2);

    sequenceOutputFrame.set_label("Sequence Output");
    sequenceOutputFrame.set_shadow_type(Gtk::SHADOW_ETCHED_IN);
    sequenceOutputText.set_border_width(5);
    sequenceOutputFrame.add(sequenceOutputText);
    sequenceTabTable.attach(sequenceOutputFrame,1,2,0,2);

    cbapiNotebook.append_page(controlTabTable, "Control Laws");
    cbapiNotebook.append_page(sequenceTabTable, "Sequences");
    cbapiNotebook.signal_switch_page().connect(sigc::mem_fun(*this, &CBAPIWindow::on_notebook_switch_page) );

    show_all_children();
    
    cout << "test 0\n";

    loadPotentialFunctions();
    loadJacobians();
    refreshResourceList();

    sensorTreeSelection = sensorList.getTreeSelection();
    referenceTreeSelection = referenceList.getTreeSelection();
    pfTreeSelection = potentialFunctionList.getTreeSelection();
    effectorTreeSelection = effectorList.getTreeSelection();
    
    sensorTreeSelection->signal_changed().connect(sigc::mem_fun(*this, &CBAPIWindow::on_sensor_selection));
    referenceTreeSelection->signal_changed().connect(sigc::mem_fun(*this, &CBAPIWindow::on_reference_selection));
    pfTreeSelection->signal_changed().connect(sigc::mem_fun(*this, &CBAPIWindow::on_potential_function_selection));
    effectorTreeSelection->signal_changed().connect(sigc::mem_fun(*this, &CBAPIWindow::on_effector_selection));

    dataThread = new ControlDataThread();
    dataThread->control_thread_update_finished().connect(sigc::bind<1>(sigc::mem_fun(*this, &CBAPIWindow::on_control_thread_update), dataThread));

}

CBAPIWindow::~CBAPIWindow() { 
    for(int i=0; i<pfInfo.size(); i++) 
        delete pfInfo[i];
    pfInfo.clear();
}


void CBAPIWindow::on_control_thread_update(ControlDataThread *dThread) {

    string str = dataThread->getOutputString();

    if(sequenceRunning) {
        sequenceOutputText.append_text(str);
        
        // check to see if the sequence needs to transition
        if ( (cbapi.getState(0, true) == CB::CONVERGED) || 
             (cbapi.getState(0,true) == CB::UNDEFINED) ) {           
            cout << "controller[" << cbapi.getSequenceControllerID() << "] state: " << (int)(cbapi.getState(0,true)) << endl;            
             if(cbapi.getSequenceControllerID() == (cbapi.getNumControllers(true)-1)) {
                 dataThread->stopUpdateThread();
                 cbapi.stopSequence();
                 sequenceOutputText.append_text("control sequence finished\n","white");
                 sequenceRunning = false;
             } else {
                 cbapi.goToNextControllerInSequence();
             }
         } 

     } else {
         controlOutputText.append_text(str);
     }
 }

 void CBAPIWindow::on_notebook_switch_page(GtkNotebookPage* /* page */, guint page_num) {
     cout << "CBAPI Switching to tab " << page_num << endl;
 }

 void CBAPIWindow::on_sensor_selection() {
     cout << "got sensor selection" << endl;
     string sen = sensorList.getSelected();
     if(sen=="") return;
 }

 void CBAPIWindow::on_reference_selection() {
     cout << "got reference selection" << endl;
     string ref = referenceList.getSelected();
     if(ref=="") return;
 }

 void CBAPIWindow::on_potential_function_selection() {
     cout << "got pf selection" << endl;
     string pf = potentialFunctionList.getSelected();
     if(pf=="") return;

     refreshResourceList();

     int id;
     for(id=0; id<pfInfo.size(); id++) {
         if(pfInfo[id]->name == pf) break;
     }

     referenceList.clear();
     sensorList.clear();

     printf("sensor 1 info size: %d\n", sensorInfo.size());
     for(int i=0; i<sensorInfo.size(); i) {
         printf("testing sensor %s %s\n", sensorInfo[i].space.c_str(), sensorInfo[i].name.c_str());
         if(sensorInfo[i].space != pfInfo[id]->space) {
             sensorInfo.erase(sensorInfo.begin()+i);
         } else {
             i++;
         }
     }
     printf("sensor 2 info size: %d\n", sensorInfo.size());

     string str;
     for(int i=0; i<sensorInfo.size(); i++) {
         str = sensorInfo[i].space + sensorInfo[i].name;
         printf("adding sensor: %s\n", str.c_str());
         sensorList.addResource(str);
         if(pfInfo[id]->hasReference) {
             referenceList.addResource(str);
         }
     }

 }

 void CBAPIWindow::on_effector_selection() {
     cout << "got effector selection" << endl;
     string eff = effectorList.getSelected();
     if(eff=="") return;
 }

 void CBAPIWindow::on_virtual_effector_toggle() {
     cout << "got virtual effector toggle" << endl;
     if(allowVirtualEffectorsBox.get_active()) {
         showVirtualEffectors = true;
     } else {
         showVirtualEffectors = false;
     }
     refreshResourceList();
 }

 void CBAPIWindow::on_use_jacobian_transpose() {
     cout << "got use transpose toggle" << endl;
     if(useJacobianTransposeBox.get_active()) {
         useJacobianTranspose = true;
     } else {
         useJacobianTranspose = false;
     }
     cbapi.useTranspose(useJacobianTranspose);
 }

 void CBAPIWindow::on_use_pd_control() {
     cout << "got use pd control toggle" << endl;
     if(usePDControlBox.get_active()) {
         usePDControl = true;
         usePIDControl = false;
     } else {
         usePDControl = false;         
     }
     cbapi.usePDControl(usePDControl);
     cbapi.usePIDControl(usePIDControl);

     setPIDTextFields();

 }

 void CBAPIWindow::on_use_pid_control() {
     cout << "got use pid control toggle" << endl;
     if(usePIDControlBox.get_active()) {
         usePIDControl = true;
         usePDControl = false;
     } else {
         usePIDControl = false;
     }
     cbapi.usePDControl(usePDControl);
     cbapi.usePIDControl(usePIDControl);

     setPIDTextFields();
}


void CBAPIWindow::on_save_controller_output_button_clicked() {
     cout << "save controller output" << endl;

     Gtk::FileChooserDialog dialog("Save Controller Output",
                                   Gtk::FILE_CHOOSER_ACTION_SAVE);
     dialog.set_transient_for(*this);

     dialog.add_button(Gtk::Stock::CANCEL, Gtk::RESPONSE_CANCEL);
     dialog.add_button(Gtk::Stock::SAVE, Gtk::RESPONSE_OK);

     int result = dialog.run();

     string filename;

     switch(result) {
     case(Gtk::RESPONSE_OK) :
         filename = dialog.get_filename();
         cout << "save output to: " << filename.c_str() << endl;
         cbapi.saveControllerOutput(filename);
         break;
     case(Gtk::RESPONSE_CANCEL) :
         break;
     default:
         cout << "unknown button clicked..." << endl;
         break;
     }

}

void CBAPIWindow::on_set_parameters_button_clicked() {
    
     float gain_p = 1;
     float gain_i = 0;
     float gain_d = 0;
     string gainStr;

     gainStr = controllerGainPEntry.get_text();
     cout << "got input KP: " << gainStr << endl;
     sscanf(gainStr.c_str(),"%f",&gain_p);
     cout << "Setting controller P-gain: " << gain_p << endl;

     if(usePDControl && usePIDControl) {
         usePDControl = false;
     }

     if(usePIDControl) {
         gainStr = controllerGainIEntry.get_text();
         cout << "got input KI: " << gainStr << endl;
         sscanf(gainStr.c_str(),"%f",&gain_i);
         cout << "Setting controller I-gain: " << gain_i << endl;
     }

     if(usePDControl || usePIDControl) {
         gainStr = controllerGainDEntry.get_text();
         cout << "got input KD: " << gainStr << endl;
         sscanf(gainStr.c_str(),"%f",&gain_d);
         cout << "Setting controller D-gain: " << gain_d << endl;
     }

     float nf = 0;
     string nfStr = nullspaceFactorEntry.get_text();
     cout << "got input NF: " << nfStr << endl;
     sscanf(nfStr.c_str(),"%f",&nf);
     cout << "Setting Nullspace Factor: " << nf << endl;

     cbapi.setNullspaceFactor((double)nf);
     cbapi.setGains((double)gain_p, (double)gain_i, (double)gain_d);
     //cbapi.usePDControl(usePDControl);
     //cbapi.usePIDControl(usePIDControl);

}

void CBAPIWindow::setPIDTextFields() {

     usePDControlBox.set_active(usePDControl);
     usePIDControlBox.set_active(usePIDControl);
 
     if(usePDControl) {
         controllerGainIEntry.set_editable(false);
         controllerGainDEntry.set_editable(true);
         sequenceGainIEntry.set_editable(false);
         sequenceGainDEntry.set_editable(true);
         controllerGainIEntry.set_visibility(false);
         controllerGainDEntry.set_visibility(true);
         sequenceGainIEntry.set_visibility(false);
         sequenceGainDEntry.set_visibility(true);
     } else if(usePIDControl) {
         controllerGainIEntry.set_editable(true);
         controllerGainDEntry.set_editable(true);
         sequenceGainIEntry.set_editable(true);
         sequenceGainDEntry.set_editable(true);
         controllerGainIEntry.set_visibility(true);
         controllerGainDEntry.set_visibility(true);
         sequenceGainIEntry.set_visibility(true);
         sequenceGainDEntry.set_visibility(true);
     } else {
         controllerGainIEntry.set_editable(false);
         controllerGainDEntry.set_editable(false);
         sequenceGainIEntry.set_editable(false);
         sequenceGainDEntry.set_editable(false);
         controllerGainIEntry.set_visibility(false);
         controllerGainDEntry.set_visibility(false);
         sequenceGainIEntry.set_visibility(false);
         sequenceGainDEntry.set_visibility(false);
     }


}


void CBAPIWindow::refreshResourceList() {

     cout << "RefreshResourceList()" << endl;
     string serviceName;
     string simpleServiceName;
     int start,stop;  
     string resourceName, resourceSpace;
     ResourceInfo resInfo;

     //     NameClient& nic = NameClient::getNameClient();
     //NameConfig nc;
     //String name = nc.getNamespace();
     ConstString name = Network::getNameServerName();
     Bottle msg, reply;
     msg.addString("bot");
     msg.addString("list");

     // clearing out current list
     sensorList.clear();
     referenceList.clear();
     effectorList.clear();

     sensorInfo.clear();
     effectorInfo.clear();

     cout << "Requesting list of ports from name server" << endl;
     Network::write(name,
                    msg,
                    reply);
     int ct = reply.size()-1;
     cout << "Got " << ct << " port " << ((ct!=1)?"s":"") << endl ;
     for (int i=1; i<reply.size(); i++) {
         Bottle *entry = reply.get(i).asList();
         if (entry!=NULL) {
             ConstString port = entry->check("name",Value("")).asString();
             if (port!="" && port!="fallback" && port!=name.c_str()) {
                 //Contact c = Contact::byConfig(*entry);
                 //Address addr = Address::fromContact(c);
                 //if (addr.isValid()) {
                 serviceName = port.c_str();

                 if( (serviceName.compare(0,3,"/cb")==0) && (serviceName.compare(serviceName.size()-6,6,"data:o")==0) ) {
                     
                     if(serviceName.at(serviceName.size()-1)!='i') {
                         simpleServiceName = serviceName.substr(4,serviceName.size()-4-7);
                         
                         cout << "found control basis service: " << simpleServiceName.c_str() << endl;
                         
                         start = 0;
                         stop = simpleServiceName.find_first_of("/", start);
                         resourceSpace = simpleServiceName.substr(start,stop-start);
                         resourceName = simpleServiceName.substr(stop-start,simpleServiceName.size());	      
                         resInfo.name = resourceName;
                         resInfo.space = resourceSpace;

                         if(!isTemporaryPort(resInfo.name)) {
                             sensorInfo.push_back(resInfo);

                             sensorList.addResource(simpleServiceName);
                             referenceList.addResource(simpleServiceName);                        

                         } else {
                             cout << "rejecting sensor: " << simpleServiceName.c_str() << endl;
                         }
                         
                     }
                     //	      cout << "name=%s, space=%s\n", resourceName.c_str(), resourceSpace.c_str());
                     
                 } else if( (serviceName.compare(0,17,"/cb/configuration")==0) && (serviceName.compare(serviceName.size()-6,6,"data:i")==0) ) {
                     simpleServiceName = serviceName.substr(4,serviceName.size()-4-7);
                     
                     // load the runnable configurations
                     start = 0;
                     stop = simpleServiceName.find_first_of("/", start);
                     resourceSpace = simpleServiceName.substr(start,stop-start);
                     resourceName = simpleServiceName.substr(stop-start,simpleServiceName.size());	      
                     resInfo.name = resourceName;
                     resInfo.space = resourceSpace;
                     
                     if(!isTemporaryPort(resInfo.name)) {
                         effectorInfo.push_back(resInfo);
                         effectorList.addResource(simpleServiceName);
                         cout << "found control basis effector: " << simpleServiceName.c_str() << endl;

                         // now load any transformations on those configurations, possible through applying jacobians
                         if(showVirtualEffectors) {
                             for(int i=0; i<jacInfo.size(); i++) {
                                 if( (jacInfo[i].inputSpace == resourceSpace) ) {
                                     simpleServiceName = jacInfo[i].outputSpace + "/" + resourceName;
                                     effectorList.addResource(simpleServiceName);
                                     cout << "found virtual control basis effector: " << simpleServiceName.c_str() << endl;
                                 } else if( (jacInfo[i].outputSpace == resourceSpace) ) {
                                     simpleServiceName = jacInfo[i].inputSpace + resourceName;
                                     effectorList.addResource(simpleServiceName);
                                     cout << "found virtual control basis effector: " << simpleServiceName.c_str() << endl;
                                 }
                             }
                         }
                     } else {
                         cout << "rejecting effector: " << simpleServiceName.c_str() << endl;
                     }
                     
                 }
                 
                 //                } //             
             } 
         } 
     }   
 }

bool CBAPIWindow::isTemporaryPort(string s) {

    bool ret = false;
    string pf_str;
    int idx;

    for(unsigned int i=0; i<pfInfo.size(); i++) {
        idx = pfInfo[i]->name.find("/");
        if(idx == string::npos) continue;
        pf_str = pfInfo[i]->name.substr(idx);
        if(s.find(pf_str) != string::npos) {
            ret = true;
            return ret;
        }
    }

    return ret;

}
 void CBAPIWindow::loadPotentialFunctionsFromFile() {

     FILE *fp;
     string ICUB_ROOT(getenv("ICUB_ROOT"));    
     string fname = ICUB_ROOT + "/src/controlBasis/potentialFunctions/potentialFunctions.dat";
     cout << fname << endl;

     if( (fp=fopen(fname.c_str(), "r")) == NULL ) {
         cout << "problem opening \'" << fname.c_str() << "\' for reading!!!" << endl;
         pfInfo.clear();
         return;
     }

     string pfName;
     string pfSpace;
     string pfHasReference;

     char line[128];
     string lineStr;

     int start,stop;

     potentialFunctionList.clear();

     while(fgets(line, 128, fp) != NULL) {

         PotentialFunctionInfo *info = new PotentialFunctionInfo();        

         lineStr = string(line);

         start = 0;

         stop = lineStr.find_first_of(" \n", start);
         pfName = lineStr.substr(start,stop-start);
         start = lineStr.find_first_not_of(" ", stop+1);

         stop = lineStr.find_first_of(" \n", start);
         pfHasReference = lineStr.substr(start,stop-start);
         start = lineStr.find_first_not_of(" ", stop+1);

         stop = lineStr.find_first_of(" \n", start);
         pfSpace = lineStr.substr(start,stop-start);
         start = lineStr.find_first_not_of(" ", stop+1);

         info->name = pfName;
         info->space = pfSpace;
         if(pfHasReference=="true") 
             info->hasReference = true; 
         else 
             info->hasReference = false; 
         pfInfo.push_back(info);

         potentialFunctionList.addResource(pfName);

     }

     fclose(fp);
 }

 void CBAPIWindow::loadPotentialFunctions() {

     for(int i=0; i<pfInfo.size(); i++) {
         delete pfInfo[i];
     }
     pfInfo.clear();
     potentialFunctionList.clear();

     for(int i=0; i<PotentialFunctionFactory::instance().getNumRegisteredPotentialFunctions(); i++) {

         PotentialFunctionInfo *info = new PotentialFunctionInfo();
         info->name = PotentialFunctionFactory::instance().getPotentialFunctionInfo(i).name;
         info->space = PotentialFunctionFactory::instance().getPotentialFunctionInfo(i).space;
         info->hasReference = PotentialFunctionFactory::instance().getPotentialFunctionInfo(i).hasReference;
         pfInfo.push_back(info);
         potentialFunctionList.addResource(PotentialFunctionFactory::instance().getName(i));   

         cout << "Loaded PF: " << PotentialFunctionFactory::instance().getName(i).c_str() << endl;
     }

 }

 void CBAPIWindow::loadJacobians() {

     FILE *fp;
     string ICUB_ROOT(getenv("ICUB_ROOT"));    
     string fname = ICUB_ROOT + "/src/controlBasis/jacobians/jacobians.dat";
     cout << fname << endl;

     if( (fp=fopen(fname.c_str(), "r")) == NULL ) {
         cout << "problem opening \'" << fname.c_str() << "\' for reading!!!" << endl;
         jacInfo.clear();
         return;
     }

     string jacName;
     string jacSpaceIn;
     string jacSpaceOut;    
     char line[128];
     string lineStr;    
     int start,stop;
     JacobianInfo info;

     while(fgets(line, 128, fp) != NULL) {

         lineStr = string(line);
         start = 0;

         stop = lineStr.find_first_of(" \n", start);
         jacName = lineStr.substr(start,stop-start);
         start = lineStr.find_first_not_of(" ", stop+1);

         stop = lineStr.find_first_of(" \n", start);
         jacSpaceIn = lineStr.substr(start,stop-start);
         start = lineStr.find_first_not_of(" ", stop+1);

         stop = lineStr.find_first_of(" \n", start);
         jacSpaceOut = lineStr.substr(start,stop-start);
         start = lineStr.find_first_not_of(" ", stop+1);

         info.name = jacName;
         info.inputSpace = jacSpaceIn;
         info.outputSpace = jacSpaceOut;
         jacInfo.push_back(info);

     }

     fclose(fp);

 }

 void CBAPIWindow::on_add_button_clicked() { 

     cout << "ADD" << endl; 

     string sen = sensorList.getSelected();
     string ref = referenceList.getSelected();
     string pf = potentialFunctionList.getSelected();
     string eff = effectorList.getSelected();

     if( (sen=="") || (pf=="") || (eff=="") ) {
         cout << "Please select control resources" << endl;
         return;
     }

     int id;
     for(id=0; id<pfInfo.size(); id++) {
         if(pfInfo[id]->name == pf) break;
     }
     if(pfInfo[id]->hasReference) {
         cout << "Please select control resources" << endl;
         if(ref=="") return;
     }

     cout << "adding controller: " << endl;
     cout << "\t " << pf.c_str() << endl;
     cout << "\t " << sen.c_str() << endl;
     if(ref != "") cout << "\t " << ref.c_str() << endl;
     cout << "\t " << eff.c_str() << endl;

     float gain_p = 1;
     float gain_i = 0;
     float gain_d = 0;
     string gainStr;

     gainStr = controllerGainPEntry.get_text();
     cout << "got input KP: " << gainStr << endl;
     sscanf(gainStr.c_str(),"%f",&gain_p);
     cout << "Setting controller P-gain: " << gain_p << endl;

     if(usePDControl && usePIDControl) {
         usePDControl = false;
     }

     if(usePIDControl) {
         gainStr = controllerGainIEntry.get_text();
         cout << "got input KI: " << gainStr << endl;
         sscanf(gainStr.c_str(),"%f",&gain_i);
         cout << "Setting controller I-gain: " << gain_i << endl;
     }

     if(usePDControl || usePIDControl) {
         gainStr = controllerGainDEntry.get_text();
         cout << "got input KD: " << gainStr << endl;
         sscanf(gainStr.c_str(),"%f",&gain_d);
         cout << "Setting controller D-gain: " << gain_d << endl;
     }

     cbapi.addControllerToLaw(sen, ref, pf, eff, useJacobianTranspose, (double)gain_p, (double)gain_i, (double)gain_d);

     cbapi.usePDControl(usePDControl);
     cbapi.usePIDControl(usePIDControl);

     float nf = 0;
     string nfStr = nullspaceFactorEntry.get_text();
     cout << "got input NF: " << nfStr << endl;
     sscanf(nfStr.c_str(),"%f",&nf);
     cout << "Setting Nullspace Factor: " << nf << endl;
     cbapi.setNullspaceFactor(nf);

     char c[32];
     int n = cbapi.getNumControllers()-1;
     sprintf(c, "%d", n);
     string nStr = string(c);

     controlDefinitionText.append_text("Controller[" + nStr + "]\n","black");
     controlDefinitionText.append_text("\tsensor: " + sen +"\n","purple");
     if(ref != "") {
         controlDefinitionText.append_text("\tref: " + ref +"\n","purple");
     }
     controlDefinitionText.append_text("\tpf: " + pf +"\n","blue");
     controlDefinitionText.append_text("\teffector: " + eff +"\n","red");
     controlOutputText.append_text("added controller " + nStr +" to law\n", "yellow");

 }

 void CBAPIWindow::on_clear_button_clicked() { 
     cout << "CLEAR" << endl; 
     if(controlLawRunning) {
         dataThread->stopUpdateThread();
         cbapi.stopControlLaw();
         controlOutputText.append_text("stopping control law\n", "red");
         controlLawRunning = false;
     }
     if(cbapi.getNumControllers()>0) {
         cout << "clearing control law" << endl; 
         cbapi.clearControlLaw();
         cout << "clearing text window" << endl; 
         controlDefinitionText.clear_text();
         controlOutputText.append_text("clearing control law\n", "purple");
     }
 }

 void CBAPIWindow::on_run_button_clicked() { 
     cout << "RUN" << endl; 
     if(sequenceRunning) {
         cout << "CBAPI Can't run control law because sequence is running!!" << endl;
         return;
     }
     if(cbapi.getNumControllers()>0) {       
         controlOutputText.append_text("running control law\n","orange");
         cbapi.runControlLaw();
         dataThread->connectCBAPIObjects(&cbapi,&controlOutputText, false);
         dataThread->startUpdateThread();
         controlLawRunning = true;
     }
 }

 void CBAPIWindow::on_stop_button_clicked() { 
     cout << "STOP" << endl; 
     if(controlLawRunning) {
         dataThread->pauseUpdateThread();
         cbapi.pauseControlLaw();
         controlOutputText.append_text("stopping control law\n","red");
         controlLawRunning = false;
     }
     cout << "STOP FINISHED" << endl; 
 }

 void CBAPIWindow::on_refresh_button_clicked() { 
     cout << "REFRESH" << endl; 
     refreshResourceList();
     loadPotentialFunctions();
 }

 void CBAPIWindow::on_add_to_sequence_button_clicked() { 
     cout << "ADD TO SEQUENCE" << endl; 

     string sen = sensorList.getSelected();
     string ref = referenceList.getSelected();
     string pf = potentialFunctionList.getSelected();
     string eff = effectorList.getSelected();

     if( (sen=="") || (pf=="") || (eff=="") ) {
         cout << "Please select control resources" << endl;
         return;
     }

     int id;
     for(id=0; id<pfInfo.size(); id++) {
         if(pfInfo[id]->name == pf) break;
     }
     if(pfInfo[id]->hasReference) {
         cout << "Please select control resources" << endl;
         if(ref=="") return;
     }

     if(usePDControl && usePIDControl) {
         usePDControl = false;
     }

     cout << "adding sequence controller: " << endl;
     cout << "\t " << pf.c_str() << endl;
     cout << "\t " << sen.c_str() << endl;
     if(ref != "") cout << "\t " << ref.c_str() << endl;
     cout << "\t " << eff.c_str() << endl;

     float gain_p = 1;
     float gain_i = 0;
     float gain_d = 0;
     string gainStr = sequenceGainPEntry.get_text();

     gainStr = sequenceGainPEntry.get_text();
     cout << "got input KP: " << gainStr << endl;
     sscanf(gainStr.c_str(),"%f",&gain_p);
     cout << "Setting controller P-gain: " << gain_p << endl;

     if(usePIDControl) {
         gainStr = sequenceGainIEntry.get_text();
         cout << "got input KI: " << gainStr << endl;
         sscanf(gainStr.c_str(),"%f",&gain_i);
         cout << "Setting controller I-gain: " << gain_i << endl;
     }

     if(usePDControl || usePIDControl) {
         gainStr = sequenceGainDEntry.get_text();
         cout << "got input KD: " << gainStr << endl;
         sscanf(gainStr.c_str(),"%f",&gain_d);
         cout << "Setting controller D-gain: " << gain_d << endl;
     }

     cbapi.addControllerToSequence(sen, ref, pf, eff, useJacobianTranspose, (double)gain_p, (double)gain_i, (double)gain_d);

     cbapi.usePDControl(usePDControl);
     cbapi.usePIDControl(usePIDControl);

     char c[32];
     int n = cbapi.getNumControllers(true)-1;
     sprintf(c, "%d", n);
     string nStr = string(c);

     sequenceControllersText.append_text("\nController[" + nStr + "][0]\n");
     sequenceControllersText.append_text("\tsensor: " + sen +"\n");
     if(ref != "") {
         sequenceControllersText.append_text("\tref: " + ref +"\n");
     }
     sequenceControllersText.append_text("\tpf: " + pf +"\n");
     sequenceControllersText.append_text("\teffector: " + eff +"\n");
     sequenceOutputText.append_text("added controller " + nStr +" to sequence\n","purple");

 }

 void CBAPIWindow::on_add_secondary_to_sequence_button_clicked() { 
     cout << "ADD SECONDARY CONTROLLER TO SEQUENCE" << endl; 

     string sen = sensorList.getSelected();
     string ref = referenceList.getSelected();
     string pf = potentialFunctionList.getSelected();
     string eff = effectorList.getSelected();

     if(usePDControl && usePIDControl) {
         usePDControl = false;
     }

     if( (sen=="") || (pf=="") || (eff=="") ) {
         cout << "Please select control resources" << endl;
         return;
     }

     int id;
     for(id=0; id<pfInfo.size(); id++) {
         if(pfInfo[id]->name == pf) break;
     }
     if(pfInfo[id]->hasReference) {
         cout << "Please select control resources" << endl;
         if(ref=="") return;
     }
     float gain_p = 1;
     float gain_i = 0;
     float gain_d = 0;
     string gainStr = sequenceGainPEntry.get_text();

     gainStr = sequenceGainPEntry.get_text();
     cout << "got input KP: " << gainStr << endl;
     sscanf(gainStr.c_str(),"%f",&gain_p);
     cout << "Setting controller P-gain: " << gain_p << endl;

     if(usePIDControl) {
         gainStr = sequenceGainIEntry.get_text();
         cout << "got input KI: " << gainStr << endl;
         sscanf(gainStr.c_str(),"%f",&gain_i);
         cout << "Setting controller I-gain: " << gain_i << endl;
     }

     if(usePDControl || usePIDControl) {
         gainStr = sequenceGainDEntry.get_text();
         cout << "got input KD: " << gainStr << endl;
         sscanf(gainStr.c_str(),"%f",&gain_d);
         cout << "Setting controller D-gain: " << gain_d << endl;
     }

     bool ret = cbapi.addSecondaryControllerToSequence(sen, ref, pf, eff, useJacobianTranspose, (double)gain_p, (double)gain_i, (double)gain_d);

     cbapi.usePDControl(usePDControl);
     cbapi.usePIDControl(usePIDControl);

     if(ret) {
         char c[32];
         int n = cbapi.getNumControllers(true)-1;
         sprintf(c, "%d", n);
         string nStr = string(c);

         sequenceControllersText.append_text("Controller[" + nStr + "][1]\n");
         sequenceControllersText.append_text("\tsensor: " + sen +"\n");
         if(ref != "") {
             sequenceControllersText.append_text("\tref: " + ref +"\n");
         }
         sequenceControllersText.append_text("\tpf: " + pf +"\n");
         sequenceControllersText.append_text("\teffector: " + eff +"\n");
         sequenceOutputText.append_text("added controller " + nStr +" to sequence\n");
     } else {
        sequenceOutputText.append_text("did not add secondary controller. already set!\n");
    }

}

void CBAPIWindow::on_clear_sequence_button_clicked() { 
    cout << "CLEAR SEQUENCE" << endl; 
    if(sequenceRunning) {
        dataThread->stopUpdateThread();
        cbapi.stopSequence();
        sequenceOutputText.append_text("stopping control sequence\n","red");
        sequenceRunning = false;
    }
    if(cbapi.getNumControllers(true)>0) {
        cout << "clearing sequence" << endl; 
        cbapi.clearSequence();
        sequenceControllersText.clear_text();
        sequenceOutputText.append_text("clearing sequence\n","purple");
    }
}

void CBAPIWindow::on_run_sequence_button_clicked() { 
    cout << "RUN SEQUENCE" << endl; 
    if(controlLawRunning) {
        cout << "CBAPI Can't run sequence because control law is running!!" << endl;
        return;
    }
    if(cbapi.getNumControllers(true)>0) {
        sequenceOutputText.append_text("running sequence\n","orange");
        cbapi.runSequence();
        dataThread->connectCBAPIObjects(&cbapi,&sequenceOutputText, true);
        dataThread->startUpdateThread();
        sequenceRunning = true;
    }
}

void CBAPIWindow::on_fwd_sequence_button_clicked() { 
    cout << "SEQUENCE FWD" << endl; 
    if(sequenceRunning) {
        cbapi.goToNextControllerInSequence();        
    } else {
        cout << "Sequence not running, can't proceed!!" << endl;
    }
}

void CBAPIWindow::on_bk_sequence_button_clicked() { 
    cout << "SEQUENCE BK" << endl; 
    if(sequenceRunning) {
        cbapi.goToPreviousControllerInSequence();        
    } else {
        cout << "Sequence not running, can't go back!!" << endl;
    }
}

void CBAPIWindow::on_stop_sequence_button_clicked() { 
    cout << "STOP SEQUENCE" << endl; 
    if(sequenceRunning) {
        dataThread->stopUpdateThread();
        cbapi.stopSequence();
        sequenceOutputText.append_text("stopping control sequence\n","red");
        sequenceRunning = false;
    }
    cout << "STOP FINISHED" << endl; 
}

int main(int argc, char *argv[]) {
    yarp::os::Network yarp;
    Gtk::Main kit(argc,argv);
    CBAPIWindow mainWindow;
    Gtk::Main::run(mainWindow); 
    cout << "Success!!" << endl << endl;
    return 1;
}

