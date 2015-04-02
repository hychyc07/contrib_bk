
SET ROBOT=chica
SET APPLICATION_NAME=attentionSimple
SET APPLICATION_PATH=%ICUB_ROOT%/app/%APPLICATION_NAME%
SET NAME_CONTROLBOARD=

SET CAMCALIB_RIGHT_MODULE_NAME=/%ROBOT%/camCalib/right
SET CAMCALIB_RIGHT_CONFIG_FILE=eyes.ini
SET CAMCALIB_RIGHT_CONFIG_GROUP=CAMERA_CALIBRATION_RIGHT

SET SALIENCE_RIGHT_MODULE_NAME=/%ROBOT%/salience/right
SET SALIENCE_RIGHT_CONFIG_FILE=salience.ini

SET EGOSPHERE_MODULE_NAME=/%ROBOT%/egoSphere
SET EGOSPHERE_CONFIG_GROUP=EGO_SPHERE
SET EGOSPHERE_CONFIG_FILE=eyes.ini

SET ATTENTIONSELECTION_MODULE_NAME=/%ROBOT%/attentionSelection
SET ATTENTIONSELECTION_CONFIG_FILE=attentionSelection.ini

SET CONTROLGAZE_MODULE_NAME=/%ROBOT%/controlGaze
SET CONTROLGAZE_CONFIG_FILE=controlGaze.ini

START %ICUB_ROOT%/bin/camCalib --name %CAMCALIB_RIGHT_MODULE_NAME% --context %APPLICATION_NAME%/conf --from %CAMCALIB_RIGHT_CONFIG_FILE% --group %CAMCALIB_RIGHT_CONFIG_GROUP%
START %ICUB_ROOT%/bin/salience --name %SALIENCE_RIGHT_MODULE_NAME% --context %APPLICATION_NAME%/conf --from %SALIENCE_RIGHT_CONFIG_FILE%
START %ICUB_ROOT%/bin/egoSphere --name %EGOSPHERE_MODULE_NAME% --context %APPLICATION_NAME%/conf --from %EGOSPHERE_CONFIG_FILE% --group %EGOSPHERE_CONFIG_GROUP% --controlboard %NAME_CONTROLBOARD%
START %ICUB_ROOT%/bin/attentionSelection --name %ATTENTIONSELECTION_MODULE_NAME% --context  %APPLICATION_NAME%/conf --from %ATTENTIONSELECTION_CONFIG_FILE% --remoteEgoSphere %EGOSPHERE_MODULE_NAME%/conf
START %ICUB_ROOT%/bin/controlGaze --name %CONTROLGAZE_MODULE_NAME% --context  %APPLICATION_NAME%/conf --from %CONTROLGAZE_CONFIG_FILE% --robot %ROBOT%

SLEEP 10

yarp connect /chica/cam/right /chica/camCalib/right/in
yarp connect /chica/camCalib/right/out /chica/salience/right/view
yarp connect /chica/salience/right/map /chica/egoSphere/mapVisual/map_in
yarp connect /chica/egoSphere/map_out /chica/attentionSelection/i:map
yarp connect /chica/controlGaze/remoteEgoSphere /chica/egoSphere/conf
yarp connect /chica/attentionSelection/remoteEgoSphere /chica/egoSphere/conf
yarp connect /chica/attentionSelection/o:velocity /chica/controlGaze/vel
yarp connect /chica/attentionSelection/o:position /chica/controlGaze/pos
yarp connect /chica/controlGaze/status:o /attentionSelection/i:gaze
yarp connect /chica/controlGaze/remoteEgoSphere /chica/egoSphere/conf
