CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(attentionSystem)

SET(FASTFILT_DIR ${PROJECT_SOURCE_DIR}/src/fastFilt)
SET(SALIENCELIB_DIR ${PROJECT_SOURCE_DIR}/src/salience)
SET(IKIN_DIR ${PROJECT_SOURCE_DIR}/src/iKin)
SET(PREDICTORS_DIR ${PROJECT_SOURCE_DIR}/src/predictors)
SET(CAMERA_DIR ${PROJECT_SOURCE_DIR}/src/camera)
SET(EGOSPHERELIB_DIR ${PROJECT_SOURCE_DIR}/src/egoSphere)
SET(SPMAP_DIR ${PROJECT_SOURCE_DIR}/src/spMap)
SET(KINEMATICS_DIR ${PROJECT_SOURCE_DIR}/src/kinematics)
SET(ATTENTIONSELECTIONLIB_DIR ${PROJECT_SOURCE_DIR}/src/attentionSelection)
SET(ICUBQWIDGETS_DIR ${PROJECT_SOURCE_DIR}/src/gui/libICUB_QWidgets)
SET(YARPQWIDGETS_DIR ${PROJECT_SOURCE_DIR}/src/gui/libYARP_QWidgets)

SET(BUILD_GUI CACHE BOOL "Do you want to compile the graphical user interface (requires Qt3)?")

ADD_SUBDIRECTORY(src)

# iCub applications of this contrib subfolder
# =====================================================================================

SET(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}"  CACHE PATH "Application path prefix")

# Application 'attentionSimple'
SET(attention_app_name_1 attentionSimple)
FILE(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/app/attentionSimple/conf/*.ini ${CMAKE_CURRENT_SOURCE_DIR}/app/attentionSimple/conf/*.xml)
FILE(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/app/attentionSimple/scripts/*.bat ${CMAKE_CURRENT_SOURCE_DIR}/app/attentionSimple/scripts/*.sh)
icub_app(${attention_app_name_1})
icub_app_install(${attention_app_name_1} FILES ${conf} DESTINATION conf)
icub_app_install(${attention_app_name_1} FILES ${scripts} DESTINATION scripts)

# Collect all applications of this contrib subfolder and create a single build target
icub_app_all()
