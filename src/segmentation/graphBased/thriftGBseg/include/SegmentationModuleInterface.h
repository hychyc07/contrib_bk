// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_SegmentationModuleInterface
#define YARP_THRIFT_GENERATOR_SegmentationModuleInterface

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <Pixel.h>

namespace yarp {
  namespace sig {
    class SegmentationModuleInterface;
  }
}


/**
 * Interface for module that performs graph-based segmentation
 */
class yarp::sig::SegmentationModuleInterface : public yarp::os::Wire {
public:
  SegmentationModuleInterface() { yarp().setOwner(*this); }
/**
 * Set sigma (smoothing) parameter for the algorithm
 * @param newValue new value for sigma parameter
 */
  virtual void set_sigma(const double newValue);
/**
 * Set k (scale factor for boundary-detection threshold function) parameter for the algorithm
 * @param newValue new value for k parameter
 */
  virtual void set_k(const double newValue);
/**
 * Set minRegion parameter for the algorithm, i.e., the minimum size of any segmented component
 * @param newValue new value for minRegion parameter
 */
  virtual void set_minRegion(const double newValue);
/**
 * Get sigma (smoothing) parameter for the algorithm
 * @return current value for sigma parameter
 */
  virtual double get_sigma();
/**
 * Get k (scale factor for boundary-detection threshold function) parameter for the algorithm
 * @return current value for k parameter
 */
  virtual double get_k();
/**
 * Get minRegion parameter for the algorithm, i.e., the minimum size of any segmented component
 * @return current value for minRegion parameter
 */
  virtual double get_minRegion();
/**
 * Get the number of segmented components that have been detected in the last provided image
 * @return number of segmented components
 */
  virtual int32_t get_num_components();
/**
 * Get the list of pixels corresponding to the component to which a given pixel belongs
 * @param objCenter a pixel belonging to the region of interest
 * @return list of pixels belonging to the same component as the input pixels
 */
  virtual std::vector<Pixel>  get_component_around(const Pixel& objCenter);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

#endif

