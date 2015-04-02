// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <iCub/Hand.h>

namespace iCub {


int HandVocab::fromString(const std::string& input) {
  // definitely needs optimizing :-)
  if (input=="RIGHT") return (int)RIGHT;
  if (input=="LEFT") return (int)LEFT;
  if (input=="INDIFF") return (int)INDIFF;
  return -1;
}
std::string HandVocab::toString(int input) {
  switch((Hand)input) {
  case RIGHT:
    return "RIGHT";
  case LEFT:
    return "LEFT";
  case INDIFF:
    return "INDIFF";
  }
  return "";
}
} // namespace


