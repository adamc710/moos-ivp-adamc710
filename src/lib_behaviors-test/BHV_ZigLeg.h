/************************************************************/
/*    NAME: Adam Cohen                                      */
/*    ORGN: MIT                                             */
/*    FILE: BHV_ZigLeg.h                                    */
/*    DATE: 4/24/25                                         */
/************************************************************/

#ifndef ZigLeg_HEADER
#define ZigLeg_HEADER

#include <string>
#include "IvPBehavior.h"
#include "IvPFunction.h"

class BHV_ZigLeg : public IvPBehavior {
public:
  BHV_ZigLeg(IvPDomain);
  ~BHV_ZigLeg() {};
  
  bool         setParam(std::string, std::string);
  void         onSetParamComplete();
  void         onCompleteState();
  void         onIdleState();
  void         onHelmStart();
  void         postConfigStatus();
  void         onRunToIdleState();
  void         onIdleToRunState();
  IvPFunction* onRunState();

protected: // Local Utility functions
  void         updateInfoVars();
  void         checkForWaypointChange();
  void         checkForZigTrigger();
  void         sendRangePulse();
  IvPFunction* buildZigFunction();

protected: // Configuration parameters
  double       m_pulse_range;
  double       m_pulse_duration;
  double       m_zig_angle;     // Heading offset in degrees
  double       m_zig_duration;  // How long to maintain zigleg in seconds

protected: // State variables
  double       m_osx;           // Ownship X position
  double       m_osy;           // Ownship Y position
  double       m_osh;           // Ownship heading
  double       m_target_heading;// Target heading for zigleg
  int          m_curr_wpt_index;
  int          m_prev_wpt_index;
  double       m_wpt_change_time;
  double       m_zig_start_time;
  bool         m_pulse_pending;
  bool         m_zig_active;
};

#define IVP_EXPORT_FUNCTION

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_ZigLeg(domain);}
}
#endif