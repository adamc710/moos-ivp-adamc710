/************************************************************/
/*    NAME: Adam Cohen                                      */
/*    ORGN: MIT                                             */
/*    FILE: BHV_ZigLeg.cpp                                  */
/*    DATE: 4/24/25                                         */
/************************************************************/

#include <iterator>
#include <cstdlib>
#include <cmath>
#include "MBUtils.h"
#include "BuildUtils.h"
#include "BHV_ZigLeg.h"
#include "XYRangePulse.h"
#include "ZAIC_PEAK.h"

using namespace std;

//---------------------------------------------------------------
// Constructor

BHV_ZigLeg::BHV_ZigLeg(IvPDomain domain) :
  IvPBehavior(domain)
{
  // Provide a default behavior name
  IvPBehavior::setParam("name", "defaultname");

  // Declare the behavior decision space
  m_domain = subDomain(m_domain, "course");

  // Add any variables this behavior needs to subscribe for
  addInfoVars("NAV_X", "required");
  addInfoVars("NAV_Y", "required");
  addInfoVars("NAV_HEADING", "required");
  addInfoVars("WPT_INDEX", "required");
  
  // Initialize member variables
  m_curr_wpt_index = -1;
  m_prev_wpt_index = -1;
  m_wpt_change_time = 0;
  m_zig_start_time = 0;
  m_pulse_range = 20;      // Default value
  m_pulse_duration = 4;    // Default value
  m_zig_angle = 45;        // Default value - 45 degrees
  m_zig_duration = 10;     // Default value - 10 seconds
  m_pulse_pending = false;
  m_zig_active = false;
  m_osx = 0;
  m_osy = 0;
  m_osh = 0;
  m_target_heading = 0;
}

//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_ZigLeg::setParam(string param, string val)
{
  // Convert the parameter to lower case for more general matching
  param = tolower(param);

  // Get the numerical value of the param argument for convenience once
  double double_val = atof(val.c_str());
  
  if((param == "pulse_range") && isNumber(val)) {
    m_pulse_range = double_val;
    return(true);
  }
  else if((param == "pulse_duration") && isNumber(val)) {
    m_pulse_duration = double_val;
    return(true);
  }
  else if((param == "zig_angle") && isNumber(val)) {
    m_zig_angle = double_val;
    return(true);
  }
  else if((param == "zig_duration") && isNumber(val)) {
    m_zig_duration = double_val;
    return(true);
  }

  // If not handled above, then pass it to the superclass
  return(IvPBehavior::setParam(param, val));
}

//---------------------------------------------------------------
// Procedure: onSetParamComplete()
//   Purpose: Invoked once after all parameters have been handled.
//            Good place to ensure all required params have are set.
//            Or any inter-param relationships like a<b.

void BHV_ZigLeg::onSetParamComplete()
{
}

//---------------------------------------------------------------
// Procedure: onHelmStart()
//   Purpose: Invoked once upon helm start, even if this behavior
//            is a template and not spawned at startup

void BHV_ZigLeg::onHelmStart()
{
}

//---------------------------------------------------------------
// Procedure: onIdleState()
//   Purpose: Invoked on each helm iteration if conditions not met.

void BHV_ZigLeg::onIdleState()
{
  // Even in idle state, we want to update our internal state variables
  updateInfoVars();
  checkForWaypointChange();
  checkForZigTrigger();
}

//---------------------------------------------------------------
// Procedure: onCompleteState()

void BHV_ZigLeg::onCompleteState()
{
}

//---------------------------------------------------------------
// Procedure: postConfigStatus()
//   Purpose: Invoked each time a param is dynamically changed

void BHV_ZigLeg::postConfigStatus()
{
}

//---------------------------------------------------------------
// Procedure: onIdleToRunState()
//   Purpose: Invoked once upon each transition from idle to run state

void BHV_ZigLeg::onIdleToRunState()
{
}

//---------------------------------------------------------------
// Procedure: onRunToIdleState()
//   Purpose: Invoked once upon each transition from run to idle state

void BHV_ZigLeg::onRunToIdleState()
{
}

//---------------------------------------------------------------
// Procedure: updateInfoVars()
//   Purpose: Update member variables from InfoBuffer

void BHV_ZigLeg::updateInfoVars()
{
  // Get ownship position and heading
  bool ok1, ok2, ok3;
  m_osx = getBufferDoubleVal("NAV_X", ok1);
  m_osy = getBufferDoubleVal("NAV_Y", ok2);
  m_osh = getBufferDoubleVal("NAV_HEADING", ok3);
  
  if(!ok1 || !ok2 || !ok3) {
    postWMessage("BHV_ZigLeg: Error getting NAV_X, NAV_Y, or NAV_HEADING");
  }
  
  // Check if WPT_INDEX exists in the info_buffer
  if(!m_info_buffer->isKnown("WPT_INDEX")) {
    return;  // Exit if not available - don't log repeatedly
  }
  
  // Get waypoint index
  bool ok4;
  string wpt_index_str = getBufferStringVal("WPT_INDEX", ok4);
  
  if(ok4 && isNumber(wpt_index_str)) {
    m_curr_wpt_index = atoi(wpt_index_str.c_str());
  }
}

//---------------------------------------------------------------
// Procedure: checkForWaypointChange()
//   Purpose: Check if waypoint index has changed and mark the time if it has

void BHV_ZigLeg::checkForWaypointChange()
{
  if (m_curr_wpt_index != m_prev_wpt_index && m_curr_wpt_index >= 0) {
    m_wpt_change_time = getBufferCurrTime();
    m_prev_wpt_index = m_curr_wpt_index;
    m_pulse_pending = true;
    postMessage("ZIGLEG_DETECT", "Waypoint change detected: " + intToString(m_curr_wpt_index));
  }
}

//---------------------------------------------------------------
// Procedure: checkForZigTrigger()
//   Purpose: Check if it's time to trigger a zigleg (5 seconds after waypoint change)

void BHV_ZigLeg::checkForZigTrigger()
{
  if (!m_pulse_pending) {
    return;
  }
  
  double curr_time = getBufferCurrTime();
  double time_since_change = curr_time - m_wpt_change_time;
  
  if (time_since_change >= 5.0) {
    // Send a range pulse for visual feedback
    sendRangePulse();
    
    // Set the target heading as an offset from current heading
    m_target_heading = m_osh + m_zig_angle;
    
    // Normalize to 0-359
    while (m_target_heading >= 360)
      m_target_heading -= 360;
    while (m_target_heading < 0)
      m_target_heading += 360;
    
    // Mark the start time of the zigleg
    m_zig_start_time = curr_time;
    m_zig_active = true;
    m_pulse_pending = false;
    
    postMessage("ZIGLEG_START", "Started zigleg to heading: " + doubleToString(m_target_heading, 1));
  }
}

//---------------------------------------------------------------
// Procedure: sendRangePulse()
//   Purpose: Send a range pulse message for visual feedback

void BHV_ZigLeg::sendRangePulse()
{
  XYRangePulse pulse;
  pulse.set_x(m_osx);
  pulse.set_y(m_osy);
  pulse.set_label("bhv_zigleg");
  pulse.set_rad(m_pulse_range);
  pulse.set_time(getBufferCurrTime());
  pulse.set_color("edge", "yellow");
  pulse.set_color("fill", "yellow");
  pulse.set_duration(m_pulse_duration);

  string spec = pulse.get_spec();
  postMessage("VIEW_RANGE_PULSE", spec);
  
  postMessage("ZIGLEG_PULSE", "Sent pulse at waypoint index " + intToString(m_curr_wpt_index));
}

//---------------------------------------------------------------
// Procedure: buildZigFunction()
//   Purpose: Create an IvPFunction for the zigleg maneuver

IvPFunction* BHV_ZigLeg::buildZigFunction()
{
  ZAIC_PEAK peak(m_domain, "course");
  
  // Set parameters for the ZAIC peak
  peak.setValueWrap(true);
  peak.setSummit(m_target_heading);
  peak.setPeakWidth(0);
  peak.setBaseWidth(180.0);
  peak.setSummitDelta(0);
  peak.setMinMaxUtil(0, 100);
  
  // Extract the IvP function
  IvPFunction *ipf = peak.extractIvPFunction();
  
  // Check for extraction error
  if(!ipf) {
    postWMessage("ZigLeg: Error creating IvP function");
    return(NULL);
  }
  
  // Debug message
  string msg = "ZigLeg: IvP function created with target heading: " + doubleToString(m_target_heading, 1);
  postMessage("ZIGLEG_DEBUG", msg);
  
  return(ipf);
}

//---------------------------------------------------------------
// Procedure: onRunState()
//   Purpose: Invoked each iteration when run conditions have been met.

IvPFunction* BHV_ZigLeg::onRunState()
{
  // Update information from the information buffer
  updateInfoVars();
  
  // Check if the waypoint index has changed
  checkForWaypointChange();
  
  // Check if it's time to trigger a zigleg
  checkForZigTrigger();
  
  // If zigleg is active, check if we should still be producing an objective function
  if (m_zig_active) {
    double curr_time = getBufferCurrTime();
    double time_since_start = curr_time - m_zig_start_time;
    
    if (time_since_start >= m_zig_duration) {
      // Time to end the zigleg
      m_zig_active = false;
      postMessage("ZIGLEG_END", "Ended zigleg maneuver");
      return(NULL);
    }
    
    // Still active, build and return the IvP function
    IvPFunction *ipf = buildZigFunction();
    
    // Set the priority weight
    if(ipf)
      ipf->setPWT(m_priority_wt);
    
    return(ipf);
  }
  
  // Not active, return NULL
  return(NULL);
}