/************************************************************/
/*    NAME: Adam Cohen                                      */
/*    ORGN: MIT                                             */
/*    FILE: BHV_Pulse.cpp                                   */
/*    DATE:                                                 */
/************************************************************/

#include <iterator>
#include <cstdlib>
#include "MBUtils.h"
#include "BuildUtils.h"
#include "BHV_Pulse.h"
#include "XYRangePulse.h"

using namespace std;

//---------------------------------------------------------------
// Constructor

BHV_Pulse::BHV_Pulse(IvPDomain domain) :
  IvPBehavior(domain)
{
  // Provide a default behavior name
  IvPBehavior::setParam("name", "defaultname");

  // Declare the behavior decision space
  m_domain = subDomain(m_domain, "course,speed");

  // Add any variables this behavior needs to subscribe for
  // Add it with explicit request for notification
  addInfoVars("NAV_X", "required");
  addInfoVars("NAV_Y", "required");
  addInfoVars("WPT_INDEX", "required");
  
  // Initialize member variables
  m_curr_wpt_index = -1;
  m_prev_wpt_index = -1;
  m_wpt_change_time = 0;
  m_pulse_range = 20;      // Default value
  m_pulse_duration = 4;    // Default value
  m_pulse_pending = false;
  m_osx = 0;
  m_osy = 0;
}

//---------------------------------------------------------------
// Procedure: setParam()

bool BHV_Pulse::setParam(string param, string val)
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

  // If not handled above, then pass it to the superclass
  return(IvPBehavior::setParam(param, val));
}

//---------------------------------------------------------------
// Procedure: onSetParamComplete()
//   Purpose: Invoked once after all parameters have been handled.
//            Good place to ensure all required params have are set.
//            Or any inter-param relationships like a<b.

void BHV_Pulse::onSetParamComplete()
{
}

//---------------------------------------------------------------
// Procedure: onHelmStart()
//   Purpose: Invoked once upon helm start, even if this behavior
//            is a template and not spawned at startup

void BHV_Pulse::onHelmStart()
{
}

//---------------------------------------------------------------
// Procedure: onIdleState()
//   Purpose: Invoked on each helm iteration if conditions not met.

void BHV_Pulse::onIdleState()
{
  // Even in idle state, we want to update our internal state variables
  updateInfoVars();
  checkForWaypointChange();
  checkForPulseTrigger();
}

//---------------------------------------------------------------
// Procedure: onCompleteState()

void BHV_Pulse::onCompleteState()
{
}

//---------------------------------------------------------------
// Procedure: postConfigStatus()
//   Purpose: Invoked each time a param is dynamically changed

void BHV_Pulse::postConfigStatus()
{
}

//---------------------------------------------------------------
// Procedure: onIdleToRunState()
//   Purpose: Invoked once upon each transition from idle to run state

void BHV_Pulse::onIdleToRunState()
{
}

//---------------------------------------------------------------
// Procedure: onRunToIdleState()
//   Purpose: Invoked once upon each transition from run to idle state

void BHV_Pulse::onRunToIdleState()
{
}

//---------------------------------------------------------------
// Procedure: updateInfoVars()
//   Purpose: Update member variables from InfoBuffer

void BHV_Pulse::updateInfoVars()
{
  // Get ownship position
  bool ok1, ok2;
  m_osx = getBufferDoubleVal("NAV_X", ok1);
  m_osy = getBufferDoubleVal("NAV_Y", ok2);
  
  if(!ok1 || !ok2) {
    postWMessage("BHV_Pulse: Error getting NAV_X or NAV_Y");
  }
  
  // Check if WPT_INDEX exists in the info_buffer
  if(!m_info_buffer->isKnown("WPT_INDEX")) {
    return;  // Exit if not available - don't log repeatedly
  }
  
  // Get waypoint index
  bool ok3;
  string wpt_index_str = getBufferStringVal("WPT_INDEX", ok3);
  
  if(ok3 && isNumber(wpt_index_str)) {
    m_curr_wpt_index = atoi(wpt_index_str.c_str());
  }
}

//---------------------------------------------------------------
// Procedure: checkForWaypointChange()
//   Purpose: Check if waypoint index has changed and mark the time if it has

void BHV_Pulse::checkForWaypointChange()
{
  if (m_curr_wpt_index != m_prev_wpt_index && m_curr_wpt_index >= 0) {
    m_wpt_change_time = getBufferCurrTime();
    m_prev_wpt_index = m_curr_wpt_index;
    m_pulse_pending = true;
    postMessage("PULSE_DETECT", "Waypoint change detected: " + intToString(m_curr_wpt_index));
  }
}

//---------------------------------------------------------------
// Procedure: checkForPulseTrigger()
//   Purpose: Check if it's time to trigger a pulse (5 seconds after waypoint change)

void BHV_Pulse::checkForPulseTrigger()
{
  if (!m_pulse_pending) {
    return;
  }
  
  double curr_time = getBufferCurrTime();
  double time_since_change = curr_time - m_wpt_change_time;
  
  if (time_since_change >= 5.0) {
    sendRangePulse();
    m_pulse_pending = false;
  }
}

//---------------------------------------------------------------
// Procedure: sendRangePulse()
//   Purpose: Send a range pulse message

void BHV_Pulse::sendRangePulse()
{
  XYRangePulse pulse;
  pulse.set_x(m_osx);
  pulse.set_y(m_osy);
  pulse.set_label("bhv_pulse");
  pulse.set_rad(m_pulse_range);
  pulse.set_time(getBufferCurrTime());
  pulse.set_color("edge", "yellow");
  pulse.set_color("fill", "yellow");
  pulse.set_duration(m_pulse_duration);

  string spec = pulse.get_spec();
  postMessage("VIEW_RANGE_PULSE", spec);
  
  postMessage("PULSE_DEBUG", "Sent pulse at waypoint index " + intToString(m_curr_wpt_index));
}

//---------------------------------------------------------------
// Procedure: onRunState()
//   Purpose: Invoked each iteration when run conditions have been met.

IvPFunction* BHV_Pulse::onRunState()
{
  // Update information from the information buffer
  updateInfoVars();
  
  // Check if the waypoint index has changed
  checkForWaypointChange();
  
  // Check if it's time to trigger a pulse
  checkForPulseTrigger();

  // This behavior doesn't produce an IvP function, so return NULL
  return(NULL);
}