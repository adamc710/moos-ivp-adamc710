/************************************************************/
/*    NAME: Adam Cohen                                      */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: Odometry.cpp                                    */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "Odometry.h"
#include <cmath>
#include <string>


using namespace std;

//---------------------------------------------------------
// Constructor()

Odometry::Odometry()
{
  m_first_reading = true;
  m_current_x  = 0.0;
  m_current_y = 0.0;
  m_previous_x = 0.0;
  m_previous_y = 0.0;
  m_total_distance = 0.0;
  m_timestamp = 0.0;
  m_nav_stale_thresh = 10.0;  // Default value if not specified in config
  m_depth_thresh = 999999999;
  m_current_depth = 0.0;
  m_odometry_at_depth = 0.0;
  m_unit_conversion = 1.0;  // Default to meters (no conversion)
  m_unit_name = "meters";   // Default unit name
}

//---------------------------------------------------------
// Destructor

Odometry::~Odometry()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool Odometry::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  // Initialize variables to track new position coordinates
  double new_x = m_current_x;
  double new_y = m_current_y;
  bool got_next_x = false;
  bool got_next_y = false;

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();
    double dval   = msg.GetDouble();
    
    if(key == "NAV_X" || key == "NAV_Y" || key == "NAV_DEPTH") {
      // Update timestamp when receiving any NAV data
      m_timestamp = MOOSTime();
      // Clear any existing NAV timeout warning
      retractRunWarning("NAV_TIMEOUT");
      
      if(key == "NAV_X") { //NAV_X
        new_x = dval;
        if (got_next_y){
          m_position_queue.push(std::make_pair(new_x, new_y));
          got_next_x = false;
          got_next_y = false;
        } else {
          got_next_x = true;
        }
        cout << "Got Nav X" << endl;
      }
      else if(key == "NAV_Y") { // NAV_Y
        new_y = dval;
        if (got_next_x){
          m_position_queue.push(std::make_pair(new_x, new_y));
          got_next_y = false;
          got_next_x = false;
        } else {
          got_next_y = true;
        }
        cout << "Got Nav Y" << endl;
      }
      else if(key == "NAV_DEPTH"){
        m_current_depth = dval;
        cout << "Got Nav Depth" << endl;
      }
    else if(key == "ODOMETRY_UNITS") {
      string value = msg.GetString();
      handleUnitChange(value);
    }
    else if(key != "APPCAST_REQ")
      reportRunWarning("Unhandled Mail: " + key);
   }
   
   
}
    return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool Odometry::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool Odometry::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // Check for NAV timeout using configurable threshold
  double current_time = MOOSTime();
  if (current_time - m_timestamp > m_nav_stale_thresh) {
    reportRunWarning("NAV_TIMEOUT: No NAV_X or NAV_Y updates received in over " + 
                    doubleToString(m_nav_stale_thresh) + " seconds");
  }

  // Process all positions in queue
  while(!m_position_queue.empty()) {
    std::pair<double, double> new_pos = m_position_queue.front();
    m_position_queue.pop();
    
    // Skip distance calculation on first reading
    if(m_first_reading) {
      m_first_reading = false;
      m_current_x = new_pos.first;
      m_current_y = new_pos.second;
    }
    else {
      // Store current position as previous
      m_previous_x = m_current_x;
      m_previous_y = m_current_y;
      
      // Update current position
      m_current_x = new_pos.first;
      m_current_y = new_pos.second;

      // Calculate distance using Pythagorean theorem
      double delta_x = m_current_x - m_previous_x;
      double delta_y = m_current_y - m_previous_y;
      double step_distance = sqrt(delta_x * delta_x + delta_y * delta_y);
      
      // Add to total distance
      m_total_distance += step_distance;

      // Process odometry only if depth > 25 meters
      if(m_current_depth > m_depth_thresh) {
        m_odometry_at_depth += step_distance;
       }
    }
  }

  // When publishing the odometry distance, publish both the original and converted values
  if(m_unit_conversion != 1.0) {
    double converted_dist = m_total_distance * m_unit_conversion;
    Notify("ODOMETRY_DIST_" + toupper(m_unit_name), converted_dist);
  }
  Notify("ODOMETRY_DIST", m_total_distance);  // Always publish in meters
  Notify("ODOMETRY_DIST_AT_DEPTH", m_odometry_at_depth);
  
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool Odometry::OnStartUp()
{
  // Initialize base class
  AppCastingMOOSApp::OnStartUp();

  // Container for configuration parameters
  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  // Attempt to read configuration block for this app
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  // Process each configuration parameter
  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;  // Store original parameter line for warning messages
    string line  = *p;  // Working copy of the parameter line
    // Split line into parameter name and value
    string param = tolower(biteStringX(line, '='));  // Convert to lowercase for case-insensitive comparison
    string value = line;

    // Track whether parameter was handled
    bool handled = false;
    if(param == "nav_stale_thresh") {
      // Configure timeout threshold for NAV updates
      double thresh = atof(value.c_str());
      if (thresh <= 0) {
        reportConfigWarning("nav_stale_thresh must be greater than 0. Using default: 10.0");
      } else {
        m_nav_stale_thresh = thresh;
        handled = true;
      }
    }
    else if(param == "distance_units") {
      // Configure units for distance reporting (meters, feet, etc.)
      handled = handleUnitChange(value);
    }
    else if(param == "depth_thresh") {
      handled = setDoubleOnString(m_depth_thresh, value);  // Placeholder for future parameter
    }
    else if(param == "bar") {
      handled = true;  // Placeholder for future parameter
    }

    // Report any unhandled parameters
    if(!handled)
      reportUnhandledConfigWarning(orig);
  }
  
  // Register for variables we want to receive
  registerVariables();	
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void Odometry::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  Register("NAV_DEPTH", 0);
  Register("ODOMETRY_UNITS", 0);
}


//------------------------------------------------------------
// Procedure: buildReport()

bool Odometry::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File:                                       " << endl;
  m_msgs << "============================================" << endl;
  m_msgs << "Current Odometry:  " << m_total_distance << endl;
  m_msgs << "NAV X:  " << m_current_x << endl;
  m_msgs << "NAV Y:  " << m_current_y << endl;
  m_msgs << "ODOMETRY_DIST_AT_DEPTH: " << m_odometry_at_depth << endl;


  ACTable actab(4);
  actab << "Alpha | Bravo | Charlie | Delta";
  actab.addHeaderLines();
  actab << "one" << "two" << "three" << "four";
  m_msgs << actab.getFormattedString();

  return(true);
}

bool Odometry::handleUnitChange(string unit_spec)
{
  // Convert to lower case for case-insensitive comparison
  unit_spec = tolower(unit_spec);
  
  if(unit_spec == "meters" || unit_spec == "m") {
    m_unit_conversion = 1.0;
    m_unit_name = "meters";
  }
  else if(unit_spec == "feet" || unit_spec == "ft") {
    m_unit_conversion = 3.28084;
    m_unit_name = "feet";
  }
  else if(unit_spec == "yards" || unit_spec == "yd") {
    m_unit_conversion = 1.09361;
    m_unit_name = "yards";
  }
  else if(unit_spec == "kilometers" || unit_spec == "km") {
    m_unit_conversion = 0.001;
    m_unit_name = "kilometers";
  }
  else if(unit_spec == "miles" || unit_spec == "mi") {
    m_unit_conversion = 0.000621371;
    m_unit_name = "miles";
  }
  else {
    reportRunWarning("Unrecognized unit specification: " + unit_spec);
    return false;
  }
  
  return true;
}




