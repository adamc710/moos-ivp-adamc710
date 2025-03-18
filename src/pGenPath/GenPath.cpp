/************************************************************/
/*    NAME: Adam Cohen                                      */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: GenPath.cpp                                     */
/*    DATE: March 18, 2025                                  */
/************************************************************/

#include <iterator>
#include <algorithm>
#include <cmath>
#include "MBUtils.h"
#include "ACTable.h"
#include "GenPath.h"

using namespace std;

//---------------------------------------------------------
// Constructor

GenPath::GenPath()
{
  // Initialize configuration variables
  m_updates_var = "WPT_UPDATE";
  m_path_complete = false;
  
  // Initialize state variables
  m_nav_x = 0;
  m_nav_y = 0;
  m_received_first_point = false;
  m_received_last_point = false;
}

//---------------------------------------------------------
// Destructor

GenPath::~GenPath()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool GenPath::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();
    string sval   = msg.GetString(); 
    double dval   = msg.GetDouble();

    if(key == "VISIT_POINT") {
      // Process a visit point
      handleVisitPoint(sval);
    }
    else if(key == "NAV_X") {
      m_nav_x = dval;
    }
    else if(key == "NAV_Y") {
      m_nav_y = dval;
    }
    else if(key != "APPCAST_REQ") {
      reportRunWarning("Unhandled Mail: " + key);
    }
  }
  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool GenPath::OnConnectToServer()
{
  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()

bool GenPath::Iterate()
{
  AppCastingMOOSApp::Iterate();
  
  // If received the last point, generate path
  if (m_received_last_point && !m_path_complete && m_received_first_point) {
    generatePath();
    m_path_complete = true;
  }
  
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()

bool GenPath::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = line;

    bool handled = false;
    if(param == "updates_var") {
      m_updates_var = value;
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }
  
  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables

void GenPath::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("VISIT_POINT", 0);
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
}

//---------------------------------------------------------
// Procedure: buildReport()

bool GenPath::buildReport()
{
  m_msgs << "============================================" << endl;
  m_msgs << "pGenPath Report" << endl;
  m_msgs << "============================================" << endl;

  m_msgs << "Configuration:" << endl;
  m_msgs << "  Updates Variable: " << m_updates_var << endl;
  
  m_msgs << "State:" << endl;
  m_msgs << "  Vehicle Position: " << m_nav_x << ", " << m_nav_y << endl;
  m_msgs << "  Received First Point: " << (m_received_first_point ? "yes" : "no") << endl;
  m_msgs << "  Received Last Point: " << (m_received_last_point ? "yes" : "no") << endl;
  m_msgs << "  Path Complete: " << (m_path_complete ? "yes" : "no") << endl;
  
  m_msgs << "Points Received: " << m_points.size() << endl;
  
  if (!m_points.empty()) {
    ACTable actab(4);
    actab << "Index | X | Y | ID";
    actab.addHeaderLines();
    
    for (size_t i = 0; i < m_points.size(); i++) {
      actab << std::to_string(i)
            << m_points[i].get_vx()
            << m_points[i].get_vy()
            << m_points[i].get_label();
    }
    m_msgs << actab.getFormattedString();
  }
  
  if (m_path_complete) {
    m_msgs << endl << "Generated Path:" << endl;
    string path_spec = m_path.get_spec();
    m_msgs << "  " << path_spec << endl;
  }
  
  return(true);
}

//---------------------------------------------------------
// Procedure: handleVisitPoint

void GenPath::handleVisitPoint(const string& point_str)
{
  // Check for firstpoint and lastpoint
  if (point_str == "firstpoint") {
    reportEvent("Received 'firstpoint' message");
    m_received_first_point = true;
    m_points.clear();
    m_path_complete = false;
    return;
  }
  else if (point_str == "lastpoint") {
    reportEvent("Received 'lastpoint' message");
    m_received_last_point = true;
    return;
  }
  
  // Parse regular point
  vector<string> parts = parseString(point_str, ',');
  double x = 0, y = 0;
  string id = "";
  
  for (size_t i = 0; i < parts.size(); i++) {
    string part = stripBlankEnds(parts[i]);
    if (strBegins(part, "x="))
      x = atof(part.substr(2).c_str());
    else if (strBegins(part, "y="))
      y = atof(part.substr(2).c_str());
    else if (strBegins(part, "id="))
      id = part.substr(3);
  }
  
  // Create and store the point
  XYPoint point(x, y);
  point.set_label(id);
  m_points.push_back(point);
  
  reportEvent("Added point to list: " + point_str);
}

//---------------------------------------------------------
// Procedure: generatePath

void GenPath::generatePath()
{
  if (m_points.empty()) {
    reportRunWarning("Cannot generate path with no points");
    return;
  }
  
  // Create a new segment list for the path
  XYSegList path;
  
  // Copy points to a working list
  vector<XYPoint> remaining_points = m_points;
  
  // Start from current vehicle position
  double current_x = m_nav_x;
  double current_y = m_nav_y;
  
  // Greedy algorithm
  while (!remaining_points.empty()) {
    double min_dist = -1;
    int min_index = -1;
    
    // Find the closest point
    for (size_t i = 0; i < remaining_points.size(); i++) {
      double dist = calculateDistance(current_x, current_y, remaining_points[i].get_vx(), remaining_points[i].get_vy());
      if (min_dist < 0 || dist < min_dist) {
        min_dist = dist;
        min_index = i;
      }
    }
    
    // Add the closest point to the path
    if (min_index >= 0) {
      XYPoint closest = remaining_points[min_index];
      path.add_vertex(closest.get_vx(), closest.get_vy());
      
      // Update current position
      current_x = closest.get_vx();
      current_y = closest.get_vy();
      
      // Remove the point from the remaining list
      remaining_points.erase(remaining_points.begin() + min_index);
    }
  }
  
  // Store the generated path
  m_path = path;
  
  // Send the waypoint update
  string update_str = "points = ";
  update_str += m_path.get_spec();
  Notify(m_updates_var, update_str);
  
  reportEvent("Generated and published waypoint path: " + update_str);
}

//---------------------------------------------------------
// Procedure: calculateDistance

double GenPath::calculateDistance(double x1, double y1, double x2, double y2)
{
  return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}