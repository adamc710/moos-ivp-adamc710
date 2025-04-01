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
  m_visit_radius = 5.0;  // Default visit radius of 5 meters
  
  // Initialize state variables
  m_nav_x = 0;
  m_nav_y = 0;
  m_received_first_point = false;
  m_received_last_point = false;
  m_mission_complete = false;
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
    else if(key == "GENPATH_REGENERATE") {
      // Handle regeneration request
      if(!m_mission_complete) {
        reportEvent("Received regeneration request");
        regeneratePath();
      }
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

bool GenPath::Iterate() {
    AppCastingMOOSApp::Iterate();

    // Check visited points in every iteration
    checkVisitedPoints();

    // If received the last point and path isn't complete, generate path
    if (m_received_last_point && !m_path_complete && m_received_first_point) {
        generatePath();
        m_path_complete = true;
    }

    AppCastingMOOSApp::PostReport();
    return true;
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
    else if(param == "visit_radius") {
      m_visit_radius = atof(value.c_str());
      if(m_visit_radius < 0) {
        reportConfigWarning("Invalid visit_radius value (must be non-negative): " + value);
        m_visit_radius = 5.0;  // Reset to default
      }
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
  Register("GENPATH_REGENERATE", 0);
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
  m_msgs << "  Visit Radius: " << m_visit_radius << " meters" << endl;
  
  m_msgs << "State:" << endl;
  m_msgs << "  Vehicle Position: " << m_nav_x << ", " << m_nav_y << endl;
  m_msgs << "  Received First Point: " << (m_received_first_point ? "yes" : "no") << endl;
  m_msgs << "  Received Last Point: " << (m_received_last_point ? "yes" : "no") << endl;
  m_msgs << "  Path Complete: " << (m_path_complete ? "yes" : "no") << endl;
  m_msgs << "  Mission Complete: " << (m_mission_complete ? "yes" : "no") << endl;
  
  m_msgs << "Points Received: " << m_points.size() << endl;
  
  if (!m_points.empty()) {
    ACTable actab(5);
    actab << "Index | X | Y | ID | Visited";
    actab.addHeaderLines();
    
    for (size_t i = 0; i < m_points.size(); i++) {
      string visited = isPointVisited(i) ? "yes" : "no";
      actab << std::to_string(i)
            << m_points[i].get_vx()
            << m_points[i].get_vy()
            << m_points[i].get_label()
            << visited;
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
    m_mission_complete = false;
    m_visited_points.clear();
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
  
  // Initialize point as not visited
  m_visited_points.push_back(false);
  
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
  vector<bool> is_visited = m_visited_points;
  vector<int> remaining_indices;
  
  // Find unvisited points
  for (size_t i = 0; i < remaining_points.size(); i++) {
    if (!is_visited[i]) {
      remaining_indices.push_back(i);
    }
  }
  
  // If all points are visited, mission is complete
  if (remaining_indices.empty()) {
    reportEvent("All points have been visited - mission complete!");
    Notify("MISSION_COMPLETE", "true");
    m_mission_complete = true;
    return;
  }
  
  // Start from current vehicle position
  double current_x = m_nav_x;
  double current_y = m_nav_y;
  
  // Greedy algorithm to find shortest path through unvisited points
  while (!remaining_indices.empty()) {
    double min_dist = -1;
    int min_index_pos = -1;
    
    // Find the closest unvisited point
    for (size_t i = 0; i < remaining_indices.size(); i++) {
      int pt_index = remaining_indices[i];
      double dist = calculateDistance(current_x, current_y, 
                                     remaining_points[pt_index].get_vx(), 
                                     remaining_points[pt_index].get_vy());
      if (min_dist < 0 || dist < min_dist) {
        min_dist = dist;
        min_index_pos = i;
      }
    }
    
    // Add the closest point to the path
    if (min_index_pos >= 0) {
      int pt_index = remaining_indices[min_index_pos];
      XYPoint closest = remaining_points[pt_index];
      path.add_vertex(closest.get_vx(), closest.get_vy());
      
      // Update current position
      current_x = closest.get_vx();
      current_y = closest.get_vy();
      
      // Remove the point from the remaining indices
      remaining_indices.erase(remaining_indices.begin() + min_index_pos);
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
// Procedure: checkVisitedPoints

void GenPath::checkVisitedPoints() {
    bool any_visited = false;

    for (size_t i = 0; i < m_points.size(); i++) {
        if (m_visited_points[i]) continue;

        double dist = calculateDistance(m_nav_x, m_nav_y,
                                        m_points[i].get_vx(),
                                        m_points[i].get_vy());

        if (dist <= m_visit_radius) {
            m_visited_points[i] = true;
            any_visited = true;
            reportEvent("Point " + m_points[i].get_label() + " visited!");
        }
    }

/*
    // Check if all points have been visited
    bool all_visited = std::all_of(m_visited_points.begin(), m_visited_points.end(),
                                   [](bool visited) { return visited; });

    if (all_visited) {
        reportEvent("All points have been visited - mission complete!");
        Notify("MISSION_COMPLETE", "true");
        m_mission_complete = true;
    } else if (!any_visited && m_path_complete) {
        // Post regeneration request if no new points were visited and path is complete
        Notify("GENPATH_REGENERATE", "true");
    }
    */
}


//---------------------------------------------------------
// Procedure: regeneratePath

void GenPath::regeneratePath() {
    bool all_visited = std::all_of(m_visited_points.begin(), m_visited_points.end(),
                                   [](bool visited) { return visited; });

    if (all_visited) {
        reportEvent("Regeneration requested but all points already visited - mission complete!");
        Notify("MISSION_COMPLETE", "true");
        m_mission_complete = true;
        return;
    }

    generatePath();
    m_path_complete = false;  // Reset path completion flag
    reportEvent("Path regenerated with unvisited points");
}


//---------------------------------------------------------
// Procedure: calculateDistance

double GenPath::calculateDistance(double x1, double y1, double x2, double y2)
{
  return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

//---------------------------------------------------------
// Procedure: isPointVisited

bool GenPath::isPointVisited(size_t index)
{
  if (index < m_visited_points.size()) {
    return m_visited_points[index];
  }
  return false;
}
