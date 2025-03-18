/************************************************************/
/*    NAME: Adam Cohen                                      */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: PointAssign.cpp                                 */
/*    DATE: March 13, 2025                                  */
/************************************************************/

#include <iterator>
#include <algorithm>
#include <cstdlib>
#include "MBUtils.h"
#include "ACTable.h"
#include "PointAssign.h"
#include "XYPoint.h"    // For visualization

using namespace std;

//---------------------------------------------------------
// Constructor

PointAssign::PointAssign()
{
  // Initialize member variables
  m_assign_by_region = false;
  m_points_received = 0;
  m_points_assigned = 0;
  m_first_point_sent = false;
  m_last_point_sent = false;
  
  m_vehicle_colors["HENRY"] = "yellow";
  m_vehicle_colors["GILDA"] = "red";
}

//---------------------------------------------------------
// Destructor

PointAssign::~PointAssign()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail

bool PointAssign::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();
    string sval   = msg.GetString(); 

    if(key == "VISIT_POINT") {
      // New visit point received
      handleVisitPoint(sval);
    }
    else if(key != "APPCAST_REQ") {
      reportRunWarning("Unhandled Mail: " + key);
    }
  }
  return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer

bool PointAssign::OnConnectToServer()
{
  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()

bool PointAssign::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // If we have received all points but haven't sent "lastpoint" yet
  if (m_points_to_assign.empty() && !m_last_point_sent && m_first_point_sent) {
    // Send "lastpoint" to all vehicles
    for (size_t i = 0; i < m_vehicle_names.size(); i++) {
      string vname = (m_vehicle_names[i]);
      string var_name = "VISIT_POINT_" + vname;
      Notify(var_name, "lastpoint");
      
      //uFldShoreBroker sharing message with the vehicle
      configureSharing(vname, "lastpoint");
    }
    m_last_point_sent = true;
    reportEvent("Sent 'lastpoint' to all vehicles");
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}



//---------------------------------------------------------
// Procedure: OnStartUp()

bool PointAssign::OnStartUp()
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
    if(param == "vname") {
      // Add vehicle name to list
      m_vehicle_names.push_back(toupper(value));
      handled = true;
    }
    else if(param == "assign_by_region") {
      // Set assignment method
      if(tolower(value) == "true") {
        m_assign_by_region = true;
      }
      else {
        m_assign_by_region = false;
      }
      handled = true;
    }
    else if(param == "vehicle_color") {
      string vname = toupper(biteStringX(value, ','));
      string color = value;
      if(!vname.empty() && !color.empty()) {
        m_vehicle_colors[vname] = color;
        handled = true;
      }
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }

  if(m_vehicle_names.empty()) {
    reportConfigWarning("No vehicle names specified in configuration");
  }
  else {
    string vnames = "";
    for(size_t i=0; i<m_vehicle_names.size(); i++) {
      if(i > 0) vnames += ", ";
      vnames += m_vehicle_names[i];
      
      // Assign vehicle color
      string vname = m_vehicle_names[i];
      if(m_vehicle_colors.find(vname) == m_vehicle_colors.end()) {
        if(i == 0) m_vehicle_colors[vname] = "yellow";
        else if(i == 1) m_vehicle_colors[vname] = "red";
        else m_vehicle_colors[vname] = "red";
      }
      
      // Configure sharing for each vehicle
      setupSharingForVehicle(vname);
    }
    reportEvent("Vehicle names: " + vnames);
  }

  reportEvent("Assignment method: " + (m_assign_by_region ? string("By Region") : string("Alternating")));

  // Visualize the east/west boundary if using region-based assignment
  if(m_assign_by_region) {
    // Draw a line at x=87.5 (the region boundary)
    double boundary_x = 87.5;
    for(double y = -175; y <= -25; y += 15) {
      postViewPoint(boundary_x, y, "boundary_" + doubleToStringX(y), "blue");
    }
    reportEvent("Visualized east/west boundary at x=87.5");
  }

  registerVariables();
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables

void PointAssign::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("VISIT_POINT", 0);
}

//---------------------------------------------------------
// Procedure: buildReport()

bool PointAssign::buildReport()
{
  m_msgs << "==========================================" << endl;
  m_msgs << "pPointAssign Report" << endl;
  m_msgs << "==========================================" << endl;
  
  m_msgs << "Configuration:" << endl;
  m_msgs << "  Vehicle Names: ";
  for(size_t i=0; i<m_vehicle_names.size(); i++) {
    if(i > 0) m_msgs << ", ";
    string vname = m_vehicle_names[i];
    m_msgs << vname << " (color: " << m_vehicle_colors[vname] << ")";
  }
  m_msgs << endl;
  
  m_msgs << "  Assignment Method: " << (m_assign_by_region ? "By Region" : "Alternating") << endl;
  m_msgs << "Statistics:" << endl;
  m_msgs << "  Points Received: " << m_points_received << endl;
  m_msgs << "  Points Assigned: " << m_points_assigned << endl;
  
  m_msgs << "Status:" << endl;
  m_msgs << "  First Point Sent: " << (m_first_point_sent ? "Yes" : "No") << endl;
  m_msgs << "  Last Point Sent: " << (m_last_point_sent ? "Yes" : "No") << endl;
  
  m_msgs << "Points Per Vehicle:" << endl;
  for(map<string, int>::iterator it = m_points_per_vehicle.begin(); it != m_points_per_vehicle.end(); ++it) {
    m_msgs << "  " << it->first << ": " << it->second << endl;
  }
  
  return(true);
}

//---------------------------------------------------------
// Procedure: handleVisitPoint

void PointAssign::handleVisitPoint(const string& point_str)
{
  // Increment received points counter
  m_points_received++;
  
  // Store the point for later assignment
  m_points_to_assign.push_back(point_str);
  
  // Process the queue of points to assign
  processPointQueue();
}

//---------------------------------------------------------
// Procedure: processPointQueue

void PointAssign::processPointQueue()
{
  // If no vehicles, return
  if(m_vehicle_names.empty()) {
    return;
  }
  
  // If haven't sent "firstpoint" yet, send it to all vehicles
  if(!m_first_point_sent) {
    for(size_t i=0; i<m_vehicle_names.size(); i++) {
      string vname = m_vehicle_names[i];
      string var_name = "VISIT_POINT_" + vname;
      Notify(var_name, "firstpoint");
      
      // Configure uFldShoreBroker to share this message with the vehicle
      configureSharing(vname, "firstpoint");
      
      // Initialize the points per vehicle counter
      m_points_per_vehicle[vname] = 0;
    }
    m_first_point_sent = true;
    reportEvent("Sent 'firstpoint' to all vehicles");
  }
  
  // Process all points in the queue
  while(!m_points_to_assign.empty()) {
    string point_str = m_points_to_assign.front();
    m_points_to_assign.pop_front();
    
    // Determine which vehicle should get this point
    string vehicle_name;
    
    // Extract coordinates and ID from the point string
    double x = 0, y = 0;
    string id = "";
    
    // Parse the point string to extract x, y, and id
    vector<string> parts = parseString(point_str, ',');
    for(size_t i=0; i<parts.size(); i++) {
      string part = stripBlankEnds(parts[i]);
      if(strBegins(part, "x="))
        x = atof(part.substr(2).c_str());
      else if(strBegins(part, "y="))
        y = atof(part.substr(2).c_str());
      else if(strBegins(part, "id="))
        id = part.substr(3);
    }
    
    if(m_assign_by_region) {
      // Assign by region (east-west)
      if(x < 87.5) {
        // West region
        vehicle_name = m_vehicle_names[0];
      } else {
        // East region
        vehicle_name = m_vehicle_names[m_vehicle_names.size() > 1 ? 1 : 0];
      }
    } else {
      // Assign alternating
      vehicle_name = m_vehicle_names[m_points_assigned % m_vehicle_names.size()];
    }
    
    // Visualize the point assignment using the appropriate vehicle color
    string color = m_vehicle_colors[vehicle_name];
    postViewPoint(x, y, "visit_" + id, color);
    
    // Send the point to the selected vehicle
    string var_name = "VISIT_POINT_" + vehicle_name;
    Notify(var_name, point_str);
    
    // Configure uFldShoreBroker to share this message with the vehicle
    configureSharing(vehicle_name, point_str);
    
    // Update counters
    m_points_assigned++;
    m_points_per_vehicle[vehicle_name]++;
    
    reportEvent("Assigned point to " + vehicle_name + ": " + point_str);
  }
}

//---------------------------------------------------------
// Procedure: setupSharingForVehicle

void PointAssign::setupSharingForVehicle(const string& vname)
{
  string var_name = "VISIT_POINT_" + vname;
  
  // Configure uFldShoreBroker to route this variable to the vehicle
  string route_str = "ROUTE = " + var_name + ",VISIT_POINT," + vname;
  Notify("USR_BROKER_CONFIG", route_str);
  
  reportEvent("Set up sharing for " + vname + ": " + route_str);
}

//---------------------------------------------------------
// Procedure: configureSharing

void PointAssign::configureSharing(const string& vname, const string& point_str)
{
}

//---------------------------------------------------------
// Procedure: postViewPoint

void PointAssign::postViewPoint(double x, double y, string label, string color)
{
  XYPoint point(x, y);
  point.set_label(label);
  point.set_color("vertex", color);
  point.set_param("vertex_size", "4");

  string spec = point.get_spec();
  Notify("VIEW_POINT", spec);
  
  // For debugging
  reportEvent("Posted view point: " + label + " at x=" + doubleToStringX(x) + ", y=" + doubleToStringX(y) + ", color=" + color);
}

