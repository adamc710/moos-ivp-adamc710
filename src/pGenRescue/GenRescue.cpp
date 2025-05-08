/************************************************************/
/*    NAME: Eric Wang                                       */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: GenRescue.cpp                                   */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#include <iterator>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <vector>
#include <algorithm>
#include "MBUtils.h"
#include "ACTable.h"
#include "GenRescue.h"
#include "XYPoint.h"
#include "XYSegList.h"
#include "NodeRecord.h"
#include "XYFormatUtilsPoint.h"
#include "NodeMessage.h"  // In the lib_ufield library

using namespace std;

//---------------------------------------------------------
// Constructor()

GenRescue::GenRescue()
{
  numPoints = 0;
  navx = 0;
  navy = 0;
  m_visit_radius = 5;
  regenerateFlag = false;
  m_hostname = "abe";
  m_dest_name = "ben";
  m_moos_varname = "SURVEY_UPDATE";
}

//---------------------------------------------------------
// Destructor

GenRescue::~GenRescue()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()

bool GenRescue::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();

#if 0 // Keep these around just for template
    string comm  = msg.GetCommunity();
    double dval  = msg.GetDouble();
    string sval  = msg.GetString(); 
    string msrc  = msg.GetSource();
    double mtime = msg.GetTime();
    bool   mdbl  = msg.IsDouble();
    bool   mstr  = msg.IsString();
#endif
     if (key == "SWIMMER_ALERT") {
        string sval = msg.GetString();
        addPoint(sval);
        regenerateFlag = true;
      }

     else if (key == "NAV_X") {
        navx = msg.GetDouble();
     }

     else if (key == "NAV_Y") {
        navy = msg.GetDouble();
     }

     else if (key == "FOUND_SWIMMER") {
        string sval = msg.GetString();
        removePoint(sval);
        regenerateFlag = true;
     }

     else if (key == "GENRESCUE_REGENERATE") {
        string sval = msg.GetString();
        if (sval == "regenerate_request") {
          regenerateFlag = true;
        }
     }

     else if(key != "APPCAST_REQ") // handled by AppCastingMOOSApp
       reportRunWarning("Unhandled Mail: " + key);
   }
	
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()

bool GenRescue::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
//            happens AppTick times per second

bool GenRescue::Iterate()
{
  AppCastingMOOSApp::Iterate();
  // Do your thing here!
  XYPoint currPos(navx, navy);
  for (int i = 0; i < pointList.size(); ++i) {
    if (pointBool[i] == false) {
      continue;
    }
    double distance = sqrt(pow(currPos.get_vx() - pointList[i].get_vx(), 2) + pow(currPos.get_vy() - pointList[i].get_vy(), 2));
    if (distance <= m_visit_radius) {
      pointBool[i] = false;
      numPoints--;
    }
  }
  if (regenerateFlag) {
    generatePath();
  }
  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
//            happens before connection is open

bool GenRescue::OnStartUp()
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
    if(param == "foo") {
      handled = true;
    }
    else if(param == "bar") {
      handled = true;
    }
    else if(param == "visit_radius") {
      setDoubleOnString(m_visit_radius, value);
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);

  }
  
  registerVariables();	
  Notify("READY_STATUS", m_host_community);
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()

void GenRescue::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("SWIMMER_ALERT", 0);
  Register("FOUND_SWIMMER", 0);
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  Register("GENRESCUE_REGENERATE", 0);
  //Register("WPT_STAT", 0);
  // Register("FOOBAR", 0);
}

//---------------------------------------------------------
// Procedure: addPoint()

void GenRescue::addPoint(std::string report)
{
  size_t ind_x = report.find("x=");
  size_t ind_y = report.find("y=");
  size_t ind_id = report.find("id=");
  string x_string = report.substr(ind_x + 2, ind_y - ind_x - 2);
  string y_string = report.substr(ind_y + 2, report.find(",", ind_y) - ind_y - 2);
  string id_string = report.substr(ind_id + 3, ind_id + 5);
  double x_coord = std::stod(x_string);
  double y_coord = std::stod(y_string);

  bool addToBuffer = true;
  if (pointHistory.size() > 0) {
    for (auto it = pointHistory.begin(); it != pointHistory.end(); ) {
      if (it->get_label() == id_string) {
        addToBuffer = false;
      }
      ++it;
    }
  }

  if (addToBuffer) {
    XYPoint point(x_coord, y_coord);
    point.set_label(id_string);
    pointList.push_back(point);
    pointBool.push_back(true);
    pointHistory.push_back(point);
    numPoints++;
  }
}

//---------------------------------------------------------
// Procedure: removePoint()
void GenRescue::removePoint(std::string report)
{
  size_t ind_id = report.find("id=");
  string id_string = report.substr(ind_id + 3, ind_id + 5);

  for (auto it = pointList.begin(); it != pointList.end(); ) {
    if (it->get_label() == id_string) {
        it = pointList.erase(it);
        numPoints--;
    } else {
        ++it;
    }
  }
}

//---------------------------------------------------------
// Procedure: generatePath()

void GenRescue::generatePath()
{
  regenerateFlag = false;
  if (pointList.size() == 0) {
    Notify("GENRESCUE_REGENERATE", "finished_mission");
    return;
  }

  XYSegList seglist;
  seglist.set_label("Waypoints");
  seglist.set_param("edge_color", "white");

  XYPoint currPos(navx, navy);

  std::vector<XYPoint> pointList_remaining;

  for (size_t i = 0; i < pointList.size(); ++i) {
    if (pointBool[i]) {
      pointList_remaining.push_back(pointList[i]);
    }
  }

  while(!pointList_remaining.empty()) {
    double min_dist = numeric_limits<double>::max();
    vector<XYPoint>::iterator nearest_it;
    int best_index = 0;

    for (auto it = pointList_remaining.begin(); it != pointList_remaining.end(); ++it) {
      double dx = currPos.get_vx() - it->get_vx();
      double dy = currPos.get_vy() - it->get_vy();
      double dist = hypot(dx, dy);

      if (dist < min_dist) {
        min_dist = dist;
        nearest_it = it;
      }
    }

    seglist.add_vertex(nearest_it->x(), nearest_it->y());
    currPos = *nearest_it;
    pointList_remaining.erase(nearest_it);
  }

  //seglist.add_vertex(navx, navy);

  // SEND SEGLIST UPDATE
  string update_str = "points = ";
  update_str       += seglist.get_spec();
  Notify("SURVEY_UPDATE", update_str);

  NodeMessage node_message;

  node_message.setSourceNode(m_hostname);
  node_message.setDestNode(m_dest_name);
  node_message.setVarName(m_moos_varname);
  node_message.setStringVal(seglist.get_spec());

  string msg = node_message.getSpec();

  Notify("NODE_MESSAGE_LOCAL", msg); 
 
  string spec = seglist.get_spec();
  Notify("VIEW_SEGLIST", spec);

  Notify("GENRESCUE_REGENERATE", "regenerated_already");
  //Notify("STATION_KEEP", "false");
  //Notify("SURVEY", "true");
  //Notify("RETURN", "false");
}

//------------------------------------------------------------
// Procedure: buildReport()

bool GenRescue::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "File:                                       " << endl;
  m_msgs << "============================================" << endl;

  ACTable actab(4);
  actab << "numPoints | m_visit_radius | Charlie | Delta";
  actab.addHeaderLines();
  actab << doubleToStringX(numPoints,1) << doubleToStringX(m_visit_radius) << doubleToStringX(pointList.size()) << "four";
  m_msgs << actab.getFormattedString();

  return(true);
}



