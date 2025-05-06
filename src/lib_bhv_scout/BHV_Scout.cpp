/*****************************************************************/
/*    NAME: Adam Cohen                                           */
/*    ORGN: Dept of Mechanical Eng / MIT/WHOI Joint Program      */
/*    FILE: BHV_Scout.h                                          */
/*    DATE: May 1 2025                                           */
/*****************************************************************/

#include <cstdlib>
#include <math.h>
#include "BHV_Scout.h"
#include "MBUtils.h"
#include "AngleUtils.h"
#include "BuildUtils.h"
#include "GeomUtils.h"
#include "ZAIC_PEAK.h"
#include "OF_Coupler.h"
#include "XYFormatUtilsPoly.h"
#include "XYFormatUtilsSegl.h"

using namespace std;

//-----------------------------------------------------------
// Constructor()

BHV_Scout::BHV_Scout(IvPDomain gdomain) : 
  IvPBehavior(gdomain)
{
  IvPBehavior::setParam("name", "ben");
 
  // Default values for behavior state variables
  m_osx  = 0;
  m_osy  = 0;

  // All distances are in meters, all speed in meters per second
  // Default values for configuration parameters 
  m_desired_speed  = 1; 
  m_capture_radius = 10;

  m_pt_set = false;
  m_zig_direction = false;
  
  addInfoVars("NAV_X, NAV_Y", "NAV_HEADING");
  addInfoVars("RESCUE_REGION");
  addInfoVars("SCOUTED_SWIMMER");
  addInfoVars("SURVEY_UPDATE");
}

//---------------------------------------------------------------
// Procedure: setParam() - handle behavior configuration parameters

bool BHV_Scout::setParam(string param, string val) 
{
  // Convert the parameter to lower case for more general matching
  param = tolower(param);
  
  bool handled = true;
  if(param == "capture_radius")
    handled = setPosDoubleOnString(m_capture_radius, val);
  else if(param == "desired_speed")
    handled = setPosDoubleOnString(m_desired_speed, val);
  else if(param == "tmate")
    handled = setNonWhiteVarOnString(m_tmate, val);
  else
    handled = false;

  srand(time(NULL));
  
  return(handled);
}

//-----------------------------------------------------------
// Procedure: onEveryState()

void BHV_Scout::onEveryState(string str) 
{
  if(!getBufferVarUpdated("SCOUTED_SWIMMER"))
    return;

  string report = getBufferStringVal("SCOUTED_SWIMMER");
  if(report == "")
    return;

  if(m_tmate == "") {
    postWMessage("Mandatory Teammate name is null");
    return;
  }
  postOffboardMessage(m_tmate, "SWIMMER_ALERT", report);
}

//-----------------------------------------------------------
// Procedure: onIdleState()

void BHV_Scout::onIdleState() 
{
  m_curr_time = getBufferCurrTime();
}

//-----------------------------------------------------------
// Procedure: onRunState()

IvPFunction *BHV_Scout::onRunState() 
{
    // Check for new path updates
    bool ok_path_update = false; 
    string path_spec = getBufferStringVal("SURVEY_UPDATE", ok_path_update);
    
    if(ok_path_update && path_spec != "") {
        m_rescue_path = string2SegList(path_spec);
        if(m_rescue_path.size() > 0) {
            postEventMessage("Received new rescue path with " + 
                           uintToString(m_rescue_path.size()) + " segments");
            // Post the path for visualization
            string path_spec = m_rescue_path.get_spec();
            postMessage("VIEW_SEGLIST", path_spec);
        }
    }
    
    // Get vehicle position
    bool ok1, ok2;
    m_osx = getBufferDoubleVal("NAV_X", ok1);
    m_osy = getBufferDoubleVal("NAV_Y", ok2);
    if(!ok1 || !ok2) {
        postWMessage("No ownship X/Y info in info_buffer.");
        return(0);
    }

    // Execute zig-zag pattern every 15 seconds if we have a valid path
    if(m_rescue_path.size() > 0 && 
       (getBufferCurrTime() - m_last_zig_time > 15)) {
        executeZigDeviation();
        m_last_zig_time = getBufferCurrTime();
    }

    // Check for unregistered swimmers in the area
    if(getBufferVarUpdated("SCOUTED_SWIMMER")) {
        string swimmer_report = getBufferStringVal("SCOUTED_SWIMMER");
        if(swimmer_report != "") {
            // Debug: Print the raw swimmer report
            postEventMessage("Received swimmer report: " + swimmer_report);
            
            // Parse the swimmer position as a polygon
            XYPolygon swimmer_poly = string2Poly(swimmer_report);
            
            // Debug: Print polygon details
            string poly_spec = swimmer_poly.get_spec();
            postEventMessage("Parsed polygon: " + poly_spec);
            
            // Validate the polygon
            if(swimmer_poly.size() >= 3) {
                if(swimmer_poly.is_convex()) {
                    // Get the center point of the swimmer polygon
                    double center_x = swimmer_poly.get_center_x();
                    double center_y = swimmer_poly.get_center_y();
                    XYPoint swimmer_pos(center_x, center_y);
                    
                    // Visualize the swimmer polygon
                    swimmer_poly.set_color("fill", "red");
                    swimmer_poly.set_edge_size(2);
                    postMessage("VIEW_POLYGON", swimmer_poly.get_spec());
                    
                    // Report to rescue vehicle
                    if(m_tmate != "") {
                        string spec = swimmer_pos.get_spec();
                        postOffboardMessage(m_tmate, "SWIMMER_ALERT", spec);
                        postEventMessage("Reported unregistered swimmer to " + m_tmate);
                    }
                } else {
                    // For non-convex polygons, just use the center point
                    double center_x = swimmer_poly.get_center_x();
                    double center_y = swimmer_poly.get_center_y();
                    XYPoint swimmer_pos(center_x, center_y);
                    
                    // Visualize the original polygon in orange
                    swimmer_poly.set_color("fill", "orange");
                    swimmer_poly.set_edge_size(2);
                    postMessage("VIEW_POLYGON", swimmer_poly.get_spec());
                    
                    // Report to rescue vehicle
                    if(m_tmate != "") {
                        string spec = swimmer_pos.get_spec();
                        postOffboardMessage(m_tmate, "SWIMMER_ALERT", spec);
                        postEventMessage("Reported unregistered swimmer (non-convex) to " + m_tmate);
                    }
                }
            } else {
                postWMessage("Invalid swimmer polygon: too few vertices (" + 
                           uintToString(swimmer_poly.size()) + ")");
            }
        }
    }

    // Build the IvP function for navigation
    IvPFunction *ipf = buildFunction();
    if(ipf == 0) 
        postWMessage("Problem Creating the IvP Function");
    
    return(ipf);
}

void BHV_Scout::executeZigDeviation() {
    // Get current position and heading
    bool ok1, ok2, ok3;
    double current_x = getBufferDoubleVal("NAV_X", ok1);
    double current_y = getBufferDoubleVal("NAV_Y", ok2);
    double heading = getBufferDoubleVal("NAV_HEADING", ok3);
    
    if(!ok1 || !ok2 || !ok3) {
        postWMessage("Missing navigation information for zig-zag");
        return;
    }

    // Calculate zig-zag angle (30 degrees to alternating sides)
    double zig_angle = m_zig_direction ? 30 : -30;
    m_zig_direction = !m_zig_direction;
    
    // Calculate new point 75m away at the zig-zag angle
    double new_heading = heading + zig_angle;
    double new_x = current_x + (75 * cos(new_heading * M_PI/180));
    double new_y = current_y + (75 * sin(new_heading * M_PI/180));
    
    // Create a temporary point for visualization
    XYPoint temp_point(new_x, new_y);
    temp_point.set_vertex_color("yellow");
    temp_point.set_vertex_size(3);
    postMessage("VIEW_POINT", temp_point.get_spec());
    
    // Set the target point
    XYPoint target(new_x, new_y);
    postWaypoint(target);
}

//-----------------------------------------------------------
// Procedure: updateScoutPoint()

void BHV_Scout::updateScoutPoint()
{
  if(m_pt_set)
    return;

  string region_str = getBufferStringVal("RESCUE_REGION");
  if(region_str == "")
    postWMessage("Unknown RESCUE_REGION");
  else
    postRetractWMessage("Unknown RESCUE_REGION");

  XYPolygon region = string2Poly(region_str);
  if(!region.is_convex()) {
    postWMessage("Badly formed RESCUE_REGION");
    return;
  }
  m_rescue_region = region;
  
  cout << "updateScoutPoint(): " << endl;
  
  double ptx = 0;
  double pty = 0;
  bool ok = randPointInPoly(m_rescue_region, ptx, pty);
  if(!ok) {
    postWMessage("Unable to generate scout point");
    return;
  }
    
  m_ptx = ptx;
  m_pty = pty;
  m_pt_set = true;
  string msg = "New pt: " + doubleToStringX(ptx) + "," + doubleToStringX(pty);
  postEventMessage(msg);
}

//-----------------------------------------------------------
// Procedure: postViewPoint()

void BHV_Scout::postViewPoint(bool viewable) 
{

  XYPoint pt(m_ptx, m_pty);
  pt.set_vertex_size(5);
  pt.set_vertex_color("orange");
  pt.set_label(m_us_name + "'s next waypoint");
  
  string point_spec;
  if(viewable)
    point_spec = pt.get_spec("active=true");
  else
    point_spec = pt.get_spec("active=false");
  postMessage("VIEW_POINT", point_spec);
}


//-----------------------------------------------------------
// Procedure: buildFunction()

IvPFunction *BHV_Scout::buildFunction() 
{
    // Get vehicle position
    bool ok1, ok2;
    m_osx = getBufferDoubleVal("NAV_X", ok1);
    m_osy = getBufferDoubleVal("NAV_Y", ok2);
    if(!ok1 || !ok2) {
        postWMessage("No ownship X/Y info in info_buffer.");
        return(0);
    }

    // If we have a rescue path, follow it
    if(m_rescue_path.size() > 0) {
        // Get the next point to follow on the path
        // Look ahead 3 points to stay ahead of rescue vehicle
        unsigned int look_ahead = 3;
        unsigned int target_idx = (look_ahead < m_rescue_path.size()) ? look_ahead : 0;
        
        double next_x = m_rescue_path.get_point(target_idx).x();
        double next_y = m_rescue_path.get_point(target_idx).y();
        
        // Calculate distance to next point
        double dist = hypot((next_x - m_osx), (next_y - m_osy));
        
        // If we're close enough to the point, remove it from the path
        if(dist < m_capture_radius) {
            m_rescue_path.get_point(0);
            if(m_rescue_path.size() > 0) {
                target_idx = (look_ahead < m_rescue_path.size()) ? look_ahead : 0;
                next_x = m_rescue_path.get_point(target_idx).x();
                next_y = m_rescue_path.get_point(target_idx).y();
            }
        }
        
        // Build speed function
        ZAIC_PEAK spd_zaic(m_domain, "speed");
        spd_zaic.setSummit(m_desired_speed);
        spd_zaic.setPeakWidth(0.5);
        spd_zaic.setBaseWidth(1.0);
        spd_zaic.setSummitDelta(0.8);  
        if(spd_zaic.stateOK() == false) {
            string warnings = "Speed ZAIC problems " + spd_zaic.getWarnings();
            postWMessage(warnings);
            return(0);
        }
        
        // Build heading function
        double rel_ang_to_wpt = relAng(m_osx, m_osy, next_x, next_y);
        ZAIC_PEAK crs_zaic(m_domain, "course");
        crs_zaic.setSummit(rel_ang_to_wpt);
        crs_zaic.setPeakWidth(0);
        crs_zaic.setBaseWidth(180.0);
        crs_zaic.setSummitDelta(0);  
        crs_zaic.setValueWrap(true);
        if(crs_zaic.stateOK() == false) {
            string warnings = "Course ZAIC problems " + crs_zaic.getWarnings();
            postWMessage(warnings);
            return(0);
        }

        IvPFunction *spd_ipf = spd_zaic.extractIvPFunction();
        IvPFunction *crs_ipf = crs_zaic.extractIvPFunction();

        OF_Coupler coupler;
        IvPFunction *ivp_function = coupler.couple(crs_ipf, spd_ipf, 50, 50);
        return(ivp_function);
    }
    
    // If no path, return null
    return(0);
}

XYPoint BHV_Scout::createLegPoint(double angle, double distance) {
    double current_x = getBufferDoubleVal("NAV_X");
    double current_y = getBufferDoubleVal("NAV_Y");
    
    double new_x = current_x + distance * cos(angle * M_PI/180);
    double new_y = current_y + distance * sin(angle * M_PI/180);
    
    return XYPoint(new_x, new_y);
}

void BHV_Scout::postWaypoint(const XYPoint& point) {
    string point_spec = point.get_spec();
    postMessage("WAYPOINT", point_spec);
}

void BHV_Scout::postReturnToPath() {
    // Get current position
    bool ok1, ok2;
    double current_x = getBufferDoubleVal("NAV_X", ok1);
    double current_y = getBufferDoubleVal("NAV_Y", ok2);
    if(!ok1 || !ok2) {
        postWMessage("No ownship X/Y info in info_buffer.");
        return;
    }

    // Find the nearest point on the rescue path
    if(m_rescue_path.size() == 0) {
        postWMessage("No rescue path available");
        return;
    }

    // Find the closest segment in the path
    double min_dist = -1;
    XYPoint return_point;
    
    for(unsigned int i=0; i<m_rescue_path.size(); i++) {
        XYPoint seg_point = m_rescue_path.get_point(i);
        double dist = hypot((seg_point.x() - current_x), (seg_point.y() - current_y));
        
        if(min_dist == -1 || dist < min_dist) {
            min_dist = dist;
            return_point = seg_point;
        }
    }

    // Post the return waypoint
    postWaypoint(return_point);
}

void BHV_Scout::reportSwimmer(const XYPoint& position) {
    string spec = position.get_spec();
    postMessage("UNREG_SWIMMER", spec);
}

// Add handler for UNREG_SWIMMER messages
void BHV_Scout::handleNewSwimmer(const XYPoint& position) {
    m_swimmer_targets.push_back(position);
    postEventMessage("Added new swimmer to targets list");
}

XYSegList m_rescue_path;
double m_last_zig_time;
bool m_zig_direction;
string m_tmate;  // teammate name (rescue vehicle)

