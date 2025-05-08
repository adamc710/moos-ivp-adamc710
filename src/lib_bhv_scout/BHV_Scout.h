/*****************************************************************/
/*    NAME: Adam Cohen                                           */
/*    ORGN: Dept of Mechanical Eng / MIT/WHOI Joint Program      */
/*    FILE: BHV_Scout.h                                          */
/*    DATE: May 1 2025                                           */
/*                                                               */
/* This program is free software; you can redistribute it and/or */
/* modify it under the terms of the GNU General Public License   */
/* as published by the Free Software Foundation; either version  */
/* 2 of the License, or (at your option) any later version.      */
/*                                                               */
/* This program is distributed in the hope that it will be       */
/* useful, but WITHOUT ANY WARRANTY; without even the implied    */
/* warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR       */
/* PURPOSE. See the GNU General Public License for more details. */
/*                                                               */
/* You should have received a copy of the GNU General Public     */
/* License along with this program; if not, write to the Free    */
/* Software Foundation, Inc., 59 Temple Place - Suite 330,       */
/* Boston, MA 02111-1307, USA.                                   */
/*****************************************************************/
 
#ifndef BHV_SCOUT_HEADER
#define BHV_SCOUT_HEADER

#include "IvPBehavior.h"
#include "XYSegList.h"
#include "XYPoint.h"
#include "XYPolygon.h"
#include <vector>

class BHV_Scout : public IvPBehavior {
public:
  BHV_Scout(IvPDomain);
  ~BHV_Scout() {};
  
  bool         setParam(string, string);
  void         onIdleState();
  void         onEveryState(string);
  IvPFunction* onRunState();

protected:
  // Core behavior functions
  void         updateScoutPoint();
  void         postViewPoint(bool);
  IvPFunction* buildFunction();
  
  // Path following and zig-zag functions
  void         postWaypoint(const XYPoint&);
  void         postReturnToPath();
  XYPoint      createLegPoint(double angle, double distance);
  
  // Swimmer detection and reporting
  void         reportSwimmer(const XYPoint&);
  void         handleNewSwimmer(const XYPoint&);

private:
  // State variables
  double       m_osx;           // Current X position
  double       m_osy;           // Current Y position
  double       m_ptx;           // Target X position
  double       m_pty;           // Target Y position
  bool         m_pt_set;        // Whether target point is set
  double       m_curr_time;     // Current time
  
  // Configuration parameters
  double       m_desired_speed; // Desired speed in m/s
  double       m_capture_radius;// Radius to consider waypoint reached
  string       m_tmate;         // Teammate (rescue vehicle) name
  
  // Path following variables
  XYSegList    m_rescue_path;   // Path received from rescue vehicle
  double       m_last_zig_time; // Last time zig-zag was executed
  bool         m_zig_direction; // Current zig-zag direction
  XYPolygon    m_rescue_region; // Region to scout
  bool         m_zig_trigger;
  double       m_trigger_angle;
  double       m_zig_angle;
  double       m_set_offset;
  double       m_zig_duration;
  double       m_zig_duration_portion;

  
  // Swimmer tracking
  vector<XYPoint> m_swimmer_targets; // List of detected swimmers
};

#define IVP_EXPORT_FUNCTION
extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_Scout(domain);}
}
#endif
