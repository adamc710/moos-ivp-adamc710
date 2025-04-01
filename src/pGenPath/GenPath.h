/************************************************************/
/*    NAME: Adam Cohen                                      */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: GenPath.h                                       */
/*    DATE: March 20, 2025                                  */
/************************************************************/

#ifndef GENPATH_HEADER
#define GENPATH_HEADER

#include <string>
#include <map>
#include <vector>
#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "XYPoint.h"
#include "XYSegList.h"

class GenPath : public AppCastingMOOSApp
{
 public:
   GenPath();
   ~GenPath();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();
   void handleVisitPoint(const std::string& point_str);
   void generatePath();
   void checkVisitedPoints();
   void regeneratePath();
   double calculateDistance(double x1, double y1, double x2, double y2);
   bool isPointVisited(size_t index);

 private: // Configuration variables
   std::string m_updates_var;
   bool m_path_complete;
   double m_visit_radius;

 private: // State variables
   double m_nav_x;
   double m_nav_y;
   std::vector<XYPoint> m_points;
   std::vector<bool> m_visited_points;
   XYSegList m_path;
   bool m_received_first_point;
   bool m_received_last_point;
   bool m_mission_complete;
   bool m_initial_mission_complete;
};

#endif