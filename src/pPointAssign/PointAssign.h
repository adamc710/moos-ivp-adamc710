/************************************************************/
/*    NAME: Your Name                                       */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: PointAssign.h                                   */
/*    DATE: March 13, 2025                                  */
/************************************************************/

#ifndef PointAssign_HEADER
#define PointAssign_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "XYPoint.h"  // Added for visualization
#include <string>
#include <vector>
#include <list>
#include <map>

class PointAssign : public AppCastingMOOSApp
{
 public:
   PointAssign();
   ~PointAssign();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp functions to overload 
   bool buildReport();

 protected:
   void registerVariables();
   void handleVisitPoint(const std::string& point_str);
   void processPointQueue();
   void setupSharingForVehicle(const std::string& vname);
   void configureSharing(const std::string& vname, const std::string& point_str);
   
   // New method for visualization
   void postViewPoint(double x, double y, std::string label, std::string color);

 private: // Configuration variables
   std::vector<std::string> m_vehicle_names;
   bool m_assign_by_region;
   std::map<std::string, std::string> m_vehicle_colors; // Color for each vehicle

 private: // State variables
   std::list<std::string> m_points_to_assign;
   int m_points_received;
   int m_points_assigned;
   bool m_first_point_sent;
   bool m_last_point_sent;
   std::map<std::string, int> m_points_per_vehicle;
};

#endif