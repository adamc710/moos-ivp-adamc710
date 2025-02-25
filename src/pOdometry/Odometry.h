/************************************************************/
/*    NAME: Adam Cohen                                      */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: Odometry.h                                      */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef Odometry_HEADER
#define Odometry_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <queue>

class Odometry : public AppCastingMOOSApp
{
 public:
   Odometry();
   ~Odometry();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();
   bool handleUnitChange(std::string unit_spec);

 private: // Configuration variables
  bool m_first_reading;
  double m_current_x;
  double m_current_y;
  double m_previous_x;
  double m_previous_y;
  double m_total_distance;
  double m_timestamp;
  std::queue<std::pair<double, double>> m_position_queue;
  double m_nav_stale_thresh;
  double m_unit_conversion;  // Multiplier for converting meters to desired units
  std::string m_unit_name;       // Name of the current unit (e.g., "meters", "feet", etc.)
 private: // State variables
};

#endif 
