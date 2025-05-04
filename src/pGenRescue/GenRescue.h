/************************************************************/
/*    NAME: Eric Wang                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: GenRescue.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef GenRescue_HEADER
#define GenRescue_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include "XYPoint.h"
#include "XYSegList.h"
#include <string>

class GenRescue : public AppCastingMOOSApp
{
 public:
   GenRescue();
   ~GenRescue();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();
   void generatePath();
   void addPoint(std::string report);
   void removePoint(std::string report);

 private: // Configuration variables
   double m_visit_radius;

 private: // State variables
   double numPoints;
   std::vector<XYPoint> pointList;
   std::vector<XYPoint> pointHistory;
   std::vector<bool> pointBool;
   double navx;
   double navy;
   bool regenerateFlag;
   std::string m_hostname;     // previously set name of ownship
   std::string m_dest_name;    // previously set name of vehicle to communicate
   std::string m_moos_varname; // previously set name of MOOS variable to send
};

#endif 