#ifndef SIMPLESERVICE_H
#define SIMPLESERVICE_H

/*****************************************************************************
 ******************************  I N C L U D E  ******************************
 ****************************************************************************/

#include <ros/ros.h>
#include <string>

/*****************************************************************************
 *
 ***  template <typename SERVICE> class DSimpleService
 *
 * Base class for providing a ROS service.  Simply derive a new class from
 * this template class providing the type of service and override the pure
 * virtual function PerformService to receive the request input and return
 * the output in the response.
 *
 *****************************************************************************/

template <typename SERVICE>
class DSimpleService
   {
   public:
      using REQUEST = typename SERVICE::Request;
      using RESPONSE = typename SERVICE::Response;

      DSimpleService(const std::string& strService) : m_strService{strService}
         {
         return;
         }

      DSimpleService(const DSimpleService& src) = delete;
      DSimpleService(const DSimpleService&& src) = delete;

      virtual ~DSimpleService() = default;

      DSimpleService& operator=(const DSimpleService& rhs) = delete;
      DSimpleService& operator=(const DSimpleService&& rhs) = delete;

      // Call this function on your derived class instance to advertise the service and
      // register the callback handler.  The handler called will be in your class.
      void Run(ros::NodeHandle& nh)
         {
         ros::ServiceServer server = nh.advertiseService(m_strService, &DSimpleService::PerformService, this);
         ros::spin();

         return;
         }


   protected:
      // Name of the service to be advertised
      std::string m_strService;

      // Callback for performing the service.  Overload this method with your version.
      virtual bool PerformService(REQUEST& /* req */, RESPONSE& /* resp */) = 0;

   private:

   }; // end of class DSimpleService

#endif // SIMPLESERVICE_H
