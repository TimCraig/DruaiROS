#include "templates/simpleservice.h"
#include "hello/helloservice.h"
#include <iostream>
#include <sstream>

/*****************************************************************************
*
***  class HelloServer : public DSimpleService
*
*****************************************************************************/

class HelloServer : public DSimpleService<hello::helloservice>
   {
   public:
      HelloServer(std::string strService) : DSimpleService(strService)
         {
         return;
         }

      HelloServer(const HelloServer& src) = delete;
      HelloServer(const HelloServer&& src) = delete;

      ~HelloServer() = default;

      HelloServer& operator=(const HelloServer& rhs) = delete;
      HelloServer&& operator=(const HelloServer&& rhs) = delete;

   protected:
      virtual bool PerformService(REQUEST& req, RESPONSE& res) override
         {
         std::stringstream ss;
         ss << "Received request: " << req.count << " Returned: " << (req.count + 1);
         res.incremented = ss.str();
         return (true);
         }

   private:

   }; // end of class HelloServer : public DSimpleService


int main(int argc, char *argv[])
   {
   // Initialize the ROS system.
   ros::init(argc, argv, "HelloServer");

   // Establish this program as a ROS node.
   ros::NodeHandle nh;

   HelloServer server("helloservice");
   server.Run(nh);

   return (0);

   }

