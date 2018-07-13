#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "templates/simplepublisher.h"

using namespace std;

/*****************************************************************************
*
***  class HelloPublisher
*
*****************************************************************************/

class HelloPublisher : public DSimplePublisher<std_msgs::String>
   {
   public:
      HelloPublisher(const std::string& strTopic, size_t nQueueSize, double dRate)
            : DSimplePublisher(strTopic, nQueueSize, dRate)
         {
         return;
         }

      HelloPublisher(const HelloPublisher& src) = delete;
      HelloPublisher(const HelloPublisher&& src) = delete;

      ~HelloPublisher() = default;

      HelloPublisher& operator=(const HelloPublisher& rhs) = delete;
      HelloPublisher& operator=(const HelloPublisher&& rhs) = delete;

   protected:
      virtual void SetupMsg(std_msgs::String& msg) override
         {
         static int nCount = 0;
         stringstream hello;
         hello << "Hello, ROS! " << nCount++;
         msg.data = hello.str();

         ROS_INFO_STREAM(hello.str());

         return;
         }

   private:

   }; // end of class HelloPublisher

int main(int argc, char **argv)
   {
   // Initialize the ROS system.
   ros::init(argc, argv, "hello_ros");

   // Establish this program as a ROS node.
   ros::NodeHandle nh;

   HelloPublisher Pub("HelloMessage", 10, 1);
   Pub.Loop(nh);

   // Send some output as a log message.
   //ROS_INFO_STREAM("Hello, ROS!");

   return (0);

   }
