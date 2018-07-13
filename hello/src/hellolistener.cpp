#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "templates/simplesubscriber.h"

using namespace std;

/*****************************************************************************
*
***  class HelloListener
*
*****************************************************************************/

class HelloListener : public DSimpleSubscriber<std_msgs::String>
   {
   public:
      HelloListener(const std::string& strTopic, size_t nQueueSize)
         : DSimpleSubscriber(strTopic, nQueueSize)
      {
      return;
      }


      HelloListener(const HelloListener& src) = delete;
      HelloListener(const HelloListener&& src) = delete;

      ~HelloListener() = default;

      HelloListener& operator=(const HelloListener& rhs) = delete;
      HelloListener& operator=(const HelloListener&& rhs) = delete;

   protected:
      virtual void ReceiveMsg(MSGPARM msg) override
         {
         ROS_INFO("Received %s", msg->data.c_str());
         return;
         }

   private:

   }; // end of class HelloListener



int main(int argc, char **argv)
   {
   // Initialize the ROS system.
   ros::init(argc, argv, "hellolistener");

   // Establish this program as a ROS node.
   ros::NodeHandle nh;

   ROS_INFO_STREAM("Listener Starting");
   HelloListener Sub("HelloMessage", 10);
   Sub.Run(nh);

   // Send some output as a log message.
   //ROS_INFO_STREAM("Hello, ROS!");

   return (0);

   }
