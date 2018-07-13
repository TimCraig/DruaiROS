#ifndef SIMPLEPUBLISHER_H
#define SIMPLEPUBLISHER_H

/*****************************************************************************
 ******************************  I N C L U D E  ******************************
 ****************************************************************************/

#include <ros/ros.h>
#include <string>

/*****************************************************************************
 *
 ***  template <typename MSGTYPE> class DSimplePublisher
 *
 * This template class provides the basis to easily build a simple publisher
 * publishing a message at a steady rate.
 *
 * Derive a class from this template class and supply the message type.  The
 * topic, queue size, and repetition rate are provided to the constructor.
 *
 * In your class overload the SetupMsg member function with your own.  If you
 * need additional varible tracking and member functions, add those also.
 *
 * Create an instance of your class in your code after doing the ros:init()
 * and creating your NodeHandle.  Call the Loop function on your instance
 * passing in your NodeHandle.
 *
 *****************************************************************************/

template <typename MSGTYPE>
class DSimplePublisher
   {
   public:
      DSimplePublisher(const std::string& strTopic, size_t nQueueSize, double dRate)
            : m_strTopic{strTopic}, m_nQueueSize{nQueueSize}, m_dRate{dRate}
         {
         return;
         }

      DSimplePublisher(const DSimplePublisher& src) = delete;
      DSimplePublisher(const DSimplePublisher&& src) = delete;

      virtual ~DSimplePublisher() = default;

      DSimplePublisher& operator=(const DSimplePublisher& rhs) = delete;
      DSimplePublisher& operator=(const DSimplePublisher&& rhs) = delete;

      void Loop(ros::NodeHandle& nh)
         {
         ros::Publisher pub = nh.advertise<MSGTYPE>(m_strTopic, m_nQueueSize);
         ros::Rate rate(m_dRate);
         MSGTYPE msg;

         while (ros::ok())
            {
            SetupMsg(msg);
            pub.publish(msg);
            ros::spinOnce();
            rate.sleep();
            } // end while

         return;
         }

   protected:
      std::string m_strTopic;
      size_t m_nQueueSize;
      double m_dRate;

      // Overload this method with your own to populate the message
      virtual void SetupMsg(MSGTYPE& msg) = 0;

   private:

   }; // end of class DSimplePublisher

#endif // SIMPLEPUBLISHER_H
