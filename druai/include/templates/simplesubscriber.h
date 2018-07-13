#ifndef SIMPLESUBSCRIBER_H
#define SIMPLESUBSCRIBER_H

/*****************************************************************************
 ******************************  I N C L U D E  ******************************
 ****************************************************************************/

#include <ros/ros.h>
#include <string>

/*****************************************************************************
 *
 ***  template <typename MSGTYPE> class DSimpleSubscriber
 *
 * This template class provides the basis to easily build a simple subscriber.
 * Derive a class from this template class and supply the message type.  The
 * topic and queue size are provided to the constructor.
 *
 * In your class overload the ProcessMsg member function with your own.  If you
 * need additional varible tracking and member functions, add those also.
 *
 * Create an instance of your class in your code after doing the ros:init()
 * and creating your NodeHandle.  Call the Run function on your instance
 * passing in your NodeHandle.
 *
 *****************************************************************************/

template <typename MSGTYPE>
class DSimpleSubscriber
   {
   public:
      // MSGTYPE is the base ROS message type.  MSGPARM is decorated to match
      // the paramater prototype of the callback function.  This provlides some
      // symmetry with the template parameters of this and DSimplePublisher
      using MSGPARM = const typename MSGTYPE::ConstPtr&;

      DSimpleSubscriber(const std::string& strTopic, size_t nQueueSize)
            : m_strTopic{strTopic}, m_nQueueSize{nQueueSize}
         {
         return;
         }

      DSimpleSubscriber(const DSimpleSubscriber& src) = delete;
      DSimpleSubscriber(const DSimpleSubscriber&& src) = delete;

      virtual ~DSimpleSubscriber() = default;

      DSimpleSubscriber& operator=(const DSimpleSubscriber& rhs) = delete;
      DSimpleSubscriber& operator=(const DSimpleSubscriber&& rhs) = delete;

      // Create the subscriber and run
      void Run(ros::NodeHandle& nh)
         {
         ROS_INFO_STREAM("Base::Run");
         ros::Subscriber sub = nh.subscribe(m_strTopic, m_nQueueSize, &DSimpleSubscriber::ReceiveMsg, this);
         ros::spin();

         return;
         }

   protected:
      std::string m_strTopic;
      size_t m_nQueueSize;

      // Overload this method with your own to process the message
      // The function in your class will be the one actually called
      virtual void ReceiveMsg(MSGPARM /* msg */) = 0;

   private:

   }; // end of class DSimpleSubscriber

#endif // SIMPLESUBSCRIBER_H
