/**
 * \file
 * \brief In this file are implemented 2 classes.
 * TopicObserver - abstract base class.
 * TopicWithTransform - class, that synchronizes odometry and transform.
 */

#ifndef __TOPIC_WITH_TRANSFORM_H
#define __TOPIC_WITH_TRANSFORM_H

#include <string>
#include <vector>
#include <memory>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <tf/transform_listener.h>
#include <boost/shared_ptr.hpp>

/**
 * \brief Base class; subclasses of this class observe laser scans and odometry.
 */

// TODO: make this class inner
template <typename MType>
class TopicObserver { // iface
public: // methods
  virtual void handle_transformed_msg(const boost::shared_ptr<MType>,
                                      const tf::StampedTransform&) = 0;
};

/**
 * \brief This class synchronizes transform and odometry.
 */

// TODO: add scan drop
template <typename MsgType>
class TopicWithTransform {
  /* NB: wasn't able to implement with TF2 (ROS jade),
         probably because of deadlock
           (https://github.com/ros/geometry2/pull/144)
         Try TF2 with proposed patch.
   */
public: // methods
  // TODO: copy, move ctrs, dtor
  TopicWithTransform(ros::NodeHandle nh,
                     const std::string& topic_name,
                     const std::string& target_frame):
    _target_frame{target_frame},
    _subscr{nh, topic_name, SUBSCR_QUEUE_SZ},
    _tf_lsnr{ros::Duration(5.0)},
    _msg_flt{new tf::MessageFilter<MsgType>{
      _subscr, _tf_lsnr, _target_frame, FILTER_QUEUE_SZ}} {
      _msg_flt->registerCallback(&TopicWithTransform::transformed_msg_cb,
                                  this);
  }

/**
 * Function which add the object of TopicObserver to _observers container.
 * \param obs Shared pointer on the TopicObserver
 */

  void subscribe(std::shared_ptr<TopicObserver<MsgType>> obs) {
    _observers.push_back(obs);
  }
private: // consts
  const uint32_t SUBSCR_QUEUE_SZ = 10000; ///< Size of the queue of subscribe.
  const uint32_t FILTER_QUEUE_SZ = 10000; ///< Size of the queue of filters.
private: // methods
  void transformed_msg_cb(const boost::shared_ptr<MsgType> msg) {
    tf::StampedTransform transform;
    std::string msg_frame_id =
      ros::message_traits::FrameId<MsgType>::value(*msg);
    ros::Time msg_time =
      ros::message_traits::TimeStamp<MsgType>::value(*msg);

    try{
      // NB: using last available msg (time = 0) causes issues
      //     on running with higher rate.
      _tf_lsnr.lookupTransform(_target_frame, msg_frame_id,
                               msg_time, transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("Transform retrieval failed. %s",ex.what());
      return;
    }

    for (auto obs : _observers) {
      if (auto obs_ptr = obs.lock()) {
        obs_ptr->handle_transformed_msg(msg, transform);
      }
    }
  }
private: // fields
  std::string _target_frame;
  message_filters::Subscriber<MsgType> _subscr;
  tf::TransformListener _tf_lsnr;
  std::unique_ptr<tf::MessageFilter<MsgType>> _msg_flt;
  std::vector<std::weak_ptr<TopicObserver<MsgType>>> _observers;
};

#endif // macro guard
