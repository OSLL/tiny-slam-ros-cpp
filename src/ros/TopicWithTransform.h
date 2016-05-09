/*
Copyright (c) 2016 JetBrains Research, Mobile Robot Algorithms Laboratory

Permission is hereby granted, free of charge, to any person obtaining a copy of this 
software and associated documentation files (the "Software"), to deal in the Software 
without restriction, including without limitation the rights to use, copy, modify, merge, 
publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons 
to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies 
or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
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

template <typename MType>
class TopicObserver { // iface
public: // methods
  virtual void handle_transformed_msg(const boost::shared_ptr<MType>,
                                      const tf::StampedTransform&) = 0;
};

template <typename MsgType>
class TopicWithTransform {
  /* NB: wasn't able to implement with TF2 (ROS jade),
         probably because of deadlock
           (https://github.com/ros/geometry2/pull/144)
         Try TF2 with proposed patch.
   */
public: // methods
  TopicWithTransform(ros::NodeHandle nh,
                     const std::string& topic_name,
                     const std::string& target_frame):
    _target_frame{target_frame},
    _subscr{nh, topic_name, SUBSCR_QUEUE_SZ},
    _tf_lsnr{ros::Duration(5.0)},
    _msg_flt{new tf::MessageFilter<MsgType>{
      _subscr, _tf_lsnr, _target_frame, FILTER_QUEUE_SZ}} {

      _msg_flt->registerCallback(&TopicWithTransform::transformed_msg_cb, this);
  }

  void subscribe(std::shared_ptr<TopicObserver<MsgType>> obs) {
    _observers.push_back(obs);
  }
private: // consts
  const uint32_t SUBSCR_QUEUE_SZ = 10000; // elems
  const uint32_t FILTER_QUEUE_SZ = 10000; // elems
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

#ifdef RVIZ_DEBUG
    dbg_fwd_msg(msg);
#endif

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
