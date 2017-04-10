/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, LABUST, UNIZG-FER
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the LABUST nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#ifndef ROSBAGUTILITIES_HPP_
#define ROSBAGUTILITIES_HPP_

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace labust {
	namespace tools {

    class RosbagReader {
      public:
        RosbagReader() {}

        RosbagReader(std::string bag_filename) {
          bag_name = bag_filename;
        }
        
        ~RosbagReader() {
          bag.close();
        }
        
        void setBag(std::string bag_filename) {
          bag_name = bag_filename;
        }

        void addTopic(std::string topic_name) {
          topics.push_back(topic_name);
        }

        void open() {
          bag.open(bag_name.c_str(), rosbag::bagmode::Read);
          bag_view.addQuery(bag, rosbag::TopicQuery(topics));
          message_it = bag_view.begin();
        }

        bool done() {
          return message_it == bag_view.end();
        }

        template <class T>
        T next() {
          if (topics.size() > 1) {
            ROS_ERROR("This function cannot be used with multiple topics.");
            exit(1);
          }
          rosbag::MessageInstance m = *(message_it++);
          return *(m.instantiate<T>());
        }

        rosbag::MessageInstance nextMessageInstance() {
          return *(message_it++);
        }

      private:
        rosbag::Bag bag;
        rosbag::View bag_view;
        rosbag::View::iterator message_it; 
        std::string bag_name;
        std::vector<std::string> topics;
    };

  }
}
/* ROSBAGUTILITIES_HPP_ */
#endif
