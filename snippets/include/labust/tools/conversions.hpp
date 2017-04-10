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
 *
 *  Created: 20.05.2013.
 *  Author: Đula Nađ
 *********************************************************************/
#ifndef CONVERSIONS_HPP_
#define CONVERSIONS_HPP_
#include <Eigen/Dense>

#include <geometry_msgs/Quaternion.h>

namespace labust
{
	namespace tools
	{
		/**
		 * The class offers mapping from a XYZ structure to a vector.
		 */
		template <class Point, class Iterator>
		void pointToVector(const Point& point, Iterator& vec, int offset = 0)
		{
			vec[offset+0] = point.x;
			vec[offset+1] = point.y;
			vec[offset+2] = point.z;
		}

		/**
		 * The class offers mapping from a XYZ structure to a vector.
		 */
		template <class Point, class Iterator>
		void vectorToPoint(const Iterator& vec, Point& point, int offset = 0)
		{
			point.x = vec[offset+0];
			point.y = vec[offset+1];
			point.z = vec[offset+2];
		}

		/**
		 * The function offers mapping from NED structure to a vector.
		 */
		template <class Point, class Iterator>
		void nedToVector(const Point& point, Iterator& vec, int offset = 0)
		{
			vec[offset+0] = point.north;
			vec[offset+1] = point.east;
			vec[offset+2] = point.depth;
		}

		/**
		 * The class offers mapping to a RPY structure from a vector.
		 */
		template <class Point, class Iterator>
		void vectorToNED(const Iterator& vec, Point& point, int offset = 0)
		{
			point.north = vec[offset+0];
			point.east = vec[offset+1];
			point.depth = vec[offset+2];
		}

		/**
		 * The function offers mapping from RPY structure to a vector.
		 */
		template <class Point, class Iterator>
		void rpyToVector(const Point& point, Iterator& vec, int offset = 0)
		{
			vec[offset+0] = point.roll;
			vec[offset+1] = point.pitch;
			vec[offset+2] = point.yaw;
		}

		/**
		 * The class offers mapping to a NED structure from a vector.
		 */
		template <class Point, class Iterator>
		void vectorToRPY(const Iterator& vec, Point& point, int offset = 0)
		{
			point.roll = vec[offset+0];
			point.pitch = vec[offset+1];
			point.yaw = vec[offset+2];
		}

		/**
		 * The class offers mapping from auv_msgs disable_axis structure to a vector.
		 */
		template <class Point, class Iterator>
		void vectorToDisableAxis(const Iterator& vec, Point& point)
		{
			point.x = vec[0];
			point.y = vec[1];
			point.z = vec[2];
			point.roll = vec[3];
			point.pitch = vec[4];
			point.yaw = vec[5];
		}

		template <class Point, class Iterator>
		void disableAxisToVector(Point& point, const Iterator& vec)
		{
			vec[0] = point.x;
			vec[1] = point.y;
			vec[2] = point.z;
			vec[3] = point.roll;
			vec[4] = point.pitch;
			vec[5] = point.yaw;
		}

		template <class T>
		void quaternionFromEulerZYX(double roll, double pitch, double yaw, Eigen::Quaternion<T>& q)
		{
			using namespace Eigen;
			Matrix<T,3,3> Cx,Cy,Cz,m;
			typedef Matrix<T,3,1> Vector3;
			Cx = AngleAxis<T>(roll, Vector3::UnitX());
			Cy = AngleAxis<T>(pitch, Vector3::UnitY());
			Cz = AngleAxis<T>(yaw, Vector3::UnitZ());
			//ZYX convention
			m = (Cz*Cy*Cx); //From child to parent
			//m = (Cz*Cy*Cx).transpose(); //From parent to child
			q = Quaternion<T>(m);
		}

		template <class T>
		void quaternionFromEulerZYX(double roll, double pitch, double yaw, T& q)
		{
			Eigen::Quaternion<double> t;
			quaternionFromEulerZYX(roll, pitch, yaw, t);
			q.x = t.x();
			q.y = t.y();
			q.z = t.z();
			q.w = t.w();
		}

		//\todo Test and document this method
		template <class T>
		void eulerZYXFromQuaternion(const T& q, double& roll, double& pitch, double& yaw)
		{
			using namespace Eigen;
			//From child to parent
			roll = atan2(2*(q.y()*q.z() + q.x()*q.w()),1-2*(q.x()*q.x() + q.y()*q.y()));
			pitch = -asin(2*(q.x()*q.z()-q.y()*q.w()));
			yaw = atan2(2*(q.y()*q.x()+q.w()*q.z()),1-2*(q.y()*q.y()+q.z()*q.z()));
			//From parent to child
			//roll = atan2(2*(q.y()*q.z() - q.x()*q.w()),1-2*(q.x()*q.x() + q.y()*q.y()));
			//pitch = -asin(2*(q.x()*q.z() + q.y()*q.w()));
			//yaw = atan2(2*(q.x()*q.y()-q.w()*q.z()),1-2*(q.y()*q.y()+q.z()*q.z()));
		}

		//\todo Test and document this method
		template <class T, class V>
		void eulerZYXFromQuaternion(const T& q, V& vect)
		{
			using namespace Eigen;
			//From child to parent
			enum {roll=0, pitch, yaw};
			vect(roll) = atan2(2*(q.y()*q.z() + q.x()*q.w()),1-2*(q.x()*q.x() + q.y()*q.y()));
			vect(pitch) = -asin(2*(q.x()*q.z()-q.y()*q.w()));
			vect(yaw) = atan2(2*(q.y()*q.x()+q.w()*q.z()),1-2*(q.y()*q.y()+q.z()*q.z()));
			//From parent to child
			//roll = atan2(2*(q.y()*q.z() - q.x()*q.w()),1-2*(q.x()*q.x() + q.y()*q.y()));
			//pitch = -asin(2*(q.x()*q.z() + q.y()*q.w()));
			//yaw = atan2(2*(q.x()*q.y()-q.w()*q.z()),1-2*(q.y()*q.y()+q.z()*q.z()));
		}

		//\todo Test and document this method
		inline void eulerZYXFromQuaternion(const geometry_msgs::Quaternion& q,
				double& roll, double& pitch, double& yaw)
		{
			using namespace Eigen;
			//From child to parent
			roll = atan2(2*(q.y*q.z + q.x*q.w),1-2*(q.x*q.x + q.y*q.y));
			pitch = -asin(2*(q.x*q.z-q.y*q.w));
			yaw = atan2(2*(q.y*q.x+q.w*q.z),1-2*(q.y*q.y+q.z*q.z));
			//From parent to child
			//roll = atan2(2*(q.y*q.z - q.x*q.w),1-2*(q.x*q.x + q.y*q.y));
			//pitch = -asin(2*(q.x*q.z + q.y*q.w));
			//yaw = atan2(2*(q.x*q.y-q.w*q.z),1-2*(q.y*q.y+q.z*q.z));
		}
	}
}

/* CONVERSIONS_HPP_ */
#endif
