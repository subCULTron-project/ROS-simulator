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
#ifndef GEOUTILITIES_HPP_
#define GEOUTILITIES_HPP_
#define _USE_MATH_DEFINES
#include <utility>
#include <math.h>
#include <Eigen/Dense>

namespace labust
{
	/**
	 * \todo Add Vincenty formula: http://en.wikipedia.org/wiki/Vincenty's_formulae
	 */
	namespace tools
	{
		///Constants for quick projection
		static const double deg2rad = M_PI/180;
		static const double rad2deg = 180/M_PI;
		static const double radius = 6378137;
		static const double ratio = 0.99664719;

		/**
		 * The function calculates meters per latitude degree.
		 * The conversion was taken from:
		 * http://en.wikipedia.org/wiki/Geographic_coordinate_system#Cartesian_coordinates
		 */
		inline double mpdlat(double lat)
		{
			return (111132.954 - 559.822*cos(2*lat*deg2rad) + 1.175*cos(4*lat*deg2rad));
		};
		/**
		 * The function calculates meters per longitude degree.
		 * The conversion was taken from:
		 * http://en.wikipedia.org/wiki/Geographic_coordinate_system#Cartesian_coordinates
		 */
		inline double mpdlon(double lat)
		{
			return (radius*cos(atan(ratio*tan(lat*deg2rad)))*deg2rad);
		};
		/**
		 * The function converts degrees to meters.
		 *
		 * \param difflat North distance in degrees.
		 * \param difflon East distance in degrees.
		 * \param lat Latitude position in decimal degrees.
		 *
		 * \return Returns the latitude and longitude distance in relative meters.
		 */
		inline std::pair<double,double> deg2meter(double difflat, double difflon, double lat)
    				{
			return std::pair<double,double>(difflat*mpdlat(lat),difflon*mpdlon(lat));
    				}
		/**
		 * The function converts meters to relative degrees.
		 *
		 * \param x North distance in meters.
		 * \param y East distance in meters.
		 * \param lat Latitude position in decimal degrees.
		 *
		 * \return Returns the relative angles of the distances.
		 */
		inline std::pair<double,double> meter2deg(double x, double y, double lat)
    				{
			return std::pair<double,double>(x/mpdlat(lat),y/mpdlon(lat));
    				};



		/**
		 * Constants and full WGS84<->ECEF<->NED conversions.
		 *
		 * Conversion references:
		 * 1. USER’s HANDBOOK ON DATUM TRANSFORMATIONS INVOLVING WGS 84
		 * 2. J. Zhu, "Conversion of Earth-centered Earth-fixed coordinates
		 * to geodetic coordinates," IEEE Transactions on Aerospace and
		 * Electronic Systems, vol. 30, pp. 957-961, 1994
		 * 3. http://www.navipedia.net/index.php/Transformations_between_ECEF_and_ENU_coordinates
		 */
		///Semi-major axis of the local geodetic datum ellipsoid
		static const double a = 6378137;
		///Flattening of the local geodetic datum ellipsoid
		static const double f = 1/298.257223563;
		///Semi-minor axis of the local geodetic datum ellipsoid
		static const double b = a*(1-f);
		///First eccentricity squared
		static const double esq = 2*f-f*f;
		///Secondary eccentricity squared
		static const double ecsq = (a*a - b*b)/(b*b);


		/**
		 * Converts the WGS84 geodetic coordinates into geocentric ECEF coordinates
		 * @param geo Geodetic latitude, longitude in decimal degrees and altitude in meters.
		 * 			  Internally, the vector must be (longitude,latitude,altitude).
		 * @return ECEF coordinates in meters
		 */
		inline Eigen::Vector3d geodetic2ecef(const Eigen::Vector3d& geo)
		{
			enum {lon=0, lat, alt};
			double rlat(geo(lat)*M_PI/180);
			double rlon(geo(lon)*M_PI/180);
			double R = a/sqrt(1-esq*sin(rlat)*sin(rlat));

			return Eigen::Vector3d(
					(R+geo(alt))*cos(rlat)*cos(rlon),
					(R+geo(alt))*cos(rlat)*sin(rlon),
					(R+geo(alt) - esq*R)*sin(rlat));
		}

		/**
		 * Converts the geocentric ECEF coordinates into WGS84 geodetic coordinates.
		 * @param x Geocentric position in meters.
		 * @param y Geocentric position in meters.
		 * @param z Geocentric position in meters.
		 * @return Geodetic coordinates in decimal degrees and altitude in meters.
		 */
		inline Eigen::Vector3d ecef2geodetic(const Eigen::Vector3d& xyz)
		{
			double x(xyz(0)), y(xyz(1)), z(xyz(2));

			double r = sqrt(x*x + y*y);
			double F = 54*(b*b)*(z*z);
			double G = r*r + (1-esq)*z*z - esq*(a*a - b*b);
			double c = esq*esq*F*r*r/(G*G*G);
			double s = cbrt(1+c+sqrt(c*c+2*c));
			double P = F/(3*(s+1/s+1)*(s+1/s+1)*G*G);
			double Q = sqrt(1+2*esq*esq*P);
			double r0 = -P*esq*r/(1+Q) + sqrt(0.5*a*a*(1+1/Q)
					- P*(1-esq)*z*z/(Q*(1+Q)) - 0.5*P*r*r);
			double re = r-esq*r0;
			double U = sqrt(re*re + z*z);
			double V = sqrt(re*re + (1-esq)*z*z);
			double z0 = b*b*z/(a*V);

			return Eigen::Vector3d(
					atan2(y,x)*180/M_PI,
					atan((z+ecsq*z0)/r)*180/M_PI,
					U*(1-b*b/(a*V)));
		}

		///Returns the rotation matrix between ECEF and NED frame
		inline Eigen::Matrix3d nedrot(const Eigen::Vector3d& geo)
		{
			enum {lon=0, lat, alt};
			double rlat(geo(lat)*M_PI/180);
			double rlon(geo(lon)*M_PI/180);

//			Eigen::Matrix3d rot;
//			rot<<-sin(rlat)*cos(rlon), -sin(rlon), -cos(rlon)*cos(rlat),
//					-sin(rlon)*sin(rlat), 	-cos(rlon), -sin(rlon)*cos(rlat),
//					cos(rlat),0,-sin(rlat);

			double pitch = M_PI/2 + rlat;
			double yaw = -rlon;

			Eigen::Matrix3d rz, ry;
			rz << cos(yaw), sin(yaw),0,
					-sin(yaw), cos(yaw), 0,
					0, 0, 1;
			ry << cos(pitch), 0, -sin(pitch),
					0,1,0,
					sin(pitch), 0, cos(pitch);

//			return rot;
			return rz*ry;
		}

		///Returns the rotation matrix between ECEF and NWU frame
		inline Eigen::Matrix3d nwurot(const Eigen::Vector3d& geo)
		{
			enum {lon=0, lat, alt};
			double rlat(geo(lat)*M_PI/180);
			double rlon(geo(lon)*M_PI/180);

//			Eigen::Matrix3d rot;
//			rot<<-sin(rlat)*cos(rlon), -sin(rlon), -cos(rlon)*cos(rlat),
//					-sin(rlon)*sin(rlat), 	-cos(rlon), -sin(rlon)*cos(rlat),
//					cos(rlat),0,-sin(rlat);

			double roll = M_PI/2 - rlat;
			double yaw = M_PI+rlon;

			Eigen::Matrix3d rz, rx;
			rz << cos(yaw), sin(yaw),0,
					-sin(yaw), cos(yaw), 0,
					0, 0, 1;
			rx << 1, 0 ,0,
				  0, cos(roll), sin(roll),
				  0, -sin(roll),cos(roll);

//			return rot;
			return rz*rx;
		}

		/**
		 * Converts the geocentric ECEF coordinates into local NED coordinates.
		 */
		inline Eigen::Vector3d ecef2ned(const Eigen::Vector3d& xyz, const Eigen::Vector3d& geo)
		{
			Eigen::Vector3d xyz0 = geodetic2ecef(geo);
			Eigen::Matrix3d mrot = nedrot(geo);
			return mrot.transpose() * (xyz - xyz0);
		}

		/**
		 * Converts the local NED coordinates into geocentric ECEF coordinates.
		 */
		inline Eigen::Vector3d ned2ecef(const Eigen::Vector3d& ned, const Eigen::Vector3d& geo)
		{
			Eigen::Vector3d xyz0 = geodetic2ecef(geo);
			Eigen::Matrix3d mrot = nedrot(geo);
			return mrot * ned + xyz0;
		}
	}
}
/* GEOUTILITIES_HPP_ */
#endif
