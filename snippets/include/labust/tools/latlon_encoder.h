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
 *  Author: Dula Nad
 *  Created: 29.04.2013.
 *********************************************************************/
#ifndef SNIPPETS_LATLON_ENCODER_H
#define SNIPPETS_LATLON_ENCODER_H
#include <stdexcept>
#include <cmath>

#define PP_ADD_CASE1(x) \
		case x: \
		this->latlonToBits<x>(lat,lon); \
		break; \

#define PP_ADD_CASE2(x) \
		case x: \
		this->bitsToLatlon<x>(lat,lon); \
		break; \

namespace labust
{
	namespace tools
	{
		/**
		 * The utility for trimming and converting latitude and longitude position
		 * into a defined bit range integer representation.
		 * Supported ranges: 7, 10, 14, 18, 22
		 *
		 * \todo Define how the ranges are specified.
		 */
		class LatLon2Bits
		{
		public:
			void convert(double lat, double lon, int bits = 7);

			int64_t lat, lon;

		protected:
			template <size_t precission>
			void latlonToBits(double lat, double lon);
		};

		template <>
		inline void LatLon2Bits::latlonToBits<22>(double lat, double lon)
		{
			double min = (lon - int(lon))*60;
			this->lon = int((int(lon)+180)*10000 + min*100);

			min = (lat - int(lat))*60;
			this->lat = int((int(lat)+90)*10000 + min*100);
		}
		template <>
		inline void LatLon2Bits::latlonToBits<18>(double lat, double lon)
		{
			//this->lat = int((lat - int(lat))*600000)%100000;
			//this->lon = int((lon - int(lon))*600000)%100000;
			this->lat = int((fabs(lat) - int(fabs(lat)))*1000000)%100000;
			this->lon = int((fabs(lon) - int(fabs(lon)))*1000000)%100000;
		}

		template <>
		inline void LatLon2Bits::latlonToBits<14>(double lat, double lon)
		{
			//double min = (lon - int(lon))*60;
			//this->lon = int((min - int(min))*10000);

			//min = (lat - int(lat))*60;
			//this->lat = int((min - int(min))*10000);
			this->lat = int((fabs(lat) - int(fabs(lat)))*1000000)%10000;
			this->lon = int((fabs(lon) - int(fabs(lon)))*1000000)%10000;
		}

		template <>
		inline void LatLon2Bits::latlonToBits<10>(double lat, double lon)
		{
			this->latlonToBits<14>(lat,lon);
			this->lat/=10;
			this->lon/=10;
		}
		template <>
		inline void LatLon2Bits::latlonToBits<7>(double lat, double lon)
		{
			this->latlonToBits<14>(lat,lon);
			this->lat%=100;
			this->lon%=100;
		}

		inline void LatLon2Bits::convert(double lat, double lon, int bits)
		{
				switch (bits)
				{
					case 0: break;
					PP_ADD_CASE1(7);
					PP_ADD_CASE1(10);
					PP_ADD_CASE1(14);
					PP_ADD_CASE1(18);
					PP_ADD_CASE1(22);
				default:
					throw std::runtime_error("LatLon2Bits: Missing lat-lon conversion definition.");
				}
		}

		/**
		 * The utility for trimming and converting latitude and longitude position
		 * from a defined bit range integer representation. The class needs an initial
		 * latitude and longitude and then propagates the initial latitude and longitude as
		 * data arrives.
		 * Supported ranges: 7, 10, 14, 18, 22
		 *
		 * \todo Define how the ranges are specified.
		 */
		class Bits2LatLon
		{
		public:
			Bits2LatLon(double ilat = 0, double ilon = 0):
				latitude(ilat),
				longitude(ilon),
				initLat(0),
				initLon(0)
			{
				this->normalize();
			};

			void convert(double lat, double lon, int bits = 7);

			void setInitLatLon(double lat, double lon)
			{
				latitude = lat;
				longitude = lon;
				normalize();
			}

			double latitude, longitude;
			double initLat, initLon;

		protected:
			template <size_t precission>
			void bitsToLatlon(double lat, double lon){};
			void normalize()
			{
				//initLat = (floor(latitude*6000)) / 6000.0;
				//initLon = (floor(longitude*6000)) / 6000.0;
				initLat = latitude;
				initLon = longitude;
			}
		};

		template <>
		inline void Bits2LatLon::bitsToLatlon<22>(double lat, double lon)
		{
			double rem = (lat/10000 - int(lat/10000))*10000;
			initLat = this->latitude = int(lat)/10000 + rem/6000 - 90;
			rem = (lon/10000 - int(lon/10000))*10000;
			initLon = this->longitude = int(lon)/10000 + rem / 6000 - 180;
		}
		template <>
		inline void Bits2LatLon::bitsToLatlon<18>(double lat, double lon)
		{
			/*double rem = int(((initLat*60)-int(initLat*60))*10);
			//rem = lat/10000.0 - rem;
			//int sgn = (rem >=0) ? 1: -1;
			//latitude = (((fabs(rem) > 5)? floor(initLat*6+sgn):floor(initLat*6)) + lat/100000.0)/6;
			latitude = floor(initLat*6)) + lat/100000.0)/6;

			rem = int(((initLon*60)-int(initLon*60))*10);
			rem = lon/10000.0 - rem;
			sgn = (rem >=0) ? 1: -1;
			longitude = (((fabs(rem) > 5)? floor(initLon*6+sgn*rem):floor(initLon*6)) + lon/100000.0)/6;

			initLat = (floor(latitude*6000)) / 6000.0;
		  initLon = (floor(longitude*6000)) / 6000.0;
		  */
			latitude = int(initLat*10)/10.0 + (initLat>=0?1:-1)*lat/1000000;
			longitude = int(initLon*10)/10.0 + (initLon>=0?1:-1)*lon/1000000;
		}
		template <>
		inline void Bits2LatLon::bitsToLatlon<14>(double lat, double lon)
		{
			/*if ((lat/10000.0 - fmod(initLat*60,1)) < -0.5)
				latitude = (floor(initLat*60 + 1) + lat/10000.0)/60;
			else if ((lat/10000.0 - fmod(initLat*60,1)) > 0.5)
				latitude = (floor(initLat*60 - 1) + lat/10000.0)/60;
			else
			  latitude = (floor(initLat*60) + lat/10000.0)/60;

			if ((lon/10000.0 - fmod(initLon*60,1)) < -0.5)
				longitude = (floor(initLon*60 + 1) + lon/10000.0)/60;
			else if ((lon/10000.0 - fmod(initLon*60,1)) > 0.5)
				longitude = (floor(initLon*60 - 1) + lon/10000.0)/60;
			else
			  longitude = (floor(initLon*60) + lon/10000.0)/60;

			initLat = (floor(latitude*6000)) / 6000.0;
			initLon = (floor(longitude*6000)) / 6000.0;*/

			latitude = int(initLat*100)/100.0 + (initLat>=0?1:-1)*lat/1000000;
			longitude = int(initLon*100)/100.0 + (initLon>=0?1:-1)*lon/1000000;
		}
		template <>
		inline void Bits2LatLon::bitsToLatlon<10>(double lat, double lon)
		{
			latitude = int(initLat*100)/100.0 + (initLat>=0?1:-1)*lat/100000;
			longitude = int(initLon*100)/100.0 + (initLon>=0?1:-1)*lon/100000;
		}
		template <>
		inline void Bits2LatLon::bitsToLatlon<7>(double lat, double lon)
		{
			throw std::runtime_error("Bits2LatLon: decompression for 7 bits not implemented.");
		}

		inline void Bits2LatLon::convert(double lat, double lon, int bits)
		{
				switch (bits)
				{
					case 0: break;
					//PP_ADD_CASE2(7);
					PP_ADD_CASE2(10);
					PP_ADD_CASE2(14);
					PP_ADD_CASE2(18);
					PP_ADD_CASE2(22);
				default:
					throw std::runtime_error("LatLon2Bits: Missing lat-lon conversion definition.");
				}
		}
	}
}

#undef PP_ADD_CASE1
#undef PP_ADD_CASE2

/* USBL_COMMS_LATLON_ENCODER_H */
#endif



