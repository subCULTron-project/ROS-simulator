/*
 * publish_from_csv.cpp
 *
 *  Created on: Sep 30, 2014
 *      Author: filip
 */

#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <auv_msgs/BodyVelocityReq.h>
#include <auv_msgs/NED.h>
#include <auv_msgs/DecimalLatLon.h>
#include <std_msgs/Float32.h>

//#include <labust/tools/conversions.hpp>
#include <labust/tools/GeoUtilities.hpp>

namespace labust{
	namespace sim{

		class publishData{

		public:

			//enum {stamp = 0, u = 12, v = 13, w = 14, vx = 15, vy = 16, r = 21, lat = 3, lon = 4, depth = 22, x = 6, y = 7, psi = 11 };
			enum {stamp = 0, u = 1, v = 2, w = 3, vx = -1, vy = -1, r = 4, depth = 6, psi = 5, range = 7, x = 8, y = 9, z = 10 };


			publishData():fileName(""),publishRate(0.1), counter(0){

				ros::NodeHandle nh, ph("~");
				ph.param("filename", fileName, fileName);
				ph.param("rate", publishRate, publishRate);
				//ph.param("lat", origin.latitude, 0.759399454);
				//ph.param("lon",origin.longitude, 0.2860734);

				pubVelocityRef = nh.advertise<auv_msgs::BodyVelocityReq>("velocityRef",1);
				pubRange = nh.advertise<std_msgs::Float32>("rangeMeas",1);
				pubDepth = nh.advertise<std_msgs::Float32>("depth",1);
				pubPosition = nh.advertise<auv_msgs::NED>("position",1);
				pubHeading = nh.advertise<std_msgs::Float32>("heading",1);



				file.open(fileName.c_str(), std::ios::in);

				publishLoop();
			}

			~publishData(){

			}

			void publishLoop(){

				ros::Rate rate(1/publishRate);

				while(ros::ok()){
					if(file.is_open()){

						std::string line;
						std::getline(file,line);
						parsedData = split(line,',');

						velocityData.twist.linear.x = std::atof(parsedData[u].c_str());
						velocityData.twist.linear.y = std::atof(parsedData[v].c_str());
						velocityData.twist.linear.z = std::atof(parsedData[w].c_str());
						velocityData.twist.angular.z = std::atof(parsedData[r].c_str());

						//std::pair<double,double> posxy = labust::tools::deg2meter((std::atof(parsedData[lat].c_str()) - origin.latitude)*180/M_PI, (std::atof(parsedData[lon].c_str()) - origin.longitude)*180/M_PI, std::atof(parsedData[lon].c_str())*180/M_PI);

						auv_msgs::NED position;
					    //position.north =  posxy.first;
					    //position.east = posxy.second;
					    //position.depth = std::atof(parsedData[depth].c_str());;

					    //position.north =  std::atof(parsedData[x].c_str());
					    //position.east = std::atof(parsedData[y].c_str());

					    //std_msgs::Float32 rangeData;
					    //rangeData.data = sqrt(pow(position.north,2)+pow(position.east,2)+pow(position.depth,2));

					    std_msgs::Float32 rangeData;
					    rangeData.data = std::atof(parsedData[range].c_str());


					    std_msgs::Float32 depthData;
					    depthData.data = std::atof(parsedData[depth].c_str());

					    std_msgs::Float32 headingData;
					    headingData.data = atof(parsedData[psi].c_str());


						pubVelocityRef.publish(velocityData);
						pubDepth.publish(depthData);
						pubPosition.publish(position);
						pubHeading.publish(headingData);
						if(rangeData.data > 0){
							pubRange.publish(rangeData);
						}

					}

					ros::spinOnce();
					rate.sleep();
				}
			}

			std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
				std::stringstream ss(s);
				std::string item;
				while (std::getline(ss, item, delim)) {
					if(!item.empty()){
						elems.push_back(item);
					}
				}
				return elems;
			}

			std::vector<std::string> split(const std::string &s, char delim) {
				std::vector<std::string> elems;
				split(s, delim, elems);
				return elems;
			}

			std::ifstream file;
			std::string fileName;
			double publishRate;
			std::vector<std::string> parsedData;
			auv_msgs::BodyVelocityReq velocityData;
			ros::Publisher pubVelocityRef, pubRange, pubDepth, pubPosition, pubHeading;
			auv_msgs::DecimalLatLon origin;
			int counter;
		};
	}
}


int main(int argc, char* argv[]){


	ros::init(argc,argv,"publish_from_csv");
	labust::sim::publishData();
	return 0;
}



