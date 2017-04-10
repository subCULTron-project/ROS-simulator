/*********************************************************************
 * Battery simulator. It reads data published to "battery_charging" topic.
 * Current value is published to the agent's "battery_level" topic.
 * Publishes battery alert level message if battery level is below threshold 
 * and agent is on surface.
 *********************************************************************/


#include <auv_msgs/NavSts.h>
#include <auv_msgs/NED.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <misc_msgs/GetChargeInfo.h>
#include <ros/ros.h>
#include <string>

struct BatterySensorSim
{
	BatterySensorSim():
		drainingFactor(1.0),
		initialBatteryLevel(50.0),
		timerDuration(1.0)
	{
		start = false;
		sleepMode = false;

		ros::NodeHandle nh, ph("~");
		ph.getParam("draining_factor", drainingFactor);
		ph.getParam("battery_level", initialBatteryLevel);
		
		std::string ns = ros::this_node::getNamespace();

		batteryPercentage = std_msgs::Float64();
		batteryPercentage.data = initialBatteryLevel;

		// subscribers
		powerSleepMode = nh.subscribe<std_msgs::Bool>("power_sleep_mode", 1, &BatterySensorSim::onPowerSleep, this);
		chargeSub = nh.subscribe<std_msgs::Float64>("charging", 1, &BatterySensorSim::onCharge, this);
		drainSub = nh.subscribe<std_msgs::Float64>("draining", 1, &BatterySensorSim::onDrain, this); // when sending energy to another agent
		startSub = nh.subscribe<std_msgs::Bool>("start_sim", 1, &BatterySensorSim::onStart, this);
		
		// publishers
		batteryPub = nh.advertise<std_msgs::Float64>("battery_level", 1);

		// services
		getBatterySrv = nh.advertiseService("get_battery_level", &BatterySensorSim::getBatteryLevel, this);
		
		// timer
		timer = nh.createTimer(ros::Duration(timerDuration), &BatterySensorSim::timerCallback, this);
	}
	
	void timerCallback(const ros::TimerEvent& event)
	{
		if ((not start) or sleepMode)
		    return;
		
		if (batteryPercentage.data > 0) 
			batteryPercentage.data -= drainingFactor * timerDuration;

        batteryPub.publish(batteryPercentage);
	}

	void onCharge(const typename std_msgs::Float64::ConstPtr& msg)
	{
		batteryPercentage.data += msg->data;
	}

	void onDrain(const typename std_msgs::Float64::ConstPtr& msg)
	{
		batteryPercentage.data -= msg->data;
		batteryPub.publish(batteryPercentage);
	}

	void onStart(const typename std_msgs::Bool::ConstPtr& msg)
	{
		start = true;
	}

	void onPowerSleep(const typename std_msgs::Bool::ConstPtr& msg)
	{
		sleepMode = msg->data;
	}

	bool getBatteryLevel(misc_msgs::GetChargeInfo::Request &req, misc_msgs::GetChargeInfo::Response &resp)
	{
		resp.battery_level = batteryPercentage.data;
		return true;
	}

private:

	// flags
	bool start;
	bool sleepMode;

	float initialBatteryLevel;

	ros::Subscriber startSub;
	ros::Subscriber powerSleepMode;

	ros::Subscriber chargeSub;
	ros::Subscriber drainSub;
	ros::Publisher batteryPub;

	ros::ServiceServer getBatterySrv;

	// battery level range
	std_msgs::Float64 batteryPercentage;

	float drainingFactor;

	float timerDuration;
	ros::Timer timer;
};

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"battery_sensor_sim");
	ros::NodeHandle nh;
	BatterySensorSim batterySensorSim;
	ros::spin();
	return 0;
}


