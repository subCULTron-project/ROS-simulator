/*********************************************************************
 * Current simulator. It reads data published to "current_sensor" topic.
 * Depending on agent's position, current value is being published to
 * "currents" topic (if agent position's depth component is smaller 
 * than param current_depth).
 *********************************************************************/

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Point.h>
#include <auv_msgs/NavSts.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <ros/ros.h>

struct CurrentSensorSim
{
	CurrentSensorSim():
		currentDepth(0.5) // default value
	{
		start = false;
		anchored = false;
		ros::NodeHandle nh, ph("~");
		ph.getParam("current_depth", currentDepth);

		currentPublished = false;
		gotPosition = false;

		position = geometry_msgs::Point();
		current = geometry_msgs::TwistStamped();

		// subscribers
		positionSub = nh.subscribe<auv_msgs::NavSts>("position", 1, &CurrentSensorSim::onPosition, this);
		currentSensorSub = nh.subscribe<geometry_msgs::TwistStamped>("current_sensor", 1, &CurrentSensorSim::onCurrent, this);
		// publishers
		currentPub = nh.advertise<geometry_msgs::TwistStamped>("currents", 1);
		
		currPubTimeout = ros::Time::now() + ros::Duration(0.2);
		startSub = nh.subscribe<std_msgs::Bool>("start_sim", 1, &CurrentSensorSim::onStart, this);

		anchorSrv = nh.advertiseService("anchor", &CurrentSensorSim::changeAnchoredStatus, this);	
	}

	void onStart(const typename std_msgs::Bool::ConstPtr& msg)
	{
		start = true;
	}

	void onPosition(const typename auv_msgs::NavSts::ConstPtr& msg)
	{
		// set position
		position.x = msg->position.north;
		position.y = msg->position.east;
		position.z = msg->position.depth;
		gotPosition = true;
	}

	void onCurrent(const typename geometry_msgs::TwistStamped::ConstPtr& msg)
	{
		if ((not start) || (not gotPosition))
		    return;
		
		// get new current values from message
		geometry_msgs::TwistStamped newCurrent = geometry_msgs::TwistStamped();
		newCurrent.twist.linear.x = msg->twist.linear.x;
	    newCurrent.twist.linear.y = msg->twist.linear.y;
	    newCurrent.twist.linear.z = msg->twist.linear.z;

		// publish only when current changes or current was not published in some time
		if ((newCurrent.twist.linear.x != current.twist.linear.x) || (newCurrent.twist.linear.y != current.twist.linear.y) 
			|| (newCurrent.twist.linear.z != current.twist.linear.z) || (not currentPublished) || (currPubTimeout < ros::Time::now()))
		{
		    current = newCurrent;

  		    // check depth and set current accordingly
		    if (position.z <= currentDepth and not anchored) 
		    {
		    	currentPub.publish(newCurrent);
		    }
		    else
		    {
		    	// if agent is out of current range, set current value to zero
			    newCurrent.twist.linear.x = 0;
		    	newCurrent.twist.linear.y = 0;
		    	newCurrent.twist.linear.z = 0;
			    currentPub.publish(newCurrent);
		    }

		    currPubTimeout = ros::Time::now() + ros::Duration(0.2);
		    currentPublished = true;
		}
	}

	bool changeAnchoredStatus(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp)
	{
		anchored = not anchored;
		return true;
	}

private:
	// flags
	bool start;
	bool gotPosition;
	bool currentPublished;
	bool anchored;

	ros::Subscriber startSub;
	ros::Subscriber positionSub;
	ros::Subscriber currentSensorSub;

	ros::ServiceServer anchorSrv;
	
	// current value affecting agent
	ros::Publisher currentPub;

	// cuurent depth range
	double currentDepth;
	ros::Time currPubTimeout;

	geometry_msgs::Point position;
	geometry_msgs::TwistStamped current;
};

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"current_sensor_sim");
	ros::NodeHandle nh;
	CurrentSensorSim currentSensorSim;
	ros::spin();
	return 0;
}


