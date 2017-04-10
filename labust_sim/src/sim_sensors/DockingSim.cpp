/*********************************************************************
 * Docking simulator. Counts number of docked agents.
 *********************************************************************/

#include <misc_msgs/GetDockingInfo.h>
#include <misc_msgs/ChangeDockingStatus.h>

#include <ros/ros.h>

struct DockingSim
{
	DockingSim():
		maxDocked(4) // default value
	{
		ros::NodeHandle nh, ph("~");
		ph.getParam("maximum_docked", maxDocked);

		occupied = std::vector<int>();
		available = std::vector<int>();
		for(int i = 0; i < maxDocked; i++ )
    		available.push_back(i);

		// services
		checkAvailabilitySrv = nh.advertiseService("check_docking_availability", &DockingSim::checkDockingAvailability, this);
		changeDockingStatusSrv = nh.advertiseService("change_docking_status", &DockingSim::changeDockingStatus, this);		
	}

	bool changeDockingStatus(misc_msgs::ChangeDockingStatus::Request &req, misc_msgs::ChangeDockingStatus::Response &resp)
	{
		if (req.status) // request for docking
		{
			if (std::find(occupied.begin(), occupied.end(), req.slot) == occupied.end())
			{
				//slots[req.slot] = "--"; // assign the node to slot
				occupied.push_back(req.slot);
				available.erase(std::remove(available.begin(), available.end(), req.slot), available.end());
				return true;
			}
			return false;
		}
		else
		{
			if(std::find(occupied.begin(), occupied.end(), req.slot) != occupied.end()) 
			{
				occupied.erase(std::remove(occupied.begin(), occupied.end(), req.slot), occupied.end());
				available.push_back(req.slot);
				return true;
			}
			return false;
		}
	}

	bool checkDockingAvailability(misc_msgs::GetDockingInfo::Request &req, misc_msgs::GetDockingInfo::Response &resp)
	{
		resp.available_slots = available;
		return true;
	}

private:

	ros::Subscriber newDockedSub;

	ros::ServiceServer checkAvailabilitySrv;
	ros::ServiceServer changeDockingStatusSrv;

	std::vector<int> occupied, available;

	int maxDocked;
};

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"docking_sim");
	ros::NodeHandle nh;
	DockingSim dockingSim;
	ros::spin();
	return 0;
}