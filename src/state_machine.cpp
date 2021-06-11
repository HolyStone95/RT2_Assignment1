#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/RandomPosition.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <rt2_assignment1/GoalReachingAction.h>
#include <geometry_msgs/Twist.h>
#include <iostream>

bool start = false;
bool change = false;
bool working = false;
/**The user_interface funtion
 *
 * This function is the callback function for the server
 * of /user_interface service
 *@param req  the request received from the client of the user_interface.py. 
 *@param res  the response has not been used 
 *@retval A boolean value
 */
bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res){
   /* if the user has entered 1, then the request of the command custom service  is a string, 
      initialised as "start" */
    if (req.command == "start"){
      /* the global boolean start is set to True*/
    	start = true;
    	working = true;
    }
    /* else if the user has entered 0*/  
    else {
      /* the global boolean start is set to False*/ 
    	start = false;
	change = true;
    }
    return true;
}

/**The main funtion
 *
 * This function initializes everithing that is needed
 * waits for commands and then performes the various requests
 * to services and action
 * 
 *@var service is the user_interface server 
 *@var client_rp is the client asking for a random position 
 *@var ac is the action client for reaching a goal
 */
int main(int argc, char **argv)
{
   /* Initialising the state_machine node*/
   ros::init(argc, argv, "state_machine");
   /* setting-up the node handler n*/
   ros::NodeHandle n;
   /* initialising the /user_interface service */
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   /* initialising the client for retreving the random position by means of the /position_server service */
   ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
   /* initialising a custom message of type RandomPosition */
   rt2_assignment1::RandomPosition rp;
   /* initialising a custom  message of typer GoarReaching goal */
   rt2_assignment1::GoalReachingGoal goal;
   actionlib::SimpleActionClient<rt2_assignment1::GoalReachingAction> ac("go_to_point", true);
   ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
   geometry_msgs::Twist msg;

   /* filling the custom message request fields */
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;
 
   rt2_assignment1::GoalReachingResultConstPtr result;
   int res;
   
   while(ros::ok()){
   	ros::spinOnce();
   	if (start){
   	    if (working){
                /* call for the Service random position */
   		client_rp.call(rp);
   		ROS_INFO("Waiting for action server to start.");
  		/* wait for the action server to start*/
  		ac.waitForServer(); // will wait for infinite time
      		/* initialising goal's fields with retrieved random values */
  		goal.x = rp.response.x;
  		goal.y = rp.response.y;
  		goal.theta = rp.response.theta;
  		std::cout << "\nGoing to the position: x= " << goal.x << " y= " << goal.y << " theta = " <<goal.theta << std::endl;
  		/* sending the goal to the action server */
      		ac.sendGoal(goal);
      		working = false;
  	    }
  	    else{
		result = ac.getResult();
		res =(int) result->ok;
		if (res == 1){
		    working = true;
		}
	    }	
   	}
   	else{	
   		if(change){
   	            ac.cancelGoal();
   		}
   		msg.linear.x=0;
   		msg.linear.y=0;
   		msg.angular.z=0;
   		pub.publish(msg);
   	}
   }
   return 0;
}





