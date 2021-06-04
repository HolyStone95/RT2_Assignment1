#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/Position.h"
#include "rt2_assignment1/RandomPosition.h"

bool start = false;

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
    }
    /* else if the user has entered 0*/  
    else {
      /* the global boolean start is set to False*/     
    	start = false;
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
 *@var client_p is the service client for reaching a goal
 */
int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
   ros::ServiceClient client_p = n.serviceClient<rt2_assignment1::Position>("/go_to_point");
   
   rt2_assignment1::RandomPosition rp;
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;
   rt2_assignment1::Position p;
   
   while(ros::ok()){
   	ros::spinOnce();
   	/*request a goal pose, use the response as request for go_to_point service*/
   	if (start){
   		client_rp.call(rp);
   		p.request.x = rp.response.x;
   		p.request.y = rp.response.y;
   		p.request.theta = rp.response.theta;
   		std::cout << "\nGoing to the position: x= " << p.request.x << " y= " <<p.request.y << " theta = " <<p.request.theta << std::endl;
   		client_p.call(p);
   		std::cout << "Position reached" << std::endl;
   	}
   }
   return 0;
}
