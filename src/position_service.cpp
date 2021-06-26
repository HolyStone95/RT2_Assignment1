#include "ros/ros.h"
#include "rt2_assignment1/RandomPosition.h"

/**The randMToN function
 *return a random number in a certain range
 *@param M the minimum of a certain range
 *@param N the maximum of a certain range 
 */
double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }

/** This function is the callback of the service /position_server 
 *@param req  the request received: it gets two intervals of values for x and y 
 *@param res  the response returned to the client: the random coordinates within a specific interval
 */
bool myrandom (rt2_assignment1::RandomPosition::Request &req, rt2_assignment1::RandomPosition::Response &res){
    res.x = randMToN(req.x_min, req.x_max);
    res.y = randMToN(req.y_min, req.y_max);
    res.theta = randMToN(-3.14, 3.14);
    return true;
}

/**The Main function
 *It simply initializes the service server, binding him to its function
 */
int main(int argc, char **argv)
{
   ros::init(argc, argv, "random_position_server");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/position_server", myrandom);
   ros::spin();

   return 0;
}
