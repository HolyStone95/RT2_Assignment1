#include <inttypes.h>
#include <memory>
#include <functional>
#include "rt2_assignment1/srv/random_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"


using std::placeholders::_1; // they will be replaced by the actual message during the run
using std::placeholders::_2;
using std::placeholders::_3;

namespace rt2_assignment1{

/*! Position_service Class:
* The position_service Node is implemented as a class so that it can
* be structured as a component. It will be interfaced with the ROS node
* by means of the ros1_bridge package
*/
class Position_Service : public rclcpp::Node
{
public:
	/** Initialize the random Position service. 
	* It takes as argument the callback and the three message fields
	* using the bind function, the callback is executed as soon as
	* the client makes a request.
	*/
  Position_Service(const rclcpp::NodeOptions & options)
  : Node("position_server", options)
  {
    service_ = this->create_service<rt2_assignment1::srv::RandomPosition>(
      "/position_server", std::bind(&Position_Service::handle_service, this, _1, _2, _3)); /**< callback */
  }

private:

 /**Handle_service function to handle the actual service logic
 * 
 *
 * It takes as service request some interval values for x and y coordinates and 
 * returns a set of random values as response
 *
 * @param request_header
 * @param request it contains an x/y minimum and maximum value
 * @param response it defines a random set of x,y and theta, pose of the robot
 */
  void handle_service(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<rt2_assignment1::srv::RandomPosition::Request> request,
  const std::shared_ptr<rt2_assignment1::srv::RandomPosition::Response> response) 
  {
  (void)request_header;
    //actual logic of the service
    std::cout<<"received request"<<std::endl;
    response->x = request->x_min + (rand() / ( RAND_MAX / (request->x_max-request->x_min) ) );
    response->y = request->y_min + (rand() / ( RAND_MAX / (request->y_max-request->y_min) ) );
    response->theta =  -3.14+ (rand() / ( RAND_MAX / (6.28)));
    std::cout<<"sent random"<<std::endl;
  }
  rclcpp::Service<rt2_assignment1::srv::RandomPosition>::SharedPtr service_; // service var: pointer to rclcpp service
  
  
};
}
RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::Position_Service)
