#include "geometry_msgs/Vector3Stamped.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"

#include <sstream>
#include "libusb-1.1.h"
#include "libvariense_fse103.h"



variense::Fse103 *force_sensor_ptr = nullptr;

void shutdown_cleanup(const ros::NodeHandle &nh)
{
  // Remove clutter from parameter server
  nh.deleteParam("rate");
  nh.deleteParam("filter_bandwidth");
  nh.deleteParam("serial_number");
  nh.deleteParam("sensor_id");
  ROS_INFO("Shutting down node ... ");
}

bool initialise(std_srvs::Trigger::Request& req, 
                std_srvs::Trigger::Response& res)
{
  int r;
  try
  {
    force_sensor_ptr->initialise();
    res.success = 1;
    res.message = "Sensor successfully initialised.";
  }
  catch(const std::exception& e)
  {
    res.success = 1;
    res.message = e.what() + '\n';
  }
  return r;
}

int main(int argc, char **argv) {
  // Default parameters
  const int default_rate = 200;
  const float default_filter_bandwidth = 0;
  const std::string default_serial_number = "";
  const std::string default_sensor_id = "";

  // Node parameter variables
  int rate; // Publishing rate
  float filter_bandwidth; 
  std::string serial_number;
  std::string sensor_id;

  // read commnad line arguements and node name
  ros::init(argc, argv, "force_sensor_cpp");

  // Node handle with the global namespace (used for publishing)
  ros::NodeHandle n;

  // Node handle with the node's private namespace (used with parameter server)
  ros::NodeHandle nh("~");
  // Get the ~private namespace parameters from command line or launch file
  nh.param<int>("rate", rate, default_rate);
  nh.param<float>("filter_bandwidth", filter_bandwidth, default_filter_bandwidth);
  // If a string was not parsed, attempt to parse as int
  if (!nh.param<std::string>("serial_number", serial_number,
                             default_serial_number))
  {
    int x;
    if (nh.getParam("serial_number", x))
      // Cast number to string
      serial_number = std::to_string(x);
  }

  std::cout << serial_number << std::endl;

  if (!nh.param<std::string>("sensor_id", sensor_id, default_sensor_id))
  {
    int x;
    if (nh.getParam("sensor_id", x))
      // Cast number to string
      sensor_id = std::to_string(x);
  }
  // nh.param<std::string>("sensor_id", sensor_id , default_sensor_id);


  // Attempt to connect to sensor now that we have all user parameters
  // Convert filter bandwidth (cutoff) to units of half-cycles/s 
  // variense::Fse103 force_sensor("103EAA8876", 2*filter_bandwidth/rate);
  variense::Fse103 force_sensor(serial_number, 2*filter_bandwidth/rate);
  try
  {
      force_sensor.open();
  }
  catch(const std::exception& e)
  {
      std::cout << "Error: " << e.what() << std::endl;
      shutdown_cleanup(nh);
      return 1;
  }

  // Set sensor to transfer calculated value measurements
  force_sensor.set_data_format(variense::Fse103::CALCULATED);

  // Initialise the sensor pointer for the "initialise" service
  force_sensor_ptr = &force_sensor;

  ros::ServiceServer initialise_service = n.advertiseService("initialise", initialise);

  ros::Publisher force_pub =
      n.advertise<geometry_msgs::Vector3Stamped>("force_sensor_" + sensor_id, 10);

  ROS_INFO("The node rate is %dHz", rate);
  ROS_INFO("The filter bandwidth is %.1fHz", filter_bandwidth);
  ROS_INFO("The serial number is %s", force_sensor.get_serial_number().c_str());
  ROS_INFO("The sensor id is %s", sensor_id.c_str());

  ros::Rate loop_rate(rate);

  geometry_msgs::Vector3Stamped msg;
  int count = 0;
  ROS_INFO("Publishing force sensor measurements ... ");
  while (ros::ok()) 
  {
    const std::vector<float> force = force_sensor.read_force();
    msg.header.stamp = ros::Time::now();
    msg.vector.x = force[0];
    msg.vector.y = force[1];
    msg.vector.z = force[2];

    // printf("x: %.1f, y: %.1f, z: %.1f\n", force[0], force[1], force[2]);

    // if(count == 1000)
    //   force_sensor.initialise();

    force_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  
  shutdown_cleanup(nh);

  return 0;
}

