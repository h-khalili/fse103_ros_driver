#include "geometry_msgs/Vector3Stamped.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"

#include <sstream>
#include "libusb-1.1.h"
#include "libvariense_fse103.h"


class Fse103Node
{
  // Publishing topic name
  const std::string pub_topic_prefix_ = "force_sensor";

  // variense::Fse103 _force_sensor_ptr = nullptr;
  std::unique_ptr<variense::Fse103> fse103_ptr_;

  // Default parameters
  const int default_rate = 200;
  const float default_filter_bandwidth = 0;
  const bool default_init_on_start = true;
  const std::string default_serial_number = "";
  const std::string default_sensor_id = "";

  // Node parameter variables
  int rate; // Publishing rate
  float filter_bandwidth;
  // Determine whether to initialise (reset offset) when node is run
  bool init_on_start;  
  std::string serial_number;
  std::string sensor_id;

  // Node handle with the global namespace (used for publishing)
  ros::NodeHandle nh_;

  // Node handle with the node's private namespace (used with parameter server)
  ros::NodeHandle nh_priv_;

  // ROS service for initialising the sensor. This resets the sensor offset
  ros::ServiceServer initialise_service;

  // Publisher for force sensor measurements
  ros::Publisher force_pub;
  std::string pub_topic_name;

  // Measurement publishing frequency
  ros::Rate loop_rate;

  // Publishing message object
  geometry_msgs::Vector3Stamped msg;

  public:

  Fse103Node (): loop_rate(1) // ros::Rate has to be initialised
  {
    nh_priv_ = ros::NodeHandle("~");
    initialiseParameters();
    try
    {
      connectSensor();
    } 
    catch(const std::exception& e)
    {
      ROS_FATAL("%s", e.what());
      throw;
    }
    initialisePublisher();
    initialiseServices();

    ROS_INFO("%s", info_string().c_str());

    loop_rate = ros::Rate(rate);
  }

  void spinOnce()
  {
    // Publish force measurement
    std::vector<float> force;
    try{
      force = fse103_ptr_->read_force();
    }
    catch(const std::exception& e)
    {
      ROS_FATAL("%s", e.what());
      throw;
    }
    msg.header.stamp = ros::Time::now();
    msg.vector.x = force[0];
    msg.vector.y = force[1];
    msg.vector.z = force[2];

    force_pub.publish(msg);

    // Process callbacks
    ros::spinOnce();

    loop_rate.sleep();
  }

  void initialiseParameters()
  {
    // Get the ~private namespace parameters from command line or launch file
    nh_priv_.param<int>("rate", rate, default_rate);
    nh_priv_.param<float>("filter_bandwidth", filter_bandwidth, default_filter_bandwidth);
    nh_priv_.param<bool>("init_on_start", init_on_start, default_init_on_start);
    // If a string was not parsed, attempt to parse as int
    if (!nh_priv_.param<std::string>("serial_number", serial_number,
                              default_serial_number))
    {
      int x;
      if (nh_priv_.getParam("serial_number", x))
        // Cast number to string
        serial_number = std::to_string(x);
    }

    if (!nh_priv_.param<std::string>("sensor_id", sensor_id, default_sensor_id))
    {
      int x;
      if (nh_priv_.getParam("sensor_id", x))
        // Cast number to string
        sensor_id = std::to_string(x);
    }
    
    // Remove clutter from parameter server
    nh_priv_.deleteParam("rate");
    nh_priv_.deleteParam("filter_bandwidth");
    nh_priv_.deleteParam("init_on_start");
    nh_priv_.deleteParam("serial_number");
    nh_priv_.deleteParam("sensor_id");
  }

  void connectSensor()
  {
    // Attempt to connect to sensor now that we have all user parameters
    // Convert filter bandwidth (cutoff) to units of half-cycles/s 
    fse103_ptr_.reset(new variense::Fse103(serial_number, 2*filter_bandwidth/rate));

    fse103_ptr_->open();
    serial_number = fse103_ptr_->get_serial_number();
    if(init_on_start) fse103_ptr_->initialise();
      
    // Set sensor to transfer calculated value measurements
    fse103_ptr_->set_data_format(variense::Fse103::CALCULATED);
  }

  void initialisePublisher()
  {
    pub_topic_name = pub_topic_prefix_ + (sensor_id != "" ? "_" : "") + sensor_id;  
    force_pub =
      nh_.advertise<geometry_msgs::Vector3Stamped>(pub_topic_name, 10);
  }

  void initialiseServices()
  {
    initialise_service = nh_.advertiseService(ros::this_node::getName() + "/initialise", &Fse103Node::initialise, this);
  }

  bool initialise(std_srvs::Trigger::Request& req, 
                  std_srvs::Trigger::Response& res)
  {
    try
    {
      fse103_ptr_->initialise();
      res.success = 1;
      res.message = "Sensor successfully initialised.";
    }
    catch(const std::exception& e)
    {
      res.success = 0;
      res.message = e.what() + '\n';
    }
    return res.success;
  }

  std::string info_string ()
  {
    std::ostringstream o;
    o << "Variense FSE103 force sensor driver::" << std::endl
      << "- Serial number: " << serial_number << std::endl
      << "- Publishing rate: " << rate << "Hz" << std::endl
      << "- Filter bandwidth: ";
    if(filter_bandwidth > 0)
      o << filter_bandwidth << "Hz";
    else 
      o << "N/A";
    o << std::endl
      << "- Initialisation: " << (init_on_start? "true" : "false") << std::endl
      << "- ROS topic: " << pub_topic_name << std::endl
      << "- Node name: " << ros::this_node::getName();

    return o.str();
  }

}; // class Fse103Node

int main(int argc, char **argv) {

  // read commnad line arguements and node name
  ros::init(argc, argv, "force_sensor_node");

  try
  {
    Fse103Node fse103_node;
    
    int count = 0;
    // ROS_INFO("Publishing force sensor measurements ... ");
    while (ros::ok()) 
    {
      fse103_node.spinOnce();
      ++count;
    }
  }
  catch(const std::exception& e)
  {
    std::cerr << "Exception caught!" << std::endl;
  }

  return 0;
}

