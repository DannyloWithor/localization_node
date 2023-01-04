#include "rclcpp/rclcpp.hpp"
#include "moving_average.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "localization_interfaces/msg/hedge_quality.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/visibility_control.h>
#include <tf2/buffer_core.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <chrono>

using namespace std::chrono_literals;

#define ROS_NODE_NAME "localization_node"
#define PI 3.1415

class LocalizationNode : public rclcpp::Node
{
  public:
    LocalizationNode()
    : Node(ROS_NODE_NAME)
    {
      auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

      // ************** Declare publishers for rviz visualization **************
      pubMarvelQuality = this->create_publisher<std_msgs::msg::Float64>("marvelmind_quality", 1);
			pubIidreQuality = this->create_publisher<std_msgs::msg::Int32MultiArray>("iidre_quality", 1);
      //pubOdom = this->create_publisher<nav_msgs::msg::Odometry>("odom_with_covariance", 1);
      pubIidreOdom = this->create_publisher<nav_msgs::msg::Odometry>("iidre_with_covariance", 1);
      pubMarvel1Odom = this->create_publisher<nav_msgs::msg::Odometry>("marvel1_with_covariance", 1);
      pubMarvel2Odom = this->create_publisher<nav_msgs::msg::Odometry>("marvel2_with_covariance", 1);
      pubImu = this->create_publisher<sensor_msgs::msg::Imu>("imu", 1);
      pubScanFront = this->create_publisher<sensor_msgs::msg::LaserScan>("scan_front", 1);
      pubScanBack = this->create_publisher<sensor_msgs::msg::LaserScan>("scan_back", 1);
      pubTf = this->create_publisher<tf2_msgs::msg::TFMessage>("tf", 1);
      //pubQualBeacon1 = this->create_publisher<std_msgs::msg::Float64>("iidre_qual_1", 1);
      //pubQualBeacon2 = this->create_publisher<std_msgs::msg::Float64>("iidre_qual_2", 1);
      //pubQualBeacon3 = this->create_publisher<std_msgs::msg::Float64>("iidre_qual_3", 1);
      //pubQualBeacon4 = this->create_publisher<std_msgs::msg::Float64>("iidre_qual_4", 1);
      pubIidreHeading = this->create_publisher<std_msgs::msg::Float64>("iidre_heading", 1);
      pubImuHeading = this->create_publisher<std_msgs::msg::Float64>("imu_heading", 1);

      // ************** Declare subscribers **************
      subMarvelmind1Quality = this->create_subscription
        <localization_interfaces::msg::HedgeQuality>(
        "marvel1/hedge_quality", 10, std::bind(&LocalizationNode::marvelmind1QualityCallback, this, std::placeholders::_1)); 

      subMarvelmind2Quality = this->create_subscription
        <localization_interfaces::msg::HedgeQuality>(
        "marvel2/hedge_quality", 10, std::bind(&LocalizationNode::marvelmind2QualityCallback, this, std::placeholders::_1)); 
      
      subSlamOdom = this->create_subscription
        <nav_msgs::msg::Odometry>(
        "/slam_odom_global", default_qos, std::bind(&LocalizationNode::slamOdomCallback, this, std::placeholders::_1));
      
      subIidreOdom = this->create_subscription
        <nav_msgs::msg::Odometry>(
        "/iidre_pose", 10, std::bind(&LocalizationNode::odomIidreCallback, this, std::placeholders::_1));
      
      subMarvel1Odom = this->create_subscription
        <nav_msgs::msg::Odometry>(
        "marvel2/hedge_pos", 10, std::bind(&LocalizationNode::odomMarvel1Callback, this, std::placeholders::_1));

      subMarvel2Odom = this->create_subscription
        <nav_msgs::msg::Odometry>(
        "marvel1/hedge_pos", 10, std::bind(&LocalizationNode::odomMarvel2Callback, this, std::placeholders::_1));
    
      subImu = this->create_subscription
      <sensor_msgs::msg::Imu>(
      "old_imu", 10, std::bind(&LocalizationNode::imuCallback, this, std::placeholders::_1));

      subScanBack = this->create_subscription
      <sensor_msgs::msg::LaserScan>(
      "old_scan_back", 10, std::bind(&LocalizationNode::scanBackCallback, this, std::placeholders::_1));

      subScanFront = this->create_subscription
      <sensor_msgs::msg::LaserScan>(
      "old_scan_front", 10, std::bind(&LocalizationNode::scanFrontCallback, this, std::placeholders::_1));
      
      //subTf = this->create_subscription
      //<tf2_msgs::msg::TFMessage>(
      //"tf", 10, std::bind(&LocalizationNode::tfCallback, this, std::placeholders::_1));

      //subStaticTf = this->create_subscription
      //<tf2_msgs::msg::TFMessage>(
      //"tf_static", 10, std::bind(&LocalizationNode::staticTfCallback, this, std::placeholders::_1));
      
      //subRvizMarker = this->create_subscription
			//<visualization_msgs::msg::MarkerArray>(
			//"slam_toolbox/graph_visualization", default_qos, std::bind(&LocalizationNode::markerCallback, this, std::placeholders::_1));

      ///trajectory_node_list
      ///slam_toolbox/graph_visualization

      // ************** Set default values for sensors and covariances **************
      setDefault();

      tf_buffer_ =
        std::make_unique<tf2_ros::Buffer>(this->get_clock());
      transform_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    }

  private:
    //  ************** DATA TYPES **************
    
    struct axis
    {
      float x = 0;
      float y = 0;
      float z = 0;
    };

    struct averagedAxis
    {
      MovingAverage *x;
      MovingAverage *y;
      MovingAverage *z;
    };
    
    struct imuError
    {
      averagedAxis accelerometer;
      averagedAxis gyro; 
    };

    struct beacon
    {
      geometry_msgs::msg::Point bodyPose;
      axis spotsOffset[8];
      float correctionThreshold;
    };

    //  ************** PUBLISHERS AND SUBSCRIBERS **************
    // --- Publishers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubMarvelQuality;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubIidreHeading;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubImuHeading;
		
		rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pubIidreQuality;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdom;

    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovariance>::SharedPtr pubTwist;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubIidreOdom;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubMarvel1Odom;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubMarvel2Odom;

    //rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubSlamOdom;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pubImu;

    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pubTf;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubQualBeacon1;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubQualBeacon2;
    
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubQualBeacon3;
    
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubQualBeacon4;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pubScanFront;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pubScanBack;

    // --- Subscribers
    rclcpp::Subscription<localization_interfaces::msg::HedgeQuality>::SharedPtr subMarvelmind1Quality;

    rclcpp::Subscription<localization_interfaces::msg::HedgeQuality>::SharedPtr subMarvelmind2Quality;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subSlamOdom;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subIidreOdom;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subMarvel1Odom;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subMarvel2Odom;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;

    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subTf;

    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subStaticTf;

    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr subRvizMarker;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subScanBack;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subScanFront;


    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;


  //  ************** AUX VARIABLES **************
    imuError imuOffset;
    float odomOffset[3] = {-0.7075, 1.0525, 3.1415};

    int imuCount;

    //float imuFrequency = 100; // [Hz]

    bool is_moving = false;

    builtin_interfaces::msg::Time imuLastMeas;

    nav_msgs::msg::Odometry calcOdom;
    nav_msgs::msg::Odometry calcMarvel;
    nav_msgs::msg::Odometry calcIdre;

    sensor_msgs::msg::Imu calcImu;

    beacon marvelmindBeacon1;
    beacon marvelmindBeacon2;
    beacon iidreBeacon1;
    
    axis spots[7];

    float ROBOT_X_SIZE = 1.415;
    float ROBOT_Y_SIZE = 0.855; 

    float headingImu = 0.0;
    float headingOdom = 0.0;
    float headingSlam = 0.0;
    float headingIidre = 0.0;
    float headingMarvel1 = 0.0;
    float headingMarvel2 = 0.0;

    float marvel1Quality = 0.0;
    float marvel2Quality = 0.0;

    axis iidreLastPosition;
    axis marvel1LastPosition;
    axis marvel2LastPosition;
    axis slamPose;
    axis slamOrientation;

    averagedAxis iidreBuffer;
    averagedAxis marvel1Buffer;
    averagedAxis marvel2Buffer;

  //  ************** AUX METHODS **************

  void setDefault()
  {
    //  ************** Set sensors default covariances **************
      // --- IMU
      calcImu.angular_velocity_covariance[0] = 0.3;
      calcImu.angular_velocity_covariance[4] = 0.3;
      calcImu.angular_velocity_covariance[8] = 0.1;

      calcImu.linear_acceleration_covariance[0] = 1.7;
      calcImu.linear_acceleration_covariance[4] = 1.7;
      calcImu.linear_acceleration_covariance[8] = 1.7;

      calcImu.orientation_covariance[0] = 0.7;
      calcImu.orientation_covariance[4] = 0.7;
      calcImu.orientation_covariance[8] = 0.7;

      // --- Odometry
      calcOdom.pose.covariance[0] = 1.0;
      calcOdom.pose.covariance[7] = 1.0;
      calcOdom.pose.covariance[14] = 1.0;
      calcOdom.pose.covariance[21] = 0.1;
      calcOdom.pose.covariance[28] = 0.01;
      calcOdom.pose.covariance[35] = 0.01;

      calcOdom.twist.covariance[0] = 0.1;
      calcOdom.twist.covariance[7] = 0.1;
      calcOdom.twist.covariance[14] = 0.1;
      calcOdom.twist.covariance[21] = 0.1;
      calcOdom.twist.covariance[28] = 0.1;
      calcOdom.twist.covariance[35] = 0.1;

      // --- IIDRE
      calcIdre.pose.covariance[0] = 10.0;
      calcIdre.pose.covariance[7] = 10.0;
      calcIdre.pose.covariance[14] = 1.0;
      calcIdre.pose.covariance[21] = 0.7;
      calcIdre.pose.covariance[28] = 0.7;
      calcIdre.pose.covariance[35] = 0.7;

      calcIdre.twist.covariance[0] = 1000;
      calcIdre.twist.covariance[7] = 1000;
      calcIdre.twist.covariance[14] = 1.0;
      calcIdre.twist.covariance[21] = 1.0;
      calcIdre.twist.covariance[28] = 1.0;
      calcIdre.twist.covariance[35] = 100;

      // --- Marvel
      calcMarvel.pose.covariance[0] = 0.01;
      calcMarvel.pose.covariance[7] = 0.01;
      calcMarvel.pose.covariance[14] = 1.0;
      calcMarvel.pose.covariance[21] = 0.7;
      calcMarvel.pose.covariance[28] = 0.7;
      calcMarvel.pose.covariance[35] = 0.7;

      calcMarvel.twist.covariance[0] = 1000;
      calcMarvel.twist.covariance[7] = 1000;
      calcMarvel.twist.covariance[14] = 1;
      calcMarvel.twist.covariance[21] = 1;
      calcMarvel.twist.covariance[28] = 1;
      calcMarvel.twist.covariance[35] = 100;

      imuCount = 0;

      // Set the size for the imu offset average array
      imuOffset.accelerometer.x = new MovingAverage(5);
      imuOffset.accelerometer.y = new MovingAverage(5);
      imuOffset.accelerometer.z = new MovingAverage(5);

      imuOffset.gyro.x = new MovingAverage(5);
      imuOffset.gyro.y = new MovingAverage(5);
      imuOffset.gyro.z = new MovingAverage(5);

      iidreBuffer.x = new MovingAverage(20);
      iidreBuffer.y = new MovingAverage(20);
      iidreBuffer.z = new MovingAverage(20);

      marvel1Buffer.x = new MovingAverage(1);
      marvel1Buffer.y = new MovingAverage(1);
      marvel1Buffer.z = new MovingAverage(1);

      marvel2Buffer.x = new MovingAverage(1);
      marvel2Buffer.y = new MovingAverage(1);
      marvel2Buffer.z = new MovingAverage(1);

      iidreLastPosition.x = 0.0;
      iidreLastPosition.y = 0.0;

      marvel1LastPosition.x = 0.0;
      marvel1LastPosition.y = 0.0;

      marvel2LastPosition.x = 0.0;
      marvel2LastPosition.y = 0.0;

      marvelmindBeacon1.correctionThreshold = 1.0;

      marvelmindBeacon1.spotsOffset[0].x = -0.0975;   marvelmindBeacon1.spotsOffset[0].y = 0.1040;
      marvelmindBeacon1.spotsOffset[1].x = -0.1315;   marvelmindBeacon1.spotsOffset[1].y = 0.1625;
      marvelmindBeacon1.spotsOffset[2].x = 0;         marvelmindBeacon1.spotsOffset[2].y = 0;
      marvelmindBeacon1.spotsOffset[3].x = 0;         marvelmindBeacon1.spotsOffset[3].y = 0;
      marvelmindBeacon1.spotsOffset[4].x = 0.053;     marvelmindBeacon1.spotsOffset[4].y = -0.0375;
      marvelmindBeacon1.spotsOffset[7].x = -0.21;     marvelmindBeacon1.spotsOffset[7].y = 0.0015;

      marvelmindBeacon2.correctionThreshold = 1.0;

      marvelmindBeacon2.spotsOffset[0].x = -0.0155;   marvelmindBeacon2.spotsOffset[0].y = 0.1515;
      marvelmindBeacon2.spotsOffset[1].x = -0.0415;   marvelmindBeacon2.spotsOffset[1].y = 0.1235;
      marvelmindBeacon2.spotsOffset[2].x = -0.0770;   marvelmindBeacon2.spotsOffset[2].y = -0.095;
      marvelmindBeacon2.spotsOffset[3].x = 0;         marvelmindBeacon2.spotsOffset[3].y = 0;
      marvelmindBeacon2.spotsOffset[4].x = 0.152;     marvelmindBeacon2.spotsOffset[4].y = -0.0825;
      marvelmindBeacon2.spotsOffset[7].x = 0.1115;    marvelmindBeacon2.spotsOffset[7].y = 0.0740;

      spots[0].x = 0.0;		    spots[0].y = 1.5;	
      spots[1].x = -2.0;		  spots[1].y = 1.5;		
      spots[2].x = 2.50;		  spots[2].y = 2.5;	
      spots[3].x = 4.00;		  spots[3].y = 6.0;	
      spots[4].x = 5.00;		  spots[4].y = 0.0;		
      spots[5].x = 12.00;		  spots[5].y = 4.7;	
      spots[6].x = 18.00;		  spots[6].y = 0.4;	
      spots[7].x = 9.00;		  spots[7].y = 2.00;	
  }

  float orientationFromMarvel()
  {
    float slope_between_sensors = (marvel2Buffer.y->mean() - marvel1Buffer.y->mean())/(marvel2Buffer.x->mean() - marvel1Buffer.x->mean());
    float slope_offset = (marvelmindBeacon2.bodyPose.y  - marvelmindBeacon1.bodyPose.y )/(marvelmindBeacon2.bodyPose.x  - marvelmindBeacon1.bodyPose.x);
    float position_offset = 0.0;

    if((marvel1Buffer.y->mean() > marvel2Buffer.y->mean())||(marvel1Buffer.x->mean() > marvel2Buffer.x->mean()))
      position_offset -= PI;

    return normalizeAngle(atan(slope_between_sensors - slope_offset));
  }
  
  float normalizeAngle(float angle)
  {
    if(angle > PI)
      angle -= 2*PI;
    else if(angle < -PI)
      angle += 2*PI;

    return angle;
  }
  
  bool distanceValidity(int sensor)
  {
    float marvel_1_to_2 = sqrt((marvel1Buffer.x->mean() - marvel2Buffer.x->mean()) * (marvel1Buffer.x->mean() - marvel2Buffer.x->mean()) 
                          + (marvel1Buffer.y->mean() - marvel2Buffer.y->mean()) * (marvel1Buffer.y->mean() - marvel2Buffer.y->mean()));
    
    float marvel_1_to_iidre = sqrt((marvel1Buffer.x->mean() - iidreBuffer.x->mean()) * (marvel1Buffer.x->mean() - iidreBuffer.x->mean()) 
                              + (marvel1Buffer.y->mean() - iidreBuffer.y->mean()) * (marvel1Buffer.y->mean() - iidreBuffer.y->mean()));

    float marvel_2_to_iidre = sqrt((marvel2Buffer.x->mean() - iidreBuffer.x->mean()) * (marvel2Buffer.x->mean() - iidreBuffer.x->mean()) 
                              + (marvel2Buffer.y->mean() - iidreBuffer.y->mean()) * (marvel2Buffer.y->mean() - iidreBuffer.y->mean()));

    //RCLCPP_INFO(this->get_logger(), "%f %f %f", marvel_1_to_2, marvel_1_to_iidre, marvel_2_to_iidre);
    // if marvel 1
    if((sensor == 1) && ((fabs(marvel_1_to_2) > 1.65) || (fabs(marvel_1_to_iidre) > 1.3)))
      return false;

    // if marvel 2
    if((sensor == 2) && ((fabs(marvel_1_to_2) > 1.65) || (fabs(marvel_2_to_iidre) > 1.7)))
      return false;

    // if iidre
    if((sensor == 3) && ((fabs(marvel_1_to_iidre) > 2.0) || (fabs(marvel_2_to_iidre) > 2.0)))
      return false;

    return true;
  }

  void checkNearestSpotsOffsets(beacon _beacon, float pose[2])
  {
    float distance = 0.0;
    axis dummy;
    //RCLCPP_INFO(this->get_logger(), "	Entrou");

    for(int i = 0; i < 7; i++)
    {
      distance = sqrt((pose[0]-this->spots[i].x)*(pose[0]-this->spots[i].x) + (pose[1]-this->spots[i].y)*(pose[1]-this->spots[i].y));
      //RCLCPP_INFO(this->get_logger(), "	Distance: %f", distance);
      //RCLCPP_INFO(this->get_logger(), "	Thresh: %f", _beacon.correctionThreshold);
      if(distance < _beacon.correctionThreshold)
      {
        //RCLCPP_INFO(this->get_logger(), "	Entrou2");
        //RCLCPP_INFO(this->get_logger(), "	Achou a vaga %d", i);
        pose[0] -= _beacon.spotsOffset[i].x;
        pose[1] -= _beacon.spotsOffset[i].y; 
      }
    }
  }

  //  ************** CALLBACKS **************

  void markerCallback( const visualization_msgs::msg::MarkerArray::SharedPtr marker)
	{
		//RCLCPP_INFO(this->get_logger(), "Receiving Markers");
		std::vector<visualization_msgs::msg::Marker> marker_msg = marker->markers;

		// Build odom message to be republished
		nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "map";
    odom.child_frame_id = "Base_Link";
    odom.header.stamp = marker_msg[0].header.stamp;
		odom.pose.pose = marker_msg[0].pose;

		// Inform covariances
		odom.pose.covariance[0] = 0.08;
		odom.pose.covariance[7] = 0.08;
		odom.pose.covariance[14] = 1.0;
		odom.pose.covariance[21] = 0.3;
		odom.pose.covariance[28] = 0.3;
		odom.pose.covariance[35] = 0.3;

		odom.twist.covariance[0] = 5.0;
		odom.twist.covariance[7] = 1.0;
		odom.twist.covariance[14] = 1.0;
		odom.twist.covariance[21] = 1.0;
		odom.twist.covariance[28] = 1.0;
		odom.twist.covariance[35] = 1.0;

		// Publish odom message
    //this->pubSlamOdom->publish(odom);
	}

  void cartMarkerCallback( const visualization_msgs::msg::MarkerArray::SharedPtr marker)
	{
		//RCLCPP_INFO(this->get_logger(), "Receiving Markers");
		std::vector<visualization_msgs::msg::Marker> marker_msg = marker->markers;
    std::vector<geometry_msgs::msg::Point> points = marker_msg[1].points;
    tf2::Quaternion q;

		// Build odom message to be republished
		nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "map_slam";
    odom.child_frame_id = "odom";
    odom.header.stamp = marker_msg[0].header.stamp;
		odom.pose.pose.position.x = slamPose.x;
    odom.pose.pose.position.y = slamPose.y;

    //q.setRPY(0.0, 0.0, slamOrientation.z);
    
    //odom.pose.pose.orientation.x = q.x();
    //odom.pose.pose.orientation.y = q.y();
    //odom.pose.pose.orientation.z = q.z();
    //odom.pose.pose.orientation.w = q.w();

		// Inform covariances
		odom.pose.covariance[0] = 0.5;
		odom.pose.covariance[7] = 0.5;
		odom.pose.covariance[14] = 1.0;
		odom.pose.covariance[21] = 0.3;
		odom.pose.covariance[28] = 0.3;
		odom.pose.covariance[35] = 0.3;

		odom.twist.covariance[0] = 5.0;
		odom.twist.covariance[7] = 1.0;
		odom.twist.covariance[14] = 1.0;
		odom.twist.covariance[21] = 1.0;
		odom.twist.covariance[28] = 1.0;
		odom.twist.covariance[35] = 1.0;

		// Publish odom message
    //this->pubSlamOdom->publish(odom);
	}

  void publishOdomSlam_old()
  {
    //RCLCPP_INFO(this->get_logger(), "Here! 1");
		// Build odom message to be republished
		nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "map_slam";
    odom.child_frame_id = "Base_Link";

		odom.pose.pose.position.x = slamPose.x + calcOdom.pose.pose.position.x;
    odom.pose.pose.position.y = slamPose.y + calcOdom.pose.pose.position.y;

    //RCLCPP_INFO(this->get_logger(), "Heading Slam %f  Heading Odom %f", slamOrientation.z, headingOdom);
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, (slamOrientation.z + PI) + headingOdom);
    
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

		// Inform covariances
		odom.pose.covariance[0] = 0.5;
		odom.pose.covariance[7] = 0.5;
		odom.pose.covariance[14] = 1.0;
		odom.pose.covariance[21] = 0.3;
		odom.pose.covariance[28] = 0.3;
		odom.pose.covariance[35] = 0.3;

		odom.twist.covariance[0] = 5.0;
		odom.twist.covariance[7] = 1.0;
		odom.twist.covariance[14] = 1.0;
		odom.twist.covariance[21] = 1.0;
		odom.twist.covariance[28] = 1.0;
		odom.twist.covariance[35] = 1.0;

		// Publish odom message
    //this->pubSlamOdom->publish(odom);
  }

  void publishOdomSlam()
  {
    geometry_msgs::msg::PoseStamped message;
    /*
    try {
        geometry_msgs::msg::TransformStamped transformStamped = tf_buffer_->lookupTransform("odom", "map_slam", tf2::TimePointZero);
        message.pose.position.x = transformStamped.transform.translation.x;
        message.pose.position.y = transformStamped.transform.translation.y;
        //RCLCPP_INFO(this->get_logger(), "[ODOM_TO_MAP_SLAM]: %f %f", message.pose.position.x, message.pose.position.y);
        //publisher->publish(message);
    }
    catch (tf2::TransformException & ex) {
          RCLCPP_ERROR(this->get_logger(), "StaticLayer: %s", ex.what());
    }

    try {
        geometry_msgs::msg::TransformStamped transformStamped_odom = tf_buffer_->lookupTransform("odom", "Base_Link", tf2::TimePointZero);
        message.pose.position.x = transformStamped_odom.transform.translation.x;
        message.pose.position.y = transformStamped_odom.transform.translation.y;
        //RCLCPP_INFO(this->get_logger(), "[BASE_LINK_TO_ODOM]: %f %f", message.pose.position.x, message.pose.position.y);
        //publisher->publish(message);
    }
    catch (tf2::TransformException & ex) {
          RCLCPP_ERROR(this->get_logger(), "StaticLayer: %s", ex.what());
    }*/

    try {
        geometry_msgs::msg::TransformStamped transformStamped = tf_buffer_->lookupTransform("odom", "map", tf2::TimePointZero);
        geometry_msgs::msg::TransformStamped transformStamped_odom = tf_buffer_->lookupTransform("odom", "Base_Link", tf2::TimePointZero);
       // geometry_msgs::msg::TransformStamped transformStamped_map_map_slam = tf_buffer_->lookupTransform("map", "map", tf2::TimePointZero);


        tf2::Stamped<tf2::Transform> stamped_transform_odom_to_map;
        tf2::fromMsg(transformStamped, stamped_transform_odom_to_map);

        tf2::Stamped<tf2::Transform> stamped_transform_odom_to_base;
        tf2::fromMsg(transformStamped_odom, stamped_transform_odom_to_base);

        //tf2::Stamped<tf2::Transform> stamped_transform_map_to_map_slam;
        //tf2::fromMsg(transformStamped_map_map_slam, stamped_transform_map_to_map_slam);
          
        tf2::Transform base_to_map_slam;
        base_to_map_slam.mult(stamped_transform_odom_to_map.inverse(), stamped_transform_odom_to_base);
  /* base_to_map_slam.mult(stamped_transform_map_to_map_slam, base_to_map_slam);
*/
        geometry_msgs::msg::Transform base_to_map_slam_stamped;
        base_to_map_slam_stamped = tf2::toMsg(base_to_map_slam);

        message.pose.position.x = base_to_map_slam_stamped.translation.x;
        message.pose.position.y = base_to_map_slam_stamped.translation.y;
        
        //RCLCPP_INFO(this->get_logger(), "[BASE_LINK_TO_ODOM]: %f %f", transformStamped_odom.transform.translation.x, transformStamped_odom.transform.translation.y);
        //RCLCPP_INFO(this->get_logger(), "[ODOM_TO_MAP_SLAM]: %f %f",  transformStamped.transform.translation.x,  transformStamped.transform.translation.y);
        //RCLCPP_INFO(this->get_logger(), "[BASE_LINK_TO_MAP_SLAM]: %f %f", message.pose.position.x, message.pose.position.y);
        
        
        nav_msgs::msg::Odometry odom;
        odom.header.frame_id = "map";
        odom.child_frame_id = "Base_Link";
        odom.header.stamp = transformStamped.header.stamp;

        odom.pose.pose.position.x = base_to_map_slam_stamped.translation.x;
        odom.pose.pose.position.y = base_to_map_slam_stamped.translation.y;

        odom.pose.pose.orientation.x = base_to_map_slam_stamped.rotation.x;
        odom.pose.pose.orientation.y = base_to_map_slam_stamped.rotation.y;
        odom.pose.pose.orientation.z = base_to_map_slam_stamped.rotation.z;
        odom.pose.pose.orientation.w = base_to_map_slam_stamped.rotation.w;

        // Inform covariances
        odom.pose.covariance[0] = 0.05;
        odom.pose.covariance[7] = 0.05;
        odom.pose.covariance[14] = 1.0;
        odom.pose.covariance[21] = 0.3;
        odom.pose.covariance[28] = 0.3;
        odom.pose.covariance[35] = 0.3;

        odom.twist.covariance[0] = 0.1;
        odom.twist.covariance[7] = 0.1;
        odom.twist.covariance[14] = 0.1;
        odom.twist.covariance[21] = 1.0;
        odom.twist.covariance[28] = 1.0;
        odom.twist.covariance[35] = 1.0;

        // Publish odom message
        //this->pubSlamOdom->publish(odom);

      // ------- Save orientation from Odom measurement
      tf2::Quaternion q(odom.pose.pose.orientation.x,
                        odom.pose.pose.orientation.y,
                        odom.pose.pose.orientation.z,
                        odom.pose.pose.orientation.w);

      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      this->headingSlam = yaw;		
    }
    catch (tf2::TransformException & ex) {
          RCLCPP_ERROR(this->get_logger(), "StaticLayer: %s", ex.what());
    }
  
  }

  void marvelmind1QualityCallback(const localization_interfaces::msg::HedgeQuality::SharedPtr hedge_quality_msg)
  {
    //RCLCPP_INFO(this->get_logger(), "Publishing quality data");	
    marvel1Quality = hedge_quality_msg->quality_percents;
  }

  void marvelmind2QualityCallback(const localization_interfaces::msg::HedgeQuality::SharedPtr hedge_quality_msg)
  {
    //RCLCPP_INFO(this->get_logger(), "Publishing quality data");	
    marvel2Quality = hedge_quality_msg->quality_percents;
  }

  void slamOdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
  {
    //this->publishOdomSlam();

    //RCLCPP_INFO(this->get_logger(), "Here! 2");
    // Save odom value to calculate slam position
    calcOdom.pose.pose = odom->pose.pose;

    //RCLCPP_INFO(this->get_logger(), "Publishing odom with covariance");	
    // ------- Save orientation from Odom measurement
    tf2::Quaternion q(odom->pose.pose.orientation.x,
                      odom->pose.pose.orientation.y,
                      odom->pose.pose.orientation.z,
                      odom->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    //this->headingOdom = - yaw - PI;
    headingSlam = yaw + PI;

    if(headingSlam > PI)
      headingSlam -= 2*PI;
    else if(headingSlam < -PI)
      headingSlam += 2*PI;

    //q.setRPY(roll, pitch, this->headingOdom);

    //odom->pose.pose.orientation.x = q.x();
    //odom->pose.pose.orientation.y = q.y();
    //odom->pose.pose.orientation.z = q.z();
    //odom->pose.pose.orientation.w = q.w();

    // Republish raw odom values with calculated odometry covariances 
    odom->pose.covariance = calcOdom.pose.covariance;
    
    //odom->pose.pose.position.x = odomOffset[0] - odom->pose.pose.position.x;
    //odom->pose.pose.position.y = odomOffset[1] + odom->pose.pose.position.y;
    //odom->pose.pose.position.x = odom->pose.pose.position.x;
    //odom->pose.pose.position.y = odom->pose.pose.position.y;

    // Verify foward velocity in order to determine whether the robot is moving or not using a set threshold
    if(std::fabs(odom->twist.twist.linear.x) < 0.01 )
    {
      // Set movement flag
      is_moving = false;

      // If the robot is not moving, increase IMU covariance
      //calcImu.linear_acceleration_covariance[0] = 0.5;
      //calcImu.linear_acceleration_covariance[4] = 0.5;
      //calcImu.linear_acceleration_covariance[8] = 0.5;  
    } else
    {
      // Set movement flag
      is_moving = true;

      // If the robot is moving, decrease IMU covariance
      //calcImu.linear_acceleration_covariance[0] = 0.1;
      //calcImu.linear_acceleration_covariance[4] = 0.1;
      //calcImu.linear_acceleration_covariance[8] = 0.1;  
    }

    geometry_msgs::msg::TwistWithCovariance twistMsg = odom->twist;
    
    //this->pubTwist->publish(twistMsg);
    //this->pubOdom->publish(*odom);
  }

  void odomIidreCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
  {
    odom->child_frame_id = "uwb";
    odom->header.frame_id = "_map";

    // Save current values to remove offsets
    iidreBeacon1.bodyPose.x = 0.6586;
    iidreBeacon1.bodyPose.y = -0.597;

    // Calculate offsets 
    float OFFSET_X = iidreBeacon1.bodyPose.x*cos(headingSlam+PI) - iidreBeacon1.bodyPose.y*sin(headingSlam+PI);
    float OFFSET_Y = iidreBeacon1.bodyPose.y*cos(headingSlam+PI) + iidreBeacon1.bodyPose.x*sin(headingSlam+PI);

    iidreBuffer.x->push(odom->pose.pose.position.x);
    iidreBuffer.y->push(odom->pose.pose.position.y);

    // Remove position offset due to beacon placement in the robot
    odom->pose.pose.position.x = iidreBuffer.x->mean() - OFFSET_X; 
    odom->pose.pose.position.y = iidreBuffer.y->mean() - OFFSET_Y; 
    odom->pose.pose.position.z = 0.0; 

    // Estimate orientation from trajectory
    headingIidre = orientationFromMarvel();

    if(!distanceValidity(1)||!distanceValidity(2))
    {
      //RCLCPP_INFO(this->get_logger(), "Here");
      calcIdre.pose.covariance[0] = 10;
      calcIdre.pose.covariance[7] = 10;
      calcIdre.pose.covariance[14] = 1.0;
      calcIdre.pose.covariance[21] = 500;
      calcIdre.pose.covariance[28] = 500;
      calcIdre.pose.covariance[35] = 500;

      calcIdre.twist.covariance[0] = 1000;
      calcIdre.twist.covariance[7] = 1000;
      calcIdre.twist.covariance[14] = 500;
      calcIdre.twist.covariance[21] = 500;
      calcIdre.twist.covariance[28] = 500;
      calcIdre.twist.covariance[35] = 100;
    }

    // Odometry covariances as the calculated values
    odom->pose.covariance = calcIdre.pose.covariance;

    // Save orientation data in odom message
    tf2::Quaternion q;
    q.setRPY(0, 0, headingIidre);  // Create this quaternion from roll/pitch/yaw (in radians)
    odom->pose.pose.orientation.x = q.x();
    odom->pose.pose.orientation.y = q.y();
    odom->pose.pose.orientation.z = q.z();
    odom->pose.pose.orientation.w = q.w();
    
    // Estimate orientation from trajectory
    if(iidreLastPosition.x == 0.0)
      iidreLastPosition.x = odom->pose.pose.position.x;

    if(iidreLastPosition.y == 0.0)
      iidreLastPosition.y = odom->pose.pose.position.y;

    float deltax = odom->pose.pose.position.x - iidreLastPosition.x;
    float deltay = odom->pose.pose.position.y - iidreLastPosition.y;
    float ds = sqrt(deltax*deltax + deltay*deltay);

    // Publish difference between heading estimate from iidre and imu
    std_msgs::msg::Float64 var;
    var.data = headingIidre*180/3.1415;
    //this->pubIidreHeading->publish(var);

    // Save last values in order to estimate orientation
    iidreLastPosition.x = odom->pose.pose.position.x;
    iidreLastPosition.y = odom->pose.pose.position.y;

    if((fabs(ds) > 1.0 ))
      return ;

    odom->twist.twist.angular.z = - odom->twist.twist.angular.z; 

    // Republish new odometry value with calculated covariances
    this->pubIidreOdom->publish(*odom);
  }

  void odomMarvel1Callback(const nav_msgs::msg::Odometry::SharedPtr odom)
  {
    // Check current quality 
    //if(marvel1Quality != 100.0)
      //return ;

    odom->child_frame_id = "Right_Front_Marvel1_Link";
    odom->header.frame_id = "_map";

    // Odometry covariances as the calculated values
    odom->pose.covariance = calcMarvel.pose.covariance;

    // Save current values to remove offsets
    marvelmindBeacon1.bodyPose.x = -0.63858;
    marvelmindBeacon1.bodyPose.y = -0.482;

    // Calculate offsets 
    float OFFSET_X = marvelmindBeacon1.bodyPose.x*cos(headingSlam+PI) - marvelmindBeacon1.bodyPose.y*sin(headingSlam+PI);
    float OFFSET_Y = marvelmindBeacon1.bodyPose.y*cos(headingSlam+PI) + marvelmindBeacon1.bodyPose.x*sin(headingSlam+PI);

    marvel1Buffer.x->push(odom->pose.pose.position.x);
    marvel1Buffer.y->push(odom->pose.pose.position.y);

    float correctedValues[2] = {marvel1Buffer.x->mean() - OFFSET_X, marvel1Buffer.y->mean() - OFFSET_Y};
    checkNearestSpotsOffsets(marvelmindBeacon1, correctedValues);

    // Remove position offset due to beacon placement in the robot
    odom->pose.pose.position.x = correctedValues[0]; 
    odom->pose.pose.position.y = correctedValues[1]; 
    odom->pose.pose.position.z = 0.0; 
    
    // Estimate orientation from trajectory
    headingMarvel1 = orientationFromMarvel();

    // Save orientation data in odom message
    tf2::Quaternion q;
    q.setRPY(0, 0, headingMarvel1);  // Create this quaternion from roll/pitch/yaw (in radians)
    odom->pose.pose.orientation.x = q.x();
    odom->pose.pose.orientation.y = q.y();
    odom->pose.pose.orientation.z = q.z();
    odom->pose.pose.orientation.w = q.w();

    // Verify jumps
    if(marvel1LastPosition.x == 0.0)
      marvel1LastPosition.x = odom->pose.pose.position.x;

    if(marvel1LastPosition.y == 0.0)
      marvel1LastPosition.y = odom->pose.pose.position.y;

    float deltax = odom->pose.pose.position.x - marvel1LastPosition.x;
    float deltay = odom->pose.pose.position.y - marvel1LastPosition.y;
    float ds = sqrt(deltax*deltax + deltay*deltay);
    
    // Save last values in order to estimate orientation
    marvel1LastPosition.x = odom->pose.pose.position.x;
    marvel1LastPosition.y = odom->pose.pose.position.y;

    if((fabs(ds) > 0.3) || !distanceValidity(1))
      return ;

    // Republish new odometry value with calculated covariances
    this->pubMarvel1Odom->publish(*odom);
  }

  void odomMarvel2Callback(const nav_msgs::msg::Odometry::SharedPtr odom)
  {
    //RCLCPP_INFO(this->get_logger(), "Publishing odom with covariance  %f", marvel2Quality);	
    // Check current quality 
    //if(marvel2Quality != 100.0)
      //return ;

    odom->child_frame_id = "Left_Botton_Marvel2_Link";
    odom->header.frame_id = "_map";

    // Odometry covariances as the calculated values
    odom->pose.covariance = calcMarvel.pose.covariance;

    // Save current values to remove offsets
    marvelmindBeacon2.bodyPose.x = 0.68947;
    marvelmindBeacon2.bodyPose.y = 0.282;
//TODO: TEM QUE Sumar pi ao headingSlam
    // Calculate offsets 
    float OFFSET_X = marvelmindBeacon2.bodyPose.x*cos(headingSlam+PI) - marvelmindBeacon2.bodyPose.y*sin(headingSlam+PI);
    float OFFSET_Y = marvelmindBeacon2.bodyPose.y*cos(headingSlam+PI) + marvelmindBeacon2.bodyPose.x*sin(headingSlam+PI);

    // Save values in buffer in order to filter
    marvel2Buffer.x->push(odom->pose.pose.position.x);
    marvel2Buffer.y->push(odom->pose.pose.position.y);

    float correctedValues[2] = {marvel2Buffer.x->mean() - OFFSET_X, marvel2Buffer.y->mean() - OFFSET_Y};
    checkNearestSpotsOffsets(marvelmindBeacon2, correctedValues);

    // Remove position offset due to beacon placement in the robot
    odom->pose.pose.position.x = correctedValues[0]; 
    odom->pose.pose.position.y = correctedValues[1]; 
    odom->pose.pose.position.z = 0.0; 

    // Estimate orientation from trajectory
    headingMarvel2 = orientationFromMarvel();

    // Save orientation data in odom message
    tf2::Quaternion q;
    q.setRPY(0, 0, headingMarvel2);  // Create this quaternion from roll/pitch/yaw (in radians)
    odom->pose.pose.orientation.x = q.x();
    odom->pose.pose.orientation.y = q.y();
    odom->pose.pose.orientation.z = q.z();
    odom->pose.pose.orientation.w = q.w();

    // Verify jumps
    if(marvel2LastPosition.x == 0.0)
      marvel2LastPosition.x = odom->pose.pose.position.x;

    if(marvel2LastPosition.y == 0.0)
      marvel2LastPosition.y = odom->pose.pose.position.y;

    float deltax = odom->pose.pose.position.x - marvel2LastPosition.x;
    float deltay = odom->pose.pose.position.y - marvel2LastPosition.y;
    float ds = sqrt(deltax*deltax + deltay*deltay);

    // Save last values in order to estimate orientation
    marvel2LastPosition.x = odom->pose.pose.position.x;
    marvel2LastPosition.y = odom->pose.pose.position.y;

    if((fabs(ds) > 0.3) || !distanceValidity(2))
      return ;

    // Republish new odometry value with calculated covariances
    this->pubMarvel2Odom->publish(*odom);
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr imu)
  {
    // ------- Initializations
    builtin_interfaces::msg::Time currentTime = imu->header.stamp;
    imu->header.stamp = this->now();
    //imu->header.frame_id = "IMU_MTI_670_Link";

    // ------- Check time difference since the last measurement in order de control the publication frequency
    //double dt = (double)(currentTime.sec - imuLastMeas.sec) + (double)(currentTime.nanosec - imuLastMeas.nanosec)/1000000000.0f;
    //double currentFrequency = 0;

    //if(dt != 0) currentFrequency = 1/dt;

    // ------- Verify if the robot is moving in order to calculate offsets
    if(!is_moving)
    {
      imuOffset.accelerometer.x->push(imu->linear_acceleration.x);
      imuOffset.accelerometer.y->push(imu->linear_acceleration.y);
      imuOffset.accelerometer.z->push(imu->linear_acceleration.z);

      imuOffset.gyro.x->push(imu->angular_velocity.x);
      imuOffset.gyro.y->push(imu->angular_velocity.y);
      imuOffset.gyro.z->push(imu->angular_velocity.z);
    }

    // ------- Update covariances	
    imu->angular_velocity_covariance = calcImu.angular_velocity_covariance;
    imu->linear_acceleration_covariance = calcImu.linear_acceleration_covariance;
    imu->orientation_covariance = calcImu.orientation_covariance;

    if(imu->angular_velocity.z > 0.01)
    {
      //calcOdom.pose.covariance[0] = 900;
      //calcOdom.pose.covariance[7] = 900;

      //calcOdom.twist.covariance[0] = 0.1;
      //calcOdom.twist.covariance[7] = 0.1;
      //calcOdom.twist.covariance[35] = 0.1;

    } else
    {
      //calcOdom.pose.covariance[0] = 500;
      //calcOdom.pose.covariance[7] = 500;

      //calcOdom.twist.covariance[0] = 0.1;
      //calcOdom.twist.covariance[7] = 0.1;
      //calcOdom.twist.covariance[35] = 0.001; 
    }

    // ------- Remove offsets
    //imu->linear_acceleration.x = imu->linear_acceleration.x - imuOffset.accelerometer.x->mean();
    //imu->linear_acceleration.y = imu->linear_acceleration.y - imuOffset.accelerometer.y->mean();
    //imu->linear_acceleration.z = imu->linear_acceleration.z - imuOffset.accelerometer.z->mean();  
    
    //imu->angular_velocity.x = imu->angular_velocity.x - imuOffset.gyro.x->mean();
    //imu->angular_velocity.y = imu->angular_velocity.y - imuOffset.gyro.y->mean();
    //imu->angular_velocity.z = imu->angular_velocity.z - imuOffset.gyro.z->mean();  
    
    // ------- Save orientation from IMU measurement
    tf2::Quaternion q(imu->orientation.x,
                      imu->orientation.y,
                      imu->orientation.z,
                      imu->orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // There is a offset of pi rad
    headingImu = yaw - PI;

    if(headingImu > PI)
      headingImu -= 2*PI;
    else if(headingImu < -PI)
      headingImu += 2*PI;

    std_msgs::msg::Float64 var;
    var.data = headingImu*180/PI;
    this->pubImuHeading->publish(var);

    //RCLCPP_INFO(this->get_logger(), "%f  %f \n", yaw, headingImu);	

    // ------- Publish
    this->pubImu->publish(*imu);
    imuLastMeas = currentTime; 
  }

 void scanBackCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
 {
   scan->header.stamp = this->now();
   this->pubScanBack->publish(*scan);
 }

 void scanFrontCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
 {
   scan->header.stamp = this->now();
   this->pubScanFront->publish(*scan);
 }

  void tfCallback(tf2_msgs::msg::TFMessage::SharedPtr tf_msg)
    {
      //RCLCPP_INFO(this->get_logger(), "Here! 3");
      int num_tfs = tf_msg->transforms.size();
      std_msgs::msg::Int32MultiArray array;
      array.data = {0, 0, 0, 0, 0, 0, 0, 0};
      bool is_new_data = true;
      std_msgs::msg::Float64 var;
      var.data = 0.0;
    
      for (int i = 0; i < num_tfs; i++)
      {        
        if (tf_msg->transforms.at(i).child_frame_id == "1565091E")
        {
          array.data.at(0) = tf_msg->transforms.at(i).transform.rotation.x; //FP_PWR_LVL: first-path (*1000) power in dBm
          array.data.at(1) = tf_msg->transforms.at(i).transform.rotation.y; //IDIFF: LOS/NLOS communication indicator, w/ unit
          
          var.data = (float)tf_msg->transforms.at(i).transform.rotation.y;
          //pubQualBeacon1->publish(var);
        } 
        else if (tf_msg->transforms.at(i).child_frame_id == "15648BA2"){
          array.data.at(2) = tf_msg->transforms.at(i).transform.rotation.x;  
          array.data.at(3) = tf_msg->transforms.at(i).transform.rotation.y;

          var.data = tf_msg->transforms.at(i).transform.rotation.y;
          //pubQualBeacon2->publish(var);
        } 
        else if (tf_msg->transforms.at(i).child_frame_id == "55644820"){
          array.data.at(4) = tf_msg->transforms.at(i).transform.rotation.x;
          array.data.at(5) = tf_msg->transforms.at(i).transform.rotation.y;

          var.data = tf_msg->transforms.at(i).transform.rotation.y;
          //pubQualBeacon3->publish(var); 
        } 
        else if (tf_msg->transforms.at(i).child_frame_id == "55650120"){
          array.data.at(6) = tf_msg->transforms.at(i).transform.rotation.x;
          array.data.at(7) = tf_msg->transforms.at(i).transform.rotation.y;  

          var.data = tf_msg->transforms.at(i).transform.rotation.y;
          //pubQualBeacon4->publish(var);
        
        }
        else if (tf_msg->transforms.at(i).child_frame_id == "hedge"){
          tf_msg->transforms.erase(tf_msg->transforms.cbegin()+i);
        } 
        else if (tf_msg->transforms.at(i).child_frame_id == "uwb"){
          tf_msg->transforms.erase(tf_msg->transforms.cbegin()+i);
        } 
        //else if (tf_msg->transforms.at(i).header.frame_id == "odom"){
          //tf_msg->transforms.erase(tf_msg->transforms.cbegin()+i);

        //}
        else if (tf_msg->transforms.at(i).header.frame_id == "map_slam"){
          //tf_msg->transforms.erase(tf_msg->transforms.cbegin()+i);
          slamPose.x = tf_msg->transforms.at(i).transform.translation.x;
          slamPose.y = tf_msg->transforms.at(i).transform.translation.y;
          slamPose.z = tf_msg->transforms.at(i).transform.translation.z;

          tf2::Quaternion q(tf_msg->transforms.at(i).transform.rotation.x,
                    tf_msg->transforms.at(i).transform.rotation.y,
                    tf_msg->transforms.at(i).transform.rotation.z,
                    tf_msg->transforms.at(i).transform.rotation.w);

          tf2::Matrix3x3 m(q);
          double roll, pitch, yaw;
          m.getRPY(roll, pitch, yaw);

          slamOrientation.z = yaw;

          //this->publishOdomSlam();
        } 
        else
        {
          is_new_data = false;
        }
      }

      if (is_new_data)
      {
        //pubIidreQuality->publish(array);
      }

      //pubTf->publish(*tf_msg);
    }

  void staticTfCallback(tf2_msgs::msg::TFMessage::SharedPtr tf_msg)
    {
      int num_tfs = tf_msg->transforms.size();

      for (int i = 0; i < num_tfs; i++)
      {        
        if (tf_msg->transforms.at(i).child_frame_id == "hedge")
        {
          marvelmindBeacon1.bodyPose.x = tf_msg->transforms.at(i).transform.translation.x;
          marvelmindBeacon1.bodyPose.y = tf_msg->transforms.at(i).transform.translation.y;
          marvelmindBeacon1.bodyPose.z = 0.0;
        } 
        else if(tf_msg->transforms.at(i).child_frame_id == "uwb")
        {
          iidreBeacon1.bodyPose.x = tf_msg->transforms.at(i).transform.translation.x;
          iidreBeacon1.bodyPose.y = tf_msg->transforms.at(i).transform.translation.y; 
          iidreBeacon1.bodyPose.z = 0.0;
        }
      }
    }
};

/**
 * Node for utilities necessary to evaluate scooby localization performance
 */
int main(int argc, char **argv)
{
	
  // initialize ROS node
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<LocalizationNode>());
  rclcpp::shutdown();

  return 0;
}
