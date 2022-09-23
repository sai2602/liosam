#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/Imu.h"
#include "common_ros.h"
#include "nav_msgs/Odometry.h"
#include "quaternion.h"
#include "std_msgs/Header.h"
#include "csv.h"

#include <eigen3/Eigen/Core>

#include <sstream>
#include <fstream>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//#include <stdlib.h>

ros::Subscriber sub_imu;
ros::Subscriber sub_odom;
ros::Subscriber sub_anavs_att_euler;
ros::Publisher pub_imu_raw;

geometry_msgs::PointStamped att_msg;
const double toRad = M_PI/180.f;

unsigned int v0_x = 0;
unsigned int v0_y = 0;
unsigned int v0_z = 0;
std_msgs::Header previous_odom_header;
bool flag = true;
unsigned int seq_offset = 0;
std::vector<Eigen::Vector3f> att_data;
std::vector<float> tow_data;
bool correct_initial_orientation = true;
geometry_msgs::Quaternion init_att_quaternion;
tf2::Quaternion init_att_quaternion_tf;

//std::ofstream myfile("values.txt");


void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  sensor_msgs::Imu imu_raw(*msg);

  imu_raw.header.frame_id = "imu_link";

  //convert att message from euler to quaternion
  geometry_msgs::Quaternion att_quaternion;
  eulerNED2quatENU(att_msg.point.z*toRad, att_msg.point.y*toRad, att_msg.point.x*toRad, att_quaternion);   //  x = att_head, y = att_pitch, z = att_roll

  //update imu orientation field 
  imu_raw.orientation = att_quaternion;

  imu_raw.orientation_covariance[0] = 0.01;
  imu_raw.orientation_covariance[4] = 0.01;
  imu_raw.orientation_covariance[8] = 0.01;

  //publish imu_raw topic
  pub_imu_raw.publish(imu_raw);

}

void attCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  att_msg.header = msg->header;
  att_msg.point = msg->point;
}


// create a quaternion with a -15 degrees rotation on the X axis, I should call createFromAxisAngle(1, 0, 0, -15*PI/180)
quaternion::Quaternion<double> create_from_axis_angle(const double &xx, const double &yy, const double &zz, const double &a)
{
    // Here we calculate the sin( theta / 2) once for optimization
    double factor = sin( a / 2.0 );

    // Calculate the x, y and z of the quaternion
    double x = xx * factor;
    double y = yy * factor;
    double z = zz * factor;

    // Calcualte the w value by cos( theta / 2 )
    double w = cos( a / 2.0 );

    return quaternion::normalize( quaternion::Quaternion<double> (x, y, z, w) );
}




void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  
  sensor_msgs::Imu imu_raw ;
	
  if (flag)
 {
	previous_odom_header = msg->header;
  	v0_x = msg->twist.twist.linear.x;
  	v0_y = msg->twist.twist.linear.y;
  	v0_z = msg->twist.twist.linear.z;
    seq_offset = msg->header.seq;
    flag=false;
    pub_imu_raw.publish(imu_raw);
	return; 
 }

  imu_raw.header = msg->header;
  imu_raw.header.frame_id = "imu_link";

  //copy contents from Odom message to imu message

  //create a quaternion from axis angle

  imu_raw.orientation = msg->pose.pose.orientation;

  quaternion::Quaternion<double> rot_y = create_from_axis_angle(0, 1, 0, 90*toRad);
  quaternion::Quaternion<double> rot_x = create_from_axis_angle(1, 0, 0, -90*toRad);
  quaternion::Quaternion<double> imu_raw_o (imu_raw.orientation.x, imu_raw.orientation.y, imu_raw.orientation.z, imu_raw.orientation.w);

  //multiply quarernion with imu_raw.orientation
  imu_raw_o = rot_x * rot_y * imu_raw_o;

  imu_raw.orientation.x = imu_raw_o.a();
  imu_raw.orientation.y = imu_raw_o.b();
  imu_raw.orientation.z = imu_raw_o.c();
  imu_raw.orientation.w = imu_raw_o.d();
  

  imu_raw.angular_velocity.x = msg->twist.twist.angular.x;
  imu_raw.angular_velocity.y = msg->twist.twist.angular.y;
  imu_raw.angular_velocity.z = msg->twist.twist.angular.z;

  if (((msg->header.seq-seq_offset)-(previous_odom_header.seq-seq_offset))==1)
 {

  //std::cout << "inside if stmt" << std::endl;

  float delta_t = previous_odom_header.stamp.sec + previous_odom_header.stamp.nsec*1e-9 - msg->header.stamp.sec - msg->header.stamp.nsec*1e-9;
  if (delta_t!=0.0)
  {
    imu_raw.linear_acceleration.x = (msg->twist.twist.linear.x-v0_x)/(delta_t);
    imu_raw.linear_acceleration.y = (msg->twist.twist.linear.y-v0_y)/(delta_t);
    imu_raw.linear_acceleration.z = (msg->twist.twist.linear.z-v0_z)/(delta_t);
  }

  imu_raw.orientation_covariance[0] = 0.01;
  imu_raw.orientation_covariance[4] = 0.01;
  imu_raw.orientation_covariance[8] = 0.01;

  imu_raw.angular_velocity_covariance[0] = 0.01;
  imu_raw.angular_velocity_covariance[4] = 0.01;
  imu_raw.angular_velocity_covariance[8] = 0.01;

  imu_raw.linear_acceleration_covariance[0] = 0.01;
  imu_raw.linear_acceleration_covariance[4] = 0.01;
  imu_raw.linear_acceleration_covariance[8] = 0.01;

  //publish imu_raw topic
  pub_imu_raw.publish(imu_raw);

  }

	previous_odom_header = msg->header;
  	v0_x = msg->twist.twist.linear.x;
  	v0_y = msg->twist.twist.linear.y;
  	v0_z = msg->twist.twist.linear.z;

}


void odomCallback2(const nav_msgs::Odometry::ConstPtr& msg)
{
  
  sensor_msgs::Imu imu_raw ;

  imu_raw.header = msg->header;
  imu_raw.header.frame_id = "imu_link";

  //copy contents from Odom message to imu message

  //create a quaternion from axis angle

  imu_raw.orientation = msg->pose.pose.orientation;

  quaternion::Quaternion<double> rot_y = create_from_axis_angle(0, 1, 0, 90*toRad);
  quaternion::Quaternion<double> rot_x = create_from_axis_angle(1, 0, 0, -90*toRad);
  quaternion::Quaternion<double> imu_raw_o (imu_raw.orientation.x, imu_raw.orientation.y, imu_raw.orientation.z, imu_raw.orientation.w);

  //multiply quarernion with imu_raw.orientation
  imu_raw_o = rot_x * rot_y * imu_raw_o;

  imu_raw.orientation.x = imu_raw_o.a();
  imu_raw.orientation.y = imu_raw_o.b();
  imu_raw.orientation.z = imu_raw_o.c();
  imu_raw.orientation.w = imu_raw_o.d();
  
  imu_raw.angular_velocity.x = msg->twist.twist.angular.x;
  imu_raw.angular_velocity.y = msg->twist.twist.angular.y;
  imu_raw.angular_velocity.z = msg->twist.twist.angular.z;

  imu_raw.linear_acceleration.x = msg->twist.twist.linear.x;
  imu_raw.linear_acceleration.y = msg->twist.twist.linear.y;
  imu_raw.linear_acceleration.z = msg->twist.twist.linear.z;

  imu_raw.orientation_covariance[0] = 0.01;
  imu_raw.orientation_covariance[4] = 0.01;
  imu_raw.orientation_covariance[8] = 0.01;

  imu_raw.angular_velocity_covariance[0] = 0.01;
  imu_raw.angular_velocity_covariance[4] = 0.01;
  imu_raw.angular_velocity_covariance[8] = 0.01;

  imu_raw.linear_acceleration_covariance[0] = 0.01;
  imu_raw.linear_acceleration_covariance[4] = 0.01;
  imu_raw.linear_acceleration_covariance[8] = 0.01;

  //publish imu_raw topic
  pub_imu_raw.publish(imu_raw);

}

float clockToGPSsecs(int secs, int nsecs)
{
	int current_leap_seconds = 18;
	int secondsSince1980_UTC = secs - 315964800;
	int secondsGPS = secondsSince1980_UTC + current_leap_seconds;
	int secondsPerWeek = 3600 * 24 * 7 ;
	int secondsOfWeek = secondsGPS % secondsPerWeek ;
	float t = secondsOfWeek + nsecs*1e-9;
	return t;
}

void imuWithOrientationCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  //create a new topic imu_raw
  sensor_msgs::Imu imu_raw(*msg);
  imu_raw.header.frame_id = "imu_link";

  //1. convert the header timestamp to TOW using function clock_to_gps_secs(stamp)
  float imu_raw_tow_secs = clockToGPSsecs(imu_raw.header.stamp.sec, imu_raw.header.stamp.nsec);

  //2. on global storage, search for the data to be published using TOW obtained in previous step 
  /*auto it = std::find_if (tow_data.begin(), tow_data.end(), [&imu_raw_tow_secs] (const float& val) -> bool  { 
		float abs_diff = std::abs(val-imu_raw_tow_secs);   
 	    if(abs_diff>=0 && abs_diff<0.02)
			return true;
		else
			return false; }); */

  std::vector<float> abs_diffs;

  for(auto& element : tow_data)
    abs_diffs.emplace_back(std::abs(element-imu_raw_tow_secs));

  //for(auto k : abs_diffs)
//	std::cout << k << std::endl;

  auto it = std::min_element(abs_diffs.begin(), abs_diffs.end());
  int index = -1 ;

  


  if(correct_initial_orientation)
  {
      eulerNED2quatENU(att_data[index][0]*toRad, att_data[index][1]*toRad, att_data[index][2]*toRad, init_att_quaternion);
	  correct_initial_orientation = false;
      imu_raw.orientation_covariance[0] = 0.0;
      imu_raw.orientation_covariance[4] = 0.0;
      imu_raw.orientation_covariance[8] = 0.0;

       // inverse initil orientation Quaternion
	  tf2::convert(init_att_quaternion, init_att_quaternion_tf);
      init_att_quaternion_tf = tf2::inverse(init_att_quaternion_tf);
      //tf2::Quaternion rot_y (0, -45*toRad, 0);
	  //tf2::Quaternion rot_x (0, -90*toRad, 0);
      //init_att_quaternion_tf = rot_x * rot_y * init_att_quaternion_tf;
      
  }



	// If element was found
  if (it != tow_data.end() && correct_initial_orientation==false)
  {
    // calculating the index
    index = std::distance(abs_diffs.begin(), it);
    //std::cout << index << std::endl;  
 


    
    //convert att message from euler to quaternion
    geometry_msgs::Quaternion att_quaternion;
    tf2::Quaternion att_quaternion_tf;

	//tf2::Quaternion att_quaternion_tf(att_data[index][0]*toRad, att_data[index][1]*toRad, att_data[index][2]*toRad);
    
    //eulerNED2quatENU(att_msg.point.z*toRad, att_msg.point.y*toRad, att_msg.point.x*toRad, att_quaternion);   //  x = att_head, y = att_pitch, z = att_roll
    eulerNED2quatENU(att_data[index][0]*toRad, att_data[index][1]*toRad, att_data[index][2]*toRad, att_quaternion);   //  x = att_head, y = att_pitch, z = att_roll

    tf2::convert(att_quaternion, att_quaternion_tf);

    // Multiply corrent orientation with inverse initial orientation



	//att_quaternion_tf = tf2::inverse(att_quaternion_tf);
    att_quaternion_tf =  init_att_quaternion_tf * att_quaternion_tf ;
	
    
    //std::cout << "x=" << att_quaternion_tf.x() << " y=" << att_quaternion_tf.y() << " z=" << att_quaternion_tf.z() << " w=" << att_quaternion_tf.w() <<  std::endl;

  	//quaternion::Quaternion<double> rot_x = create_from_axis_angle(0, 1, 0, 90*toRad);
  	quaternion::Quaternion<double> rot_z_135 = create_from_axis_angle(0, 0, 1, 135*toRad);
	quaternion::Quaternion<double> rot_z_180 = create_from_axis_angle(0, 0, 1, 180*toRad);
	quaternion::Quaternion<double> rot_y_180 = create_from_axis_angle(0, 1, 0, 180*toRad);
	//quaternion::Quaternion<double> rot_y = create_from_axis_angle(0, 1, 0, 45*toRad);
	//quaternion::Quaternion<double> rot_x = create_from_axis_angle(1, 0, 0, -90*toRad);
	

  	quaternion::Quaternion<double> imu_raw_o (att_quaternion_tf.x(), att_quaternion_tf.y(), att_quaternion_tf.z(), att_quaternion_tf.w());

    //multiply quarernion with imu_raw.orientation
    //imu_raw_o = rot_x * imu_raw_o;
    //imu_raw_o = rot_y_180 * rot_z_180 * rot_z_135 * imu_raw_o;

  	//quaternion::Quaternion<double> rot_z_ = create_from_axis_angle(0, 0, 1, 90*toRad);
  	//quaternion::Quaternion<double> rot_x_ = create_from_axis_angle(1, 0, 0, 90*toRad);

    //imu_raw_o = rot_z_ * rot_x_ * imu_raw_o;

  	//quaternion::Quaternion<double> rot_z_t2 = create_from_axis_angle(0, 0, 1, 90*toRad);
  	//quaternion::Quaternion<double> rot_x_t2 = create_from_axis_angle(1, 0, 0, 90*toRad);

	//imu_raw_o = rot_z_t2 * rot_x_t2 * imu_raw_o;

    //update imu orientation field 
	
  	imu_raw.orientation.x = imu_raw_o.a();
  	imu_raw.orientation.y = imu_raw_o.b();
 	imu_raw.orientation.z = imu_raw_o.c();
  	imu_raw.orientation.w = imu_raw_o.d();
	
    imu_raw.orientation_covariance[0] = 0.01;
    imu_raw.orientation_covariance[4] = 0.01;
    imu_raw.orientation_covariance[8] = 0.01;
	
  }

  //publish imu_raw topic

  //myfile << imu_raw_tow_secs << " " << tow_data[index] << std::setprecision(10) << std::endl ;
  //myfile.close();

  pub_imu_raw.publish(imu_raw);
	

}

void printPADcsv()
{

	for (auto i : att_data)
	{
		std::cout << i[0] <<" " << i[1] <<" "  << i[2] <<" " << std::endl;
	}

	for (auto j : tow_data)
	{
		std::cout << j << std::setprecision(10) << std::endl;
	}

}

void readPADcsv(std::string csv_path)
{
      std::cout << "Enter PADsolution csv file" << std::endl;
      //std::string file_name="/media/nagaraj/SSD_2TB/securepnt_data_collection/isp_recording_2021-10-11-13-01-19_2.bag.velodyne_packets_rtk.txt";
      //std::string file_name="/media/nagaraj/SSD_2TB/securepnt_data_collection/isp_recording_2021-10-11-12-31-20_1.bag.velodyne_packets_rtk.txt";
      //std::string file_name="/media/nagaraj/SSD_2TB/indoor_outdoor_parkingspace/isp_recording_2022-02-24-13-42-36_0.bag.velodyne_packets_rtk_accuracy_ECEF_baseline.txt";
      /*std::string file_name="/media/nagaraj/SSD_2TB/indoor_outdoor_parkingspace/isp_recording_2022-02-24-14-12-37_1.bag.velodyne_packets_rtk_accuracy_ECEF_baseline.txt";
      std::cin >> file_name;
      io::CSVReader<4> in(file_name);*/
      io::CSVReader<4> in(csv_path);
      std::cout << "Reading attitude from csv : " << csv_path << std::endl;
      in.read_header(io::ignore_extra_column, "tow", "att0", "att1", "att2");
      std::string tow;
      std::string att0;
      std::string att1;
      std::string att2;
      //unsigned int counter = 0; 

      while(in.read_row(tow, att0, att1, att2))
      {
        if (tow=="nan" || tow=="NaN")
		  tow= std::nanf(tow.c_str());
        if (att0=="nan" || att0=="NaN")
		  att0= std::nanf(att0.c_str());
        if (att1=="nan" || att1=="NaN")
		  att1= std::nanf(att1.c_str());
        if (att2=="nan" || att2=="NaN")
		  att2= std::nanf(att2.c_str());

        //std::cout << "tow after reading from file : " << tow << std::endl;
          
        tow_data.emplace_back(std::stof(tow));				//assuming TOW info from the PAD solution csv is always valid and available

        try
	    {
      	  att_data.emplace_back(Eigen::Vector3f({std::stof(att0), std::stof(att1), std::stof(att2)}));
          
          //std::cout << "tow std::stof(tow) : " << std::stof(tow) << std::setprecision(10) << std::endl;
        } 
        catch (std::invalid_argument const& ex) 
        {
          att_data.emplace_back(Eigen::Vector3f({0.0,0.0,0.0}));        
        }
       
        //counter+=1;     
      
      }

	 if(att_data.size()!=tow_data.size())
	 {
		std::cout << "ERROR in data reading from CSV! att_data.size()!=tow_data.size()" << std::endl; 
		return; 
	 }
     std::cout << "Done. " << std::endl;
     std::cout << "length of attitude data : " << att_data.size() << std::endl;


    std::ofstream output_file("tow_data.txt");
    //std::ostream_iterator<float> output_iterator(output_file, "\n");
    //for(auto h: tow_data )
	//	output_file << h << std::setprecision(10) << std::endl;
}




int main(int argc, char **argv)
{

  ros::init(argc, argv, "ros_topic_converter");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string setting, csv_path;
  nh_private.param("setting", setting, std::string(""));
  nh_private.param("csv_path", csv_path, std::string(""));
  std::cout << "settting: " << setting << std::endl;

  if (!strcmp(setting.c_str(),"d435i_to_imu_raw")) {
      ROS_INFO("Read topics : /anavs/solution/att_euler and /d435i/imu \nPublish new IMU topic: /imu_raw");
      sub_imu = nh.subscribe("/d435i/imu", 1000, imuCallback);
      sub_anavs_att_euler = nh.subscribe("/anavs/solution/att_euler", 1000, attCallback);
      pub_imu_raw = nh.advertise<sensor_msgs::Imu>("imu_raw", 1000);
  } 
  else if (!strcmp(setting.c_str(),"odom_to_imu_raw")) {
      ROS_INFO("Read topics : /camera/odom/sample \nPublish new IMU topic: /imu_raw");
      sub_odom = nh.subscribe("/camera/odom/sample", 1000, odomCallback2);
      pub_imu_raw = nh.advertise<sensor_msgs::Imu>("imu_raw", 1000);
  }
	
  else if (!strcmp(setting.c_str(),"read_sol")) {

      //read the PADSolution file and store the attitude data in a global storage (indexed with TOW)

	  readPADcsv(csv_path);
	  //printPADcsv();

	  //subscribe to ROS topic /d435i/imu of type sensor_msgs/Imu
      //ROS_INFO("Read topic : /imu_raw \nPublish new IMU topic with PADSolution orientation: /imu_correct");
      
      std::cout << "Read topic : /imu_raw \nPublish new IMU topic with PADSolution orientation: /imu_correct" << std::endl;
      
      sub_imu = nh.subscribe("/imu_raw", 1000, imuWithOrientationCallback);
      pub_imu_raw = nh.advertise<sensor_msgs::Imu>("/imu_correct", 1000);
  }	

  else {
     ROS_INFO("Unknown setting.");
     return 0;
  }

  ros::spin();

  return 0;
}
