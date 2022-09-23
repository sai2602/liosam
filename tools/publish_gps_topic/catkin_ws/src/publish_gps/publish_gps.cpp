#include <iostream>
#include <cmath>
#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/TimeReference.h"
#include "csv.h"
#include <eigen3/Eigen/Core>

std::vector<Eigen::Vector4f> gps_data;
std::vector<Eigen::Vector3f> bStdDev_data;
std::vector<float> tow_data;
ros::Publisher pub_gps;
ros::Subscriber sub_lidar;
ros::Publisher pub_time_ref;
int nsecs[5] = {000000000,200000000,400000000,600000000,800000000};
int nsecs_index = 0;
int pub_counter = 0;

// "Publish GPS data inside lidar callback"

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

void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& lidar)
{
	//create a new topic GPS
	sensor_msgs::NavSatFix gps_msg;
	gps_msg.header = lidar->header;
	gps_msg.header.frame_id = "navsat_link";
	
	sensor_msgs::TimeReference time_ref_msg;
	time_ref_msg.header = lidar->header;
	time_ref_msg.header.frame_id = "navsat_link";
	
	
	

	//1. convert the header timestamp to TOW using function clock_to_gps_secs(stamp)
	float lidar_tow_secs = clockToGPSsecs(lidar->header.stamp.sec, lidar->header.stamp.nsec);
	
	
	

	//2. on global storage, search for the data to be published using TOW obtained in previous step 

	std::vector<float> abs_diffs;

	for(auto& element : tow_data)
		abs_diffs.emplace_back(std::abs(element-lidar_tow_secs));

	auto it = std::min_element(abs_diffs.begin(), abs_diffs.end());
	int index = -1 ;
	
	if (it != tow_data.end())	//element was found 
	{
		// calculating the index
		index = std::distance(abs_diffs.begin(), it);
		//3. Fill GPS message contents 
		gps_msg.status.status = 2;
		gps_msg.status.service = 1;
		gps_msg.latitude = gps_data[index][0];
		gps_msg.longitude = gps_data[index][1];
		gps_msg.altitude = gps_data[index][2];
		gps_msg.position_covariance[0] = bStdDev_data[index][0];
		gps_msg.position_covariance[4] = bStdDev_data[index][1];
		gps_msg.position_covariance[8] = bStdDev_data[index][2];
		gps_msg.position_covariance_type = 2;
		
		pub_gps.publish(gps_msg);
		
		pub_counter++;
		
		if (pub_counter>3)
		{
			time_ref_msg.time_ref = ros::Time(lidar->header.stamp.sec, nsecs[nsecs_index++]);
			pub_time_ref.publish(time_ref_msg);
			pub_counter = 0;
			
			if(nsecs_index>4)
				nsecs_index = 0;

		}
		
	}
}

void readPADcsv()
{
      std::cout << "Enter PADsolution csv file" << std::endl;
      //std::string file_name="/media/nagaraj/SSD_2TB/securepnt_data_collection/isp_recording_2021-10-11-13-01-19_2.bag.velodyne_packets_rtk.txt";
      //std::string file_name="/media/nagaraj/SSD_2TB/securepnt_data_collection/isp_recording_2021-10-11-12-31-20_1.bag.velodyne_packets_rtk_accuracy.txt";
      //std::string file_name="/media/nagaraj/SSD_2TB/securepnt_data_collection/isp_recording_2021-10-11-12-31-20_1.bag.velodyne_packets_rtk_accuracy_bStdDev.txt";
      //std::string file_name = "/media/nagaraj/SSD_2TB/indoor_outdoor_parkingspace/isp_recording_2022-02-24-13-42-36_0.bag.velodyne_packets_rtk_accuracy_bStdDev_baseline.txt";
      std::string file_name = "/media/nagaraj/SSD_2TB/indoor_outdoor_parkingspace/isp_recording_2022-02-24-14-12-37_1.bag.velodyne_packets_rtk_accuracy_bStdDev_baseline.txt";
      
      //std::cin >> file_name;
      io::CSVReader<8> in(file_name);
      std::cout << "Reading attitude from csv : " << file_name << std::endl;
      in.read_header(io::ignore_extra_column, "tow", "lat", "lon", "alt", "accuracy", "bStdDeviation0", "bStdDeviation1", "bStdDeviation2");
      std::string tow;
      std::string lat;
      std::string lon;
      std::string alt;
      std::string accuracy;
      std::string bStdDeviation0;
      std::string bStdDeviation1;
      std::string bStdDeviation2;
      //unsigned int counter = 0; 

      while(in.read_row(tow, lat, lon, alt, accuracy, bStdDeviation0, bStdDeviation1, bStdDeviation2))
      {
        if (tow=="nan" || tow=="NaN")
		  tow= std::nanf(tow.c_str());
        if (lat=="nan" || lat=="NaN")
		  lat= std::nanf(lat.c_str());
        if (lon=="nan" || lon=="NaN")
		  lon= std::nanf(lon.c_str());
        if (alt=="nan" || alt=="NaN")
		  alt= std::nanf(alt.c_str());
		if (accuracy=="nan" || accuracy=="NaN")
		  accuracy= std::nan(accuracy.c_str());
		if (bStdDeviation0=="nan" || bStdDeviation0=="NaN")
		  bStdDeviation0= std::nan(bStdDeviation0.c_str());		
		if (bStdDeviation1=="nan" || bStdDeviation1=="NaN")
		  bStdDeviation1= std::nan(bStdDeviation1.c_str());		
		if (bStdDeviation2=="nan" || bStdDeviation2=="NaN")
		  bStdDeviation2= std::nan(bStdDeviation2.c_str());
        //std::cout << "tow after reading from file : " << tow << std::endl;
          
        tow_data.emplace_back(std::stof(tow));				//assuming TOW info from the PAD solution csv is always valid and available

        try
	    {
	    
      	  gps_data.emplace_back(Eigen::Vector4f({std::stof(lat), std::stof(lon), std::stof(alt), std::stof(accuracy)}));
          
          //std::cout << "tow std::stof(tow) : " << std::stof(tow) << std::setprecision(10) << std::endl;
        } 
        catch (std::invalid_argument const& ex) 
        {
          gps_data.emplace_back(Eigen::Vector4f({0.0, 0.0, 0.0, 0.0}));        
        }
       
        try
	    {
	    
      	  //bStdDev_data.emplace_back(Eigen::Vector3f({	std::stof(bStdDeviation1)*std::stof(bStdDeviation1), 
      	  //											 	std::stof(bStdDeviation0)*std::stof(bStdDeviation0), 
      	  //											 	std::stof(bStdDeviation2)*std::stof(bStdDeviation2)
      	  //											 }));
      	  
      	  bStdDev_data.emplace_back(Eigen::Vector3f({	std::pow(std::stof(bStdDeviation1),2), 
      	  											 	std::pow(std::stof(bStdDeviation0),2), 
      	  											 	std::pow(std::stof(bStdDeviation2),2)
      	  											 }));
          
          //std::cout << "tow std::stof(tow) : " << std::stof(tow) << std::setprecision(10) << std::endl;
        } 
        catch (std::invalid_argument const& ex1) 
        {
          bStdDev_data.emplace_back(Eigen::Vector3f({0.0, 0.0, 0.0}));        
        }
       
        //counter+=1;     
      
      }

	 if(gps_data.size()!=tow_data.size() || bStdDev_data.size()!=tow_data.size())
	 {
		std::cout << "ERROR in data reading from CSV! gps_data.size()!=tow_data.size() or bStdDev_data.size()!=tow_data.size()" << std::endl; 
		return; 
	 }
     std::cout << "Done. " << std::endl;
     std::cout << "length of attitude data : " << gps_data.size() << std::endl;


    //std::ofstream output_file("tow_data.txt");
    //std::ostream_iterator<float> output_iterator(output_file, "\n");
    //for(auto h: tow_data )
	//	output_file << h << std::setprecision(10) << std::endl;
}


void printPADcsv()
{

	for (auto i : gps_data)
	{
		std::cout << i[0] <<" " << i[1] <<" "  << i[2] <<" " << std::endl;
	}

	/*for (auto j : tow_data)
	{
		std::cout << j << std::setprecision(10) << std::endl;
	}*/

}

int main(int argc, char** argv)
{
	readPADcsv();
	//printPADcsv();
	ros::init(argc, argv, "gps_publisher");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	ROS_INFO("Read PADSolution file and Publish GPS topic: /gps/fix and /gps/time_reference");

	sub_lidar=nh.subscribe("/velodyne_points", 1000, lidarCallback);
	pub_gps = nh.advertise<sensor_msgs::NavSatFix>("/gps/fix", 1);	
	pub_time_ref = nh.advertise<sensor_msgs::TimeReference>("/gps/time_reference", 1);
	
	ros::spin();
	
	return 0;		
}



