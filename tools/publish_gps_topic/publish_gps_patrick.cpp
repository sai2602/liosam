/*	date: 10. Feb 2022
 *	comment: see the original unchanged file at /home/nagaraj/Development/tools/publish_gps_topic/publish_gps.cpp
 */



#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/PointCloud2.h"
#include "nav_msgs/Path.h"
#include "csv.h"
#include <eigen3/Eigen/Core>
#include <iomanip>

std::vector<Eigen::Vector4f> gps_data;
std::vector<Eigen::Vector3f> bStdDev_data;
std::vector<float> tow_data;
//ros::Publisher pub_gps;
//ros::Subscriber sub_lidar;

ros::Subscriber sub_path;
std::ofstream outFile;
int curr_count = 0;
int prev_count = 0;
bool first_flag = true;

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
/*
void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& lidar)
{
	//create a new topic GPS
	sensor_msgs::NavSatFix gps_msg;
	gps_msg.header = lidar->header;
	gps_msg.header.frame_id = "navsat_link";

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
		gps_msg.position_covariance_type = 1;
		
		pub_gps.publish(gps_msg);
		
	}
}


*/


void pathCallback (const nav_msgs::Path::ConstPtr& path)
{

	// convert message header to TOW
	
	float path_tow_secs = clockToGPSsecs(path->header.stamp.sec, path->header.stamp.nsec);
	
	// find closest match in GPS data 
	
	//2. on global storage, search for the data to be published using TOW obtained in previous step 

	std::vector<float> abs_diffs;

	for(auto& element : tow_data)
		abs_diffs.emplace_back(std::abs(element-path_tow_secs));

	auto it = std::min_element(abs_diffs.begin(), abs_diffs.end());
	int index = -1 ;
	
	
	for (auto& p : path->poses)
		curr_count++;
	
	if (first_flag==true)
	{
		prev_count = curr_count;
		first_flag = false;
	}
	
	
	if (it != tow_data.end() && curr_count == prev_count+1)	//element was found 
	{	
	
		index = std::distance(abs_diffs.begin(), it);
			
		// write that closest GPS data and path data to a file 
		
		/*outFile << tow_data[index] << "; " <<  gps_data[index][0] << "; " << gps_data[index][1] << "; " << gps_data[index][2] << "; " << 
		bStdDev_data[index][0] << "; " << bStdDev_data[index][0] << "; " << bStdDev_data[index][0] << "; " << path->poses[curr_count-1].pose.position.x << "; " <<
		path->poses[curr_count-1].pose.position.y << "; " << path->poses[curr_count-1].pose.position.z << "\n";*/
		
		outFile << std::setprecision(15) << tow_data[index] << "; " <<  
		std::setprecision(15) <<bStdDev_data[index][0] << "; " << 
		std::setprecision(15) <<bStdDev_data[index][1] << "; " << 
		std::setprecision(15) <<bStdDev_data[index][2] << "; " << 
		path->poses[curr_count-1].pose.position.x << "; " <<
		path->poses[curr_count-1].pose.position.y << "; " << 
		path->poses[curr_count-1].pose.position.z << "\n";
		
	}
		
	
	prev_count = curr_count;
	std::cout << curr_count << std::endl;
	curr_count = 0;
	
}

void readPADcsv()
{
      std::cout << "Enter PADsolution csv file" << std::endl;
      //std::string file_name="/media/nagaraj/SSD_2TB/securepnt_data_collection/isp_recording_2021-10-11-13-01-19_2.bag.velodyne_packets_rtk.txt";
      //std::string file_name="/media/nagaraj/SSD_2TB/securepnt_data_collection/isp_recording_2021-10-11-13-01-19_2.bag.velodyne_packets_rtk_accuracy_bStdDev.txt";
      //std::string file_name="/media/nagaraj/SSD_2TB/securepnt_data_collection/isp_recording_2021-10-11-12-31-20_1.bag.velodyne_packets_rtk_accuracy.txt";
      //std::string file_name="/media/nagaraj/SSD_2TB/securepnt_data_collection/isp_recording_2021-10-11-12-31-20_1.bag.velodyne_packets_rtk_accuracy_bStdDev.txt";
      //std::string file_name="/media/nagaraj/SSD_2TB/securepnt_data_collection/PAD_solution.bin.csv";
      std::string file_name="/media/nagaraj/SSD_2TB/securepnt_data_collection/isp_recording_2021-10-11-13-01-19_2.bag.velodyne_packets_rtk_accuracy_bStdDev_baseline.txt";
      //std::cin >> file_name;
      io::CSVReader<8> in(file_name);
      std::cout << "Reading attitude from csv : " << file_name << std::endl;
      in.read_header(io::ignore_extra_column, "tow", "lat", "lon", "alt", "accuracy", "b0", "b1", "b2");
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
        
        std::cout << tow << " " << std::setprecision(15) << std::stold(tow) << std::endl;
          
        tow_data.emplace_back(std::stold(tow));				//assuming TOW info from the PAD solution csv is always valid and available

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
      	  
      	  bStdDev_data.emplace_back(Eigen::Vector3f({	std::stof(bStdDeviation1), 
      	  											 	std::stof(bStdDeviation0), 
      	  											 	std::stof(bStdDeviation2)
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
	outFile.open ("/media/nagaraj/SSD_2TB/securepnt_data_collection/lio_sam_vs_rtk_results.txt");
	outFile << "tow; b0; b1; b2; x; y; z\n";
	//printPADcsv();
	ros::init(argc, argv, "gps_publisher");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	ROS_INFO("Read PADSolution file and Publish GPS topic: /gps/fix ");

	//sub_lidar=nh.subscribe("/velodyne_points", 1000, lidarCallback);
	//pub_gps = nh.advertise<sensor_msgs::NavSatFix>("/gps/fix", 1000);	
	sub_path=nh.subscribe("/lio_sam/mapping/path", 1000, pathCallback);
	

  	
	
	ros::spin();
	
	return 0;		
}



