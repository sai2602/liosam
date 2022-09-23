/*	date: 10. Feb 2022
 *	comment: see the original unchanged file at /home/nagaraj/Development/tools/publish_gps_topic/publish_gps.cpp
 */



#include <iostream>
#include <fstream>
#include "csv.h"
#include <eigen3/Eigen/Core>
#include <iomanip>


struct double3

{

double x;
double y;
double z;
};

struct double4
{
double x;
double y;
double z;
double w;
};

struct state_t
{
uint8_t att_state;
uint8_t rtk_state;

};

std::vector<Eigen::Vector4f> gps_data;
std::vector<double3> ECEF_data;
std::vector<float> tow_data;
std::vector<uint32_t> resCode_data;

std::ofstream outFile;

// "Publish GPS data inside lidar callback"

float clockToGPSsecs(int& secs, int& nsecs)
{
	int current_leap_seconds = 18;
	int secondsSince1980_UTC = secs - 315964800;
	int secondsGPS = secondsSince1980_UTC + current_leap_seconds;
	int secondsPerWeek = 3600 * 24 * 7 ;
	int secondsOfWeek = secondsGPS % secondsPerWeek ;
	float t = secondsOfWeek + nsecs*1e-9;
	return t;
}

float clockToGPSsecs(int&& secs, int&& nsecs)
{
	int current_leap_seconds = 18;
	int secondsSince1980_UTC = secs - 315964800;
	int secondsGPS = secondsSince1980_UTC + current_leap_seconds;
	int secondsPerWeek = 3600 * 24 * 7 ;
	int secondsOfWeek = secondsGPS % secondsPerWeek ;
	float t = secondsOfWeek + nsecs*1e-9;
	return t;
}



void decodeResCode(uint32_t& resCode, state_t& s)
{
// attitude- and rtk-filter state    //  00 --> No solution    //  01 --> Least-Squares solution    //  10 --> Float solution    //  11 --> Fixed solution    
	uint8_t att_state = 0;
    uint8_t rtk_state = 0;
        // extract bit 9 to 10 from resCode (state-definition for the Attitude Kalman-Filter)    
        
    if( resCode & ( 0x01 << 9 ) ){
        att_state += 1;
    }
    if( resCode & ( 0x01 << 10 ) ){
        att_state += 2;
    }
    // extract bit 11 to 12 from resCode (state-definition for the RTK-Position Kalman-Filter)    
    
    if( resCode & ( 0x01 << 11 ) ){
        rtk_state += 1;
    }
    if( resCode & ( 0x01 << 12 ) ){
        rtk_state += 2;
    }
    
    s.att_state = att_state;
    s.rtk_state = rtk_state;
}

void pathCallback (std::string& lio_sam_csv_file)
{

	// convert message header to TOW
	
	//decode the string and get pose, and timestamp
	
	io::CSVReader<9> in(lio_sam_csv_file);
	
	std::cout << "Reading LIO-SAM path poses ... " << std::endl;
	std::string secs, nsecs, x_pos, y_pos, z_pos, x_ori, y_ori, z_ori, w_ori;
	in.read_header(io::ignore_extra_column, "secs", "nsecs", "x_pos", "y_pos", "z_pos", "x_ori", "y_ori", "z_ori", "w_ori");
	while(in.read_row(secs, nsecs, x_pos, y_pos, z_pos, x_ori, y_ori, z_ori, w_ori))
    {
	
		float path_tow_secs = clockToGPSsecs(std::stoi(secs), std::stoi(nsecs));
	
		// find closest match in GPS data 
	
		//2. on global storage, search for the data to be published using TOW obtained in previous step 

		std::vector<float> abs_diffs;

		for(auto& element : tow_data)
			abs_diffs.emplace_back(std::abs(element-path_tow_secs));

		auto it = std::min_element(abs_diffs.begin(), abs_diffs.end());
		int index = -1 ;
		state_t s;
	
	
		if (it != tow_data.end())	//element was found 
		{	
	
			index = std::distance(abs_diffs.begin(), it);
			
			// write that closest GPS data and path data to a file 
		
			/*outFile << tow_data[index] << "; " <<  gps_data[index][0] << "; " << gps_data[index][1] << "; " << gps_data[index][2] << "; " << 
			bStdDev_data[index][0] << "; " << bStdDev_data[index][0] << "; " << bStdDev_data[index][0] << "; " << path->poses[curr_count-1].pose.position.x << "; " <<
			path->poses[curr_count-1].pose.position.y << "; " << path->poses[curr_count-1].pose.position.z << "\n";*/
		
			outFile << std::setprecision(9) << tow_data[index] << ", " ;
			
			decodeResCode(resCode_data[index], s); 
			
			outFile << int(s.att_state) << ", " ;
			outFile << int(s.rtk_state) << ", " ;
		
			outFile<<std::setprecision(10) <<ECEF_data[index].x << ", " ; 
		
			outFile<<std::setprecision(10) <<ECEF_data[index].y << ", " ; 
		
			outFile<<std::setprecision(10) <<ECEF_data[index].z << ", " ;
			
			outFile << std::setprecision(9) << path_tow_secs << ", " ;
		
			outFile << std::setprecision(5) <<
			x_pos << ", " <<
			y_pos << ", " << 
			z_pos << "\n";
		
		}
		
	}
	
}

void readPADcsv(std::string& file_name)
{
      std::cout << "Enter PADsolution csv file" << std::endl;
      //std::string file_name="/media/nagaraj/SSD_2TB/securepnt_data_collection/isp_recording_2021-10-11-13-01-19_2.bag.velodyne_packets_rtk.txt";
      //std::string file_name="/media/nagaraj/SSD_2TB/securepnt_data_collection/isp_recording_2021-10-11-13-01-19_2.bag.velodyne_packets_rtk_accuracy_bStdDev.txt";
      //std::string file_name="/media/nagaraj/SSD_2TB/securepnt_data_collection/isp_recording_2021-10-11-12-31-20_1.bag.velodyne_packets_rtk_accuracy.txt";
      //std::string file_name="/media/nagaraj/SSD_2TB/securepnt_data_collection/isp_recording_2021-10-11-12-31-20_1.bag.velodyne_packets_rtk_accuracy_bStdDev.txt";
      //std::string file_name="/media/nagaraj/SSD_2TB/securepnt_data_collection/PAD_solution.bin.csv";
      //std::string file_name="/media/nagaraj/SSD_2TB/securepnt_data_collection/isp_recording_2021-10-11-13-01-19_2.bag.velodyne_packets_rtk_accuracy_bStdDev_baseline.txt";
      //std::string file_name="/media/nagaraj/SSD_2TB/securepnt_data_collection/isp_recording_2021-10-11-13-01-19_2.bag.velodyne_packets_rtk_accuracy_ECEF_baseline.txt";
      //std::string file_name="/media/nagaraj/SSD_2TB/indoor_outdoor_parkingspace/isp_recording_2022-02-24-13-42-36_0.bag.velodyne_packets_rtk_accuracy_ECEF_baseline.txt";
      //std::cin >> file_name;
      io::CSVReader<9> in(file_name);
      std::cout << "Reading attitude from csv : " << file_name << std::endl;
      in.read_header(io::ignore_extra_column, "tow", "lat", "lon", "alt", "accuracy", "ECEFX", "ECEFY", "ECEFZ", "code");
      std::string tow;
      std::string lat;
      std::string lon;
      std::string alt;
      std::string accuracy;
      std::string ECEFX;
      std::string ECEFY;
      std::string ECEFZ;
      std::string code;
      //unsigned int counter = 0; 

      while(in.read_row(tow, lat, lon, alt, accuracy, ECEFX, ECEFY, ECEFZ, code))
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
		if (ECEFX=="nan" || ECEFX=="NaN")
		  ECEFX= std::nan(ECEFX.c_str());		
		if (ECEFY=="nan" || ECEFY=="NaN")
		  ECEFY= std::nan(ECEFY.c_str());		
		if (ECEFZ=="nan" || ECEFZ=="NaN")
		  ECEFZ= std::nan(ECEFZ.c_str());
		  
		if (code=="nan" || code=="NaN")
		  code= std::nan(code.c_str());
        //std::cout << "tow after reading from file : " << tow << std::endl;
        
        
        
        std::cout << tow << " " << std::setprecision(15) << std::stold(tow) << std::endl;
          
        tow_data.emplace_back(std::stold(tow));				//assuming TOW info from the PAD solution csv is always valid and available
        resCode_data.emplace_back(uint32_t(std::stoi(code)));			//assuming resCode is a number always

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
      	  
      	  ECEF_data.emplace_back(double3{	std::stold(ECEFX), 
      	  											 	std::stold(ECEFY), 
      	  											 	std::stold(ECEFZ)
      	  											 });
          
          //std::cout << "tow std::stof(tow) : " << std::stof(tow) << std::setprecision(10) << std::endl;
        } 
        catch (std::invalid_argument const& ex1) 
        {
          ECEF_data.emplace_back(double3{0.0, 0.0, 0.0});        
        }
       
        //counter+=1;     
      
      }

	 if(gps_data.size()!=tow_data.size() || ECEF_data.size()!=tow_data.size() || resCode_data.size()!=tow_data.size())
	 {
		std::cout << "ERROR in data reading from CSV! gps_data.size()!=tow_data.size() or ECEF_data.size()!=tow_data.size() or resCode_data.size()!=tow_data.size()" << std::endl; 
		return; 
	 }
     std::cout << "Done. " << std::endl;
     std::cout << "length of attitude data : " << gps_data.size() << std::endl;


    //std::ofstream output_file("tow_data.txt");
    //std::ostream_iterator<float> output_iterator(output_file, "\n");
    //for(auto h: tow_data )
	//	output_file << h << std::setprecision(10) << std::endl;
}



int main(int argc, char** argv)
{
	//argv[1] is inputfile containing interpolated pad solution
	//argv[2] is inputfile containing lio-sam results as csv file
	//argv[3] is outputFile containing rtk and lio-sam results at same timestamps
	std::cout << "argv[1] is inputfile containing interpolated pad solution\nargv[2] is inputfile containing lio-sam results as csv file\nargv[3] is outputFile containing rtk and lio-sam results at same timestamps\n" ;
	
	if(argc!=4)
		return -1;
	
	std::string infile_PAD{argv[1]};
	std::string infile_lioSam{argv[2]};
	readPADcsv(infile_PAD);
	//outFile.open ("/media/nagaraj/SSD_2TB/indoor_outdoor_parkingspace/hoffner_round3_lio_sam_vs_rtk_ECEF.csv");
	outFile.open (argv[3]);
	outFile << "tow, att_state, rtk_state, ECEFX, ECEFY, ECEFZ, lidar_tow, lidar_x, lidar_y, lidar_z\n";
	pathCallback(infile_lioSam);
	
	return 0;		
}



