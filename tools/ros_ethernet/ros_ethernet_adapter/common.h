/*
 * Common functions used in the client and server application.
 */

/*
 * File:    common.h
 * Author:  Robert Bensch, ANavS GmbH
 *
 * History: 2020/01/23 - Initial file creation
 *
 * Created on January 23, 2020
 */

#ifndef COMMON_H
#define COMMON_H

#include "common_ros.h"

#include <limits>
#include <regex>

static const int customer_code = 5;    // 0: none (all features available)
                                       // 1: ROS wrapper for ANavS PAD solution: mode 'padsolution2ros' only
                                       // 2: STIHL/AGT LiDAR project
                                       // 3: IIT (2d LiDAR SLAM): no wheel odometry
                                       // 4: ESC-Aerospace (client: hide debug_* modes; server: only rosroviovio2pad* modes (hide lidar, odo modes)
                                       // 5: client modes: 'padsolution2ros' and 'padsensordata2ros' (Uni Heilbronn, Sony)

static int mode = 0;    // client/ server mode

// math
const double toRad = M_PI/180.f;
const double toDeg = 180.f/M_PI;

// console output
const std::vector<std::string> cout_indent{std::string(3,' '), std::string(6,' ')};

const std::string anavs_config_filename("configs/ANAVS.conf");

// Wheel odometry parameters
//  are read from global ANAVS config file ('PAD-wheel_radius', 'PAD-gear_ratio', not read yet, not used: 'PAD-wheel_distance')
static double gear_ratio = 87.88;
static double radius_wheel = 0.12;      // in meter
static double encoder_resolution = 100;

const std::string cmd_param_prefix("--");

// calculate checksum with fletcher's algorithm
static int VerifyChecksum(unsigned char *buff, size_t len)
{
//    std::cout << "VerifyChecksum: buff address: " << std::hex << (void*)buff << ", len:" << std::dec << +len << std::endl;

    fletcher_checksum_t chk;
    chk.chkA = 0;
    chk.chkB = 0;

    for(size_t i = 2; i < len-2; i++) {
        chk.chkA += buff[i];
        chk.chkB += chk.chkA;
    }

//    std::cout << "calculated chksum: " << std::dec << +chk.chkA << ", " << +chk.chkB << std::endl;
//    std::cout << "read chksum: " << std::dec << +buff[len-2] << ", " << +buff[len-1] << std::endl;
//    std::cout << "return " << (chk.chkA==buff[len-2] && chk.chkB==buff[len-1]) << std::endl;
//
//    for (size_t j = 1; j < len+1; j++) {
//        std::cout <<  "buff[" << len-j << "]=" << std::dec << +buff[len-j] << " ";
//    }
//    std::cout << std::endl;

    return chk.chkA==buff[len-2] && chk.chkB==buff[len-1];
}

// calculate checksum with fletcher's algorithm
static void CalculateChecksum(unsigned char *buff, size_t len, fletcher_checksum_t& chk)
{
    chk.chkA = 0;
    chk.chkB = 0;

    for(size_t i = 2; i < len-2; i++) {
        chk.chkA += buff[i];
        chk.chkB += chk.chkA;
    }
}

// trims string from the right, by removing trailing spaces
// see: https://www.techiedelight.com/trim-string-cpp-remove-leading-trailing-spaces/
std::string rtrim(const std::string& str) {
    return std::regex_replace(str, std::regex("\\s+$"), std::string(""));
}

// trims string from the left, by removing leading spaces
// see: https://www.techiedelight.com/trim-string-cpp-remove-leading-trailing-spaces/
std::string ltrim(const std::string& str) {
    return std::regex_replace(str, std::regex("^\\s+"), std::string(""));
}

// get folder/file/executable from a file path
// see: https://stackoverflow.com/questions/40171586/c-regex-get-folder-from-a-file-path
std::string GetLastItemFromPath(const std::string& str) {
    std::string str_rep(str);
    auto last_slash = str_rep.find_last_of('/');
    return str_rep.replace(str_rep.begin(), str_rep.begin() + last_slash + 1 , "");
}

/*
 * functions for adding cmdline parsing capabilities
 *
 * source: https://stackoverflow.com/questions/865668/how-to-parse-command-line-arguments-in-c
 *
 * usage example:
 *
    int main(int argc, char * argv[])
    {
        if(CmdOptionExists(argv, argv+argc, "-h"))
        {
            // Do stuff
        }

        char * filename = GetCmdOption(argv, argv + argc, "-f");

        if (filename)
        {
            // Do interesting things
            // ...
        }

        return 0;
    }
 */

 // returns char-pointer to found option value string
 //     if option string is empty (no argument following), return 0 (null pointer)
char* GetCmdOption(char ** begin, char ** end, const std::string& option)
{
    char ** itr = std::find(begin, end, option);
    if (itr != end && ++itr != end)
    {
        return *itr;
    }
    return 0;
}

// returns found option value (string) in argument 'option_value'
//      if no argument is following, or following argument is a parameter declaration,
//      the value 'option_value' is unchanged
void GetCmdOption(char ** begin, char ** end, const std::string& option, std::string& option_value)
{
    //std::cout << "GetCmdOption: " << option << std::endl;

    char ** itr = std::find(begin, end, option);
    if (itr != end && ++itr != end)
    {
        std::string val_str( *itr);

        std::size_t find_param = val_str.find(cmd_param_prefix);
//        std::cout << "find_param = " << find_param << std::endl;
//        std::cout << "npos = " << (int)(find_param==std::string::npos) << std::endl;
        // if argument is not a parameter declaration, ie. starting with '--'
        //  assume it is a value and set value
        if (find_param!=0 || find_param==std::string::npos) {

            option_value = val_str;

        }

    }

}

// returns found option value (int) in argument 'option_value'
//      if no argument is following, or following argument is a parameter declaration,
//      the value 'option_value' is unchanged
void GetCmdOption(char ** begin, char ** end, const std::string& option, int& option_value)
{
    char ** itr = std::find(begin, end, option);
    if (itr != end && ++itr != end)
    {
            std::string val_str( *itr);

            std::size_t find_param = val_str.find(cmd_param_prefix);
            // if argument is not a parameter declaration, ie. starting with '--'
            //  assume it is a value and set value
            if (find_param!=0 || find_param==std::string::npos) {

                try {

                    option_value = std::stoi( val_str);

                } catch (std::invalid_argument const &e) {
                    std::cout << "Bad input: std::invalid_argument thrown" << std::endl;
                } catch (std::out_of_range const &e) {
                    std::cout << "Integer overflow: std::out_of_range thrown" << std::endl;
                }

            }

    }

}

// returns found option value (double) in argument 'option_value'
//      if no argument is following, or following argument is a parameter declaration,
//      the value 'option_value' is unchanged
void GetCmdOption(char ** begin, char ** end, const std::string& option, double& option_value)
{
    char ** itr = std::find(begin, end, option);
    if (itr != end && ++itr != end)
    {
            std::string val_str( *itr);

            std::size_t find_param = val_str.find(cmd_param_prefix);
            // if argument is not a parameter declaration, ie. starting with '--'
            //  assume it is a value and set value
            if (find_param!=0 || find_param==std::string::npos) {

                try {

                    option_value = std::stod( val_str);

                } catch (std::invalid_argument const &e) {
                    std::cout << "Bad input: std::invalid_argument thrown" << std::endl;
                } catch (std::out_of_range const &e) {
                    std::cout << "Integer overflow: std::out_of_range thrown" << std::endl;
                }

            }

    }

}

bool CmdOptionExists(char** begin, char** end, const std::string& option)
{
    return std::find(begin, end, option) != end;
}

// parse anavs configuration file line for parameter name, if found
//  returns parameter value as string and returns true, otherwise returns false
bool GetParamStrValFromConfigLine(const std::string& line, const std::string& param_name, std::string& param_val) {

    // find parameter
    const size_t param_found = line.find(param_name.c_str(), 0);
    if (param_found != std::string::npos) {

//        std::cout << "found parameter name: " << param_name << std::endl;

        // find '=' character
        size_t equal_sign_found = line.find("=", param_found+1);
        if (equal_sign_found != std::string::npos) {

//            std::cout << "found '=' " << std::endl;

            // find potential comment character '#'
            const size_t comment_found = line.find("#", equal_sign_found+1);
            if (comment_found != std::string::npos) {
//                std::cout << "comment found '#' " << std::endl;
                // comment '#' at line end
                // get trimmed string between '=' and '#' characters
                param_val = line.substr(equal_sign_found + 1, comment_found - equal_sign_found - 1);
            } else {
                // no comment at line end
                // get trimmed string after '=' to the end of line
                param_val = line.substr(equal_sign_found + 1);
            }

            param_val = ltrim(param_val);
            param_val = rtrim(param_val);
//            std::cout << param_name << " = '" << param_val << "'" << std::endl;

            return true;
        }
    }

    return false;
}


// parse anavs configuration file line for parameter name, if found
//  returns parameter value as double and returns true, otherwise returns false
//
// note: only suitable for numeric values
bool GetParamDblValFromConfigLine(const std::string& line, const std::string& param_name, double& param_val) {

    std::string param_val_str;

    if (GetParamStrValFromConfigLine(line, param_name, param_val_str)) {

        std::stringstream ss(param_val_str);
        ss >> param_val;
//        std::cout << param_name << " = " << param_val << std::endl;

        return true;
    }

    return false;
}

// read anavs configuration file and parses for specific parameters, if found
//  returns parameter values by reference
//double& gear_ratio, double& radius_wheel
bool ReadAnavsConfigFile(const std::string& filename, const std::vector< std::string >& param_names,
                         std::vector<double>& param_values)
{
    const size_t num_default_params = param_values.size();

    if (param_values.size() > 0 && param_values.size() != param_names.size()) {
        std::cout << "[WARN] ReadAnavsConfigFile: param_values.size() > 0 && param_values.size() != param_names.size()" << std::endl;
    }

    if (param_values.size() < param_names.size()) {
        param_values.resize(param_names.size());
    }
    std::vector<bool> param_found(param_names.size(), false);

    ifstream infile(filename.c_str());

    // open file and
    if (infile.is_open()) {

        size_t line_idx = 0;
        std::string line;

        // step through lines
        while( getline(infile, line)) {

            line_idx++;

            // skip comment lines starting with '#'
            line = ltrim(line);
            line = rtrim(line);
            if (line.find("#") == 0) {
//                std::cout << "Skip comment line #." << std::endl;
                continue;
            }

            // step through parameter list
            for (size_t i = 0; i < param_names.size(); ++i) {

                double val;
                if (GetParamDblValFromConfigLine(line, param_names[i], val)) {
//                    std::cout << "\t" << param_names[i] << " = " << val << " (" << line_idx << ")" << std::endl;
                    param_values[i] = val;  // update value
                    param_found[i] = true;
                }
            }
        }

        for (size_t i = 0; i < param_names.size(); ++i) {
            if (!param_found[i]) {
                //if (i <= num_default_params - 1) {
                if (i+1 <= num_default_params) {
                    std::cout << "[WARN] Parameter '"<< param_names[i] << "' not found. Default value used: " << param_values[i] << std::endl;
                } else {
                    std::cout << "[ERROR] Parameter '" << param_names[i] << "' not found. No default value specified." << std::endl;
                    std::cout << "Please add parameter to configuration file!" << std::endl;
                    return false;
                }
            }
        }
        infile.close();
        return true;

    } else {
        return false;
    }
}

// TODO: Check whether it is necessary to check:
//  std::numeric_limits::has_quiet_NaN == true
//  see: https://stackoverflow.com/questions/16691207/c-c-nan-constant-literal
template<typename T>
void setNaN(T& val)
{
    val = std::numeric_limits<T>::quiet_NaN();
}

#endif // COMMON_H
