/*
 * Debug helper functions used in the client and server application.
 */

/*
 * File:    debug_helper.h
 * Author:  Robert Bensch, ANavS GmbH
 *
 * History: 2022/03/11 - Initial file creation
 *
 * Created on March 11, 2022
 */

#ifndef DEBUG_HELPER_H
#define DEBUG_HELPER_H

#include <fstream>      // std::ofstream
#include <string>

void WriteImagePGM( const unsigned char *img, int width, int height, std::string filename)
{
    // output file stream
    std::ofstream fout (filename.c_str());

    if (!fout.is_open())
    {
     std::cout << "Can't open output file"  << filename << std::endl;
     exit(1);
    }

    // write the header
    fout << "P5\n" << width << " " << height << " 255\n";

    // write the data
    fout.write((char *)img, width*height*sizeof(img[0]));

    // close the stream
    fout.close();

}

void WriteImagePPM( const unsigned char *img, int width, int height, std::string filename)
{
    // output file stream
    std::ofstream fout (filename.c_str());

    if (!fout.is_open())
    {
     std::cout << "Can't open output file"  << filename << std::endl;
     exit(1);
    }

    // write the header
    fout << "P6\n" << width << " " << height << " 255\n";

    // write the data
    fout.write((char *)img, width*height*sizeof(img[0])*3);

    // close the stream
    fout.close();

}

#endif // DEBUG_HELPER_H
