#!/bin/bash

ROS_VERSION=melodic

ROS_INCLUDE_DIR=/opt/ros/${ROS_VERSION}/include
ROS_LIB_DIR=/opt/ros/${ROS_VERSION}/lib

BASEPATH=`pwd`

#
# Build server
#
echo "Building server..."

rm -r obj/

mkdir -p obj/Release/sockets

echo "   Compiling..."

g++ -Wall -std=c++11 -fexceptions -O3 -I${ROS_INCLUDE_DIR} -c ${BASEPATH}/main.cpp -o obj/Release/main.o
g++ -Wall -std=c++11 -fexceptions -O3 -I${ROS_INCLUDE_DIR} -c ${BASEPATH}/sockets/ClientSocket.cpp -o obj/Release/sockets/ClientSocket.o
g++ -Wall -std=c++11 -fexceptions -O3 -I${ROS_INCLUDE_DIR} -c ${BASEPATH}/sockets/ClientSocketWrapper.cpp -o obj/Release/sockets/ClientSocketWrapper.o
g++ -Wall -std=c++11 -fexceptions -O3 -I${ROS_INCLUDE_DIR} -c ${BASEPATH}/sockets/ServerSocket.cpp -o obj/Release/sockets/ServerSocket.o
g++ -Wall -std=c++11 -fexceptions -O3 -I${ROS_INCLUDE_DIR} -c ${BASEPATH}/sockets/ServerSocketWrapper.cpp -o obj/Release/sockets/ServerSocketWrapper.o
g++ -Wall -std=c++11 -fexceptions -O3 -I${ROS_INCLUDE_DIR} -c ${BASEPATH}/sockets/Socket.cpp -o obj/Release/sockets/Socket.o
g++ -Wall -std=c++11 -fexceptions -O3 -I${ROS_INCLUDE_DIR} -c ${BASEPATH}/sockets/SocketData.cpp -o obj/Release/sockets/SocketData.o
g++ -Wall -std=c++11 -fexceptions -O3 -I${ROS_INCLUDE_DIR} -c ${BASEPATH}/sockets/SocketDataWrapper.cpp -o obj/Release/sockets/SocketDataWrapper.o
gcc -Wall -fexceptions -O3 -I${ROS_INCLUDE_DIR} -c ${BASEPATH}/ublox_partial.c -o obj/Release/ublox_partial.o

echo "   Linking..."

mkdir -p bin/Release

g++ -L${ROS_LIB_DIR} -o bin/Release/anavs_ros_ethernet_server obj/Release/main.o obj/Release/sockets/ClientSocket.o obj/Release/sockets/ClientSocketWrapper.o obj/Release/sockets/ServerSocket.o obj/Release/sockets/ServerSocketWrapper.o obj/Release/sockets/Socket.o obj/Release/sockets/SocketData.o obj/Release/sockets/SocketDataWrapper.o obj/Release/ublox_partial.o  -s -Wl,-rpath,${ROS_LIB_DIR}  -lroscpp -lrosconsole -lrostime -lroscpp_serialization -lmessage_filters -lboost_system

echo "Done."


#
# Build client
#
cd client/

echo "Building client..."

rm -r obj/

mkdir -p obj/Release/sockets
mkdir -p obj/Release/decoder-cpp/src
mkdir -p obj/Release/client

echo "   Compiling..."

# submodule: decoder-cpp
g++ -Wall -std=c++14 -fexceptions -O3 -I${ROS_INCLUDE_DIR} -c ${BASEPATH}/decoder-cpp/src/Crc16Ccitt.cpp -o obj/Release/decoder-cpp/src/Crc16Ccitt.o
g++ -Wall -std=c++14 -fexceptions -O3 -I${ROS_INCLUDE_DIR} -c ${BASEPATH}/decoder-cpp/src/GnssParser.cpp -o obj/Release/decoder-cpp/src/GnssParser.o
g++ -Wall -std=c++14 -fexceptions -O3 -I${ROS_INCLUDE_DIR} -c ${BASEPATH}/decoder-cpp/src/LambdaAdapter.cpp -o obj/Release/decoder-cpp/src/LambdaAdapter.o
g++ -Wall -std=c++14 -fexceptions -O3 -I${ROS_INCLUDE_DIR} -c ${BASEPATH}/decoder-cpp/src/MultiProtocolParser.cpp -o obj/Release/decoder-cpp/src/MultiProtocolParser.o
g++ -Wall -std=c++14 -fexceptions -O3 -I${ROS_INCLUDE_DIR} -c ${BASEPATH}/decoder-cpp/src/NmeaEncoder.cpp -o obj/Release/decoder-cpp/src/NmeaEncoder.o
g++ -Wall -std=c++14 -fexceptions -O3 -I${ROS_INCLUDE_DIR} -c ${BASEPATH}/decoder-cpp/src/NmeaSubParser.cpp -o obj/Release/decoder-cpp/src/NmeaSubParser.o
g++ -Wall -std=c++14 -fexceptions -O3 -I${ROS_INCLUDE_DIR} -c ${BASEPATH}/decoder-cpp/src/SbfSubParser.cpp -o obj/Release/decoder-cpp/src/SbfSubParser.o
g++ -Wall -std=c++14 -fexceptions -O3 -I${ROS_INCLUDE_DIR} -c ${BASEPATH}/decoder-cpp/src/UbxPackets.cpp -o obj/Release/decoder-cpp/src/UbxPackets.o
g++ -Wall -std=c++14 -fexceptions -O3 -I${ROS_INCLUDE_DIR} -c ${BASEPATH}/decoder-cpp/src/UbxSubparser.cpp -o obj/Release/decoder-cpp/src/UbxSubparser.o

g++ -Wall -std=c++14 -fexceptions -O3 -I${ROS_INCLUDE_DIR} -c ${BASEPATH}/sockets/ClientSocket.cpp -o obj/Release/sockets/ClientSocket.o
g++ -Wall -std=c++14 -fexceptions -O3 -I${ROS_INCLUDE_DIR} -c ${BASEPATH}/sockets/ClientSocketWrapper.cpp -o obj/Release/sockets/ClientSocketWrapper.o
g++ -Wall -std=c++14 -fexceptions -O3 -I${ROS_INCLUDE_DIR} -c ${BASEPATH}/sockets/ServerSocket.cpp -o obj/Release/sockets/ServerSocket.o
g++ -Wall -std=c++14 -fexceptions -O3 -I${ROS_INCLUDE_DIR} -c ${BASEPATH}/sockets/ServerSocketWrapper.cpp -o obj/Release/sockets/ServerSocketWrapper.o
g++ -Wall -std=c++14 -fexceptions -O3 -I${ROS_INCLUDE_DIR} -c ${BASEPATH}/sockets/Socket.cpp -o obj/Release/sockets/Socket.o
g++ -Wall -std=c++14 -fexceptions -O3 -I${ROS_INCLUDE_DIR} -c ${BASEPATH}/sockets/SocketData.cpp -o obj/Release/sockets/SocketData.o
g++ -Wall -std=c++14 -fexceptions -O3 -I${ROS_INCLUDE_DIR} -c ${BASEPATH}/sockets/SocketDataWrapper.cpp -o obj/Release/sockets/SocketDataWrapper.o
g++ -Wall -std=c++14 -fexceptions -O3 -I${ROS_INCLUDE_DIR} -c ${BASEPATH}/client/main.cpp -o obj/Release/client/main.o


echo "   Linking..."

mkdir -p bin/Release

g++ -L${ROS_LIB_DIR} -o bin/Release/anavs_ros_ethernet_client obj/Release/decoder-cpp/src/Crc16Ccitt.o obj/Release/decoder-cpp/src/GnssParser.o obj/Release/decoder-cpp/src/LambdaAdapter.o obj/Release/decoder-cpp/src/MultiProtocolParser.o obj/Release/decoder-cpp/src/NmeaEncoder.o obj/Release/decoder-cpp/src/NmeaSubParser.o obj/Release/decoder-cpp/src/SbfSubParser.o obj/Release/decoder-cpp/src/UbxPackets.o obj/Release/decoder-cpp/src/UbxSubparser.o obj/Release/sockets/ClientSocket.o obj/Release/sockets/ClientSocketWrapper.o obj/Release/sockets/ServerSocket.o obj/Release/sockets/ServerSocketWrapper.o obj/Release/sockets/Socket.o obj/Release/sockets/SocketData.o obj/Release/sockets/SocketDataWrapper.o obj/Release/client/main.o  -s -Wl,-rpath,${ROS_LIB_DIR}  -lroscpp -lrosconsole -lrostime -lroscpp_serialization -lmessage_filters -lboost_system -ltf2_ros -ltf2

echo "Done."
