#include "ros/ros.h"
#include "yeti_snowplow/wheel_speeds.h"

#include <math.h>
#include <serial/serial.h>
#include <serial/utils/serial_listener.h>
#include <sstream>
#include <string>

using std::string;
using std::stringstream;
using serial::Serial;
using serial::utils::SerialListener;
using serial::utils::BufferedFilterPtr;

//serial code based on https://github.com/wjwwood/ax2550/
std::string port;
serial::Serial *serialPort;
serial::utils::SerialListener serialListener;
bool roboteqIsConnected = false;
double leftSpeed = 0, rightSpeed = 0;

void driveModeCallback(const yeti_snowplow::wheel_speeds::ConstPtr& msg){	
	/* This fires every time a button is pressed/released
	and when an axis changes (even if it doesn't leave the
	deadzone). */

	leftSpeed = msg->left;
	rightSpeed = msg->right;
	
	ROS_INFO("Roboteq: left wheel=%f right wheel=%f", leftSpeed, rightSpeed);
}

void disconnect(){
	roboteqIsConnected = false;
	if(serialListener.isListening()){
		serialListener.stopListening();
	}
	if(serialPort != NULL) {
		delete serialPort;
		serialPort = NULL;
	}
}

void connect(){
	if(roboteqIsConnected){
		ROS_WARN("Roboteq already connected");
		return;
	}
	if(port.empty()){
		ROS_ERROR("Serial port name is empty.");
		return;
	}

	disconnect();

	serialPort = new Serial();
	serialPort->setPort(port);
 	serialPort->setBaudrate(9600);
  	serialPort->setParity(serial::parity_even);
  	serialPort->setStopbits(serial::stopbits_one);
  	serialPort->setBytesize(serial::sevenbits);
	serial::Timeout to = serial::Timeout::simpleTimeout(10);
	serialPort->setTimeout(to);

	serialPort->open();

	serialListener.setChunkSize(2);
	serialListener.startListening(*serialPort);

	roboteqIsConnected = true;
}

inline string stringFormat(const string &fmt, ...) {
  int size = 100;
  string str;
  va_list ap;
  while (1) {
	str.resize(size);
	va_start(ap, fmt);
	int n = vsnprintf((char *)str.c_str(), size, fmt.c_str(), ap);
	va_end(ap);
	if (n > -1 && n < size) {
	  str.resize(n);
	  return str;
	}
	if (n > -1) {
	  size = n + 1;
	} else {
	  size *= 2;
	}
  }
  return str;
}

inline bool isPlusOrMinus(const string &token) {
	if (token.find_first_of("+-") != string::npos) {
		return true;
	}
	return false;
}

bool sendCommand(string command){
	BufferedFilterPtr echoFilter = serialListener.createBufferedFilter(SerialListener::exactly(command));
	serialPort->write(command+"\r");
	if (echoFilter->wait(50).empty()) {
		ROS_ERROR("Failed to receive an echo from the Roboteq.");
		return false;
	}
	BufferedFilterPtr plusMinusFilter = serialListener.createBufferedFilter(isPlusOrMinus);
	string result = plusMinusFilter->wait(100);
	if(result != "+"){
		if(result == "-"){
			ROS_ERROR("The Roboteq rejected the command.");
			return false;
		}
		ROS_ERROR("Did not receive a confirmation or rejection from the Roboteq.");
		return false;
	}
	return true;
}

unsigned char constrainSpeed(double speed){
	unsigned char temp = fabs(speed);
	if(temp > 127){
		temp = 127;
	}
	return temp;
}

void move(){
	if(!roboteqIsConnected){
		ROS_WARN("The Roboteq needs to be connected to move.");
		return;
	}

	if(rightSpeed < 0){
		sendCommand(stringFormat("!a%.2X", constrainSpeed(rightSpeed)));
	}
	else{
		sendCommand(stringFormat("!A%.2X", constrainSpeed(rightSpeed)));
	}

	if(leftSpeed < 0){
		sendCommand(stringFormat("!b%.2X", constrainSpeed(leftSpeed)));
	}
	else{
		sendCommand(stringFormat("!B%.2X", constrainSpeed(leftSpeed)));
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "roboteq");

	ros::NodeHandle n;

	// Serial port parameter
	n.param("serial_port", port, std::string("/dev/ttyUSB0"));

	ros::Subscriber driveModeSub = n.subscribe("wheelSpeeds", 5, driveModeCallback);

	ros::Rate loopRate(100); //Hz
	while(ros::ok()) {
		ROS_INFO("Connecting Roboteq on port %s", port.c_str());
		try {
			connect();
		} catch(std::exception &e) {
			ROS_ERROR("Failed to connect to the Roboteq: %s", e.what());
			disconnect();
		}

		while(roboteqIsConnected && ros::ok()){
			ros::spinOnce();
			move();
			loopRate.sleep();
		}

		if(!ros::ok()) break;
		ROS_INFO("Will try to reconnect to the Roboteq in 5 seconds.");
		for (int i = 0; i < 100; ++i) {
			ros::Duration(5.0/100.0).sleep();
			if (!ros::ok()) break;
		}
	}

	// ros::spin();
	
	return 0;
}