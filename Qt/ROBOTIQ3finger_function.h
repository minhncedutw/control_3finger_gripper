/*
*   File:   ROBOTIQ_function.h
*	This Class contains ROBOTIQ 3-Finger gripper library.
*   Using library function to drive gripper. Input type will be int or double.
*	The variable is the parameter you want to set to SMC gripper,like position, velocity, force and so on.
*	Before use this class, please include all Protocol libraries and set the boost_1_54 library.
*   Boost library: http://blog.monkeypotion.net/gameprog/note/first-touch-of-boost-cpp-libraries
*
*	Author: Che-Liang Li
*	Version: v1.0
*	History:
*			2014/08/20 set up all the functions      
*													    
*
*/
//#include "stdafx.h"
#include <string>
#include <iostream>
#include <vector>
#include <conio.h>
#include <boost/thread.hpp>
#include <boost/crc.hpp>      // for boost::crc_basic, boost::crc_optimal
#include <boost/cstdint.hpp>  // for boost::uint16_t
#include "MyRobot_AsyncSerial.h"
#include "MyRobot_Protocol.h"
#include "MyRobot_ProtocolDef.h"


using namespace std;
typedef unsigned char uchar;

#ifndef ROBOTIQ3FINGER_FUNCTION_H   
#define	ROBOTIQ3FINGER_FUNCTION_H

#define BASIC_MODE 0
#define PINCH_MODE 1
#define WIDE_MODE 2
#define SCISSOR_MODE 3

class ROBOTIQ3finger_function
{ 
public:	
    //constructor
	ROBOTIQ3finger_function( std::string com_port );
	//deconstructor
	~ROBOTIQ3finger_function();

public:
	MyRobot_Protocol* myProtocol;
	

public:
	//send data buffer
	std::vector<uchar> myData;

    //Activative gripper(serve on)
	void ROBOTIQ3finger_active_on(int);
	//Reset gripper(serve off)
	void ROBOTIQ3finger_active_off();

	//read the status about the finger a,b,c
	void ROBOTIQ3finger_action_status_read(double &,double &,double &);
	void ROBOTIQ3finger_a_action_status_read(double &,double &,double &);
	void ROBOTIQ3finger_b_action_status_read(double &,double &,double &);
	void ROBOTIQ3finger_c_action_status_read(double &,double &,double &);
	void ROBOTIQ3finger_s_action_status_read(double &,double &,double &);
	
	void ROBOTIQ3finger_gripper_status_read(int&,int&,int&,int&,int&);
	void ROBOTIQ3finger_object_status_read(int&,int&,int&,int&);
	void ROBOTIQ3finger_fault_read(int&);
	
	//set the commands about the finger a,b,c
	void ROBOTIQ3finger_set_pos(double &);
	void ROBOTIQ3finger_set_all(double &,double &, double &);
	void ROBOTIQ3finger_set_fingera_pos(double &);
	void ROBOTIQ3finger_set_fingera_all(double &,double &, double &);
	void ROBOTIQ3finger_set_fingerb_pos(double &);
	void ROBOTIQ3finger_set_fingerb_all(double &,double &, double &);
	void ROBOTIQ3finger_set_fingerc_pos(double &);
	void ROBOTIQ3finger_set_fingerc_all(double &,double &, double &);
	void ROBOTIQ3finger_set_fingerabc_all(double &,double &, double &, double &,double &, double &, double &,double &, double &);
	void ROBOTIQ3finger_set_scissor_pos(double &);
	void ROBOTIQ3finger_set_scissor_all(double &,double &, double &);
	
	void ROBOTIQ3finger_glove_mode();
	void ROBOTIQ3finger_stop();
	void ROBOTIQ3finger_auto_release();
	void ROBOTIQ3finger_auto_centering();
	void ROBOTIQ3finger_individual_control_fingerabc();
	void ROBOTIQ3finger_individual_control_scissor();
	
	//check the mode
	int ROBOTIQ3finger_mode();


private:
	//print out the meanings for the registers
    void ROBOTIQ3finger_fault(int &);
	void ROBOTIQ3finger_GACT(int &);
	void ROBOTIQ3finger_GMOD(int &);
	void ROBOTIQ3finger_GGTO(int &);
	void ROBOTIQ3finger_GIMC(int &);
	void ROBOTIQ3finger_GSTA(int &);
	void ROBOTIQ3finger_GDTA(int &);
	void ROBOTIQ3finger_GDTB(int &);
	void ROBOTIQ3finger_GDTC(int &);
	void ROBOTIQ3finger_GDTS(int &);

	string Hex_to_Binary(int);
	
private:
     unsigned char* order;
};

#endif //ROBOTIQ3FINGER_FUNCTION_H  

