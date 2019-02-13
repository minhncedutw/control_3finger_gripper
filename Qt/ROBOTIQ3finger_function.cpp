/*
This is ROBOTIQ3finger_function 1.0
*/



#include <stdio.h>
#include "ROBOTIQ3finger_function.h"


using namespace std;
typedef unsigned char uchar;
//
//Class ROBOTIQ3finger_function
//


// constructor, set the communication
ROBOTIQ3finger_function::ROBOTIQ3finger_function( std::string com_port ) 
{
	myProtocol=new MyRobot_Protocol( com_port,115200,boost::asio::serial_port_base::parity(
		boost::asio::serial_port_base::parity::none));	//object construction
}

//gripper command
void ROBOTIQ3finger_function::ROBOTIQ3finger_active_on(int mode){ 
	myData.clear();
	unsigned char order[6];
	switch(mode)
	{
	case 0:
		//Activate in Basic Mode
		order[0]=0x09;
		order[1]=0x06;
		order[2]=0x03;
		order[3]=0xE8;
		order[4]=0x01;
		order[5]=0x00; 
		break;
	case 1:
		//Activate in Pinch Mode
		order[0]=0x09;
		order[1]=0x06;
		order[2]=0x03;
		order[3]=0xE8;
		order[4]=0x03;
		order[5]=0x00;; 
		break;
	case 2:
		//Activate in Wide Mode
		order[0]=0x09;
		order[1]=0x06;
		order[2]=0x03;
		order[3]=0xE8;
		order[4]=0x05;
		order[5]=0x00; 
		break;
	case 3:
		//Activate in Scissor Mode
		order[0]=0x09;
		order[1]=0x06;
		order[2]=0x03;
		order[3]=0xE8;
		order[4]=0x07;
		order[5]=0x00; 
		break;
	default:
		printf("Please enter the correct mode order.\n\n");
		break;
	}
	int a = sizeof(order);
    for(int i = 0; i < a; i++) {
		myData.push_back(order[i]); }
	bool status=myProtocol->Protocol_communicate(myData,MODBUS);
    printf("\n");
	
} 

void ROBOTIQ3finger_function::ROBOTIQ3finger_active_off(){ 
	myData.clear();
	unsigned char order[] = {0x09,0x06,0x03,0xE8,0x00,0x00};  
	int a = sizeof(order);
    for(int i = 0; i < a; i++) {
		myData.push_back(order[i]); }
	bool status=myProtocol->Protocol_communicate(myData,MODBUS);
	printf("\n");
} 

void ROBOTIQ3finger_function::ROBOTIQ3finger_action_status_read(double &pr,double & pn,double & current ){ 
	myData.clear();
	unsigned char order[] = {0x09,0x03,0x07,0xD1,0x00,0x02};
	int a = sizeof(order);
    for(int i = 0; i < a; i++) {
		myData.push_back(order[i]); }
	bool status=myProtocol->Protocol_communicate(myData,MODBUS);

	pr = myData[4];
	pn = myData[5];
	current = 0.1*myData[6];
	printf("目標位置= %2.2f , 現在位置= %2.2f , 電流= %2.2f mA \n",pr,pn,current);
    printf("\n");
	
} 

void ROBOTIQ3finger_function::ROBOTIQ3finger_a_action_status_read(double &pr,double & pn,double & current ){ 
	myData.clear();
	unsigned char order[] = {0x09,0x03,0x07,0xD1,0x00,0x02};
	int a = sizeof(order);
    for(int i = 0; i < a; i++) {
		myData.push_back(order[i]); }
	bool status=myProtocol->Protocol_communicate(myData,MODBUS);

	pr = myData[4];
	pn = myData[5];
	current = 0.1*myData[6];
	printf("目標位置= %2.2f , 現在位置= %2.2f , 電流= %2.2f mA \n",pr,pn,current);
    printf("\n");
} 

void ROBOTIQ3finger_function::ROBOTIQ3finger_b_action_status_read(double &pr,double & pn,double & current ){ 
	myData.clear();
	unsigned char order[] = {0x09,0x03,0x07,0xD3,0x00,0x02};
	int a = sizeof(order);
    for(int i = 0; i < a; i++) {
		myData.push_back(order[i]); }
	bool status=myProtocol->Protocol_communicate(myData,MODBUS);

	pr = myData[3];
	pn = myData[4];
	current = 0.1*myData[5];
	printf("目標位置= %2.2f , 現在位置= %2.2f , 電流= %2.2f mA \n",pr,pn,current);
    printf("\n");
}

void ROBOTIQ3finger_function::ROBOTIQ3finger_c_action_status_read(double &pr,double & pn,double & current ){ 
	myData.clear();
	unsigned char order[] = {0x09,0x03,0x07,0xD4,0x00,0x02};
	int a = sizeof(order);
    for(int i = 0; i < a; i++) {
		myData.push_back(order[i]); }
	bool status=myProtocol->Protocol_communicate(myData,MODBUS);

	pr = myData[4];
	pn = myData[5];
	current = 0.1*myData[6];
	printf("目標位置= %2.2f , 現在位置= %2.2f , 電流= %2.2f mA \n",pr,pn,current);
    printf("\n");
}

void ROBOTIQ3finger_function::ROBOTIQ3finger_s_action_status_read(double &pr,double & pn,double & current ){ 
	myData.clear();
	unsigned char order[] = {0x09,0x03,0x07,0xD6,0x00,0x02};
	int a = sizeof(order);
    for(int i = 0; i < a; i++) {
		myData.push_back(order[i]); }
	bool status=myProtocol->Protocol_communicate(myData,MODBUS);

	pr = myData[3];
	pn = myData[4];
	current = 0.1*myData[5];
	printf("目標位置= %2.2f , 現在位置= %2.2f , 電流= %2.2f mA \n",pr,pn,current);
    printf("\n");
}

void ROBOTIQ3finger_function::ROBOTIQ3finger_gripper_status_read(int& GACT,int& GMOD,int& GGTO,int& GIMC,int& GSTA){
	//system status parameters
	//Please refer to the page 50 in the instruction manual(v.2014)
	//Or, refer to the page 42 in the instruction manual(v.2013)
	myData.clear();
	unsigned char order[] = {0x09,0x03,0x07,0xD0,0x00,0x01};
	int size = sizeof(order);
    for(int i = 0; i < size; i++) 
	{
		myData.push_back(order[i]); 
	}
	bool status=myProtocol->Protocol_communicate(myData,MODBUS);

	int a = myData[3];//(d):a -> (hex):pq -> (b) x(0123)y(0123) -> (bit) 76543210
	int p = a/16;
	int q = a%16;
	string x = this->Hex_to_Binary(p);
	string y = this->Hex_to_Binary(q);
	GACT = y[3]-48;
	GMOD = (y[1]-48)*2+(y[2]-48);
	GGTO = y[0]-48;
	GIMC = (x[2]-48)*2+(x[3]-48);//p%4;
	GSTA = (x[0]-48)*2+(x[1]-48);//p/4;
	//printf("十六進位 = %d %d \n", p,q);
	//printf("二進位 = %s %s\n",x.c_str(),y.c_str());x[
	//printf("Initialization status: %c  Mode: %d  GOTO: %c  Gripper status: %c%c Objective detection status: %c%c \n",y[3],y[2],y[1],y[0],x[3],x[2],x[1],x[0]);
	printf("Initialization status: %d  Mode: %d  GOTO: %d  Gripper status: %d Objective detection status: %d \n",GACT,GMOD,GGTO,GIMC,GSTA);
	
	printf("Initialization status: ");
	this->ROBOTIQ3finger_GACT(GACT);
	printf("Mode: ");
	this->ROBOTIQ3finger_GMOD(GMOD);
	printf("GOTO: ");
	this->ROBOTIQ3finger_GGTO(GGTO);
	printf("Gripper status: ");
	this->ROBOTIQ3finger_GIMC(GIMC);
	printf("Objective detection status: ");
	this->ROBOTIQ3finger_GSTA(GSTA);
	printf("\n");
}

void ROBOTIQ3finger_function::ROBOTIQ3finger_object_status_read(int& GDTA,int& GDTB,int& GDTC,int& GDTS){
	//object status parameters
	//Please refer to the page 51 in the instruction manual(v.2014)
	//Or, refer to the page 43 in the instruction manual(v.2013)
	myData.clear();
	unsigned char order[] = {0x09,0x03,0x07,0xD0,0x00,0x01};
	int size = sizeof(order);
    for(int i = 0; i < size; i++) {
		myData.push_back(order[i]); }
	bool status=myProtocol->Protocol_communicate(myData,MODBUS);

	int a = myData[4];//(d):a -> (hex):pq -> (b) x(0123)y(0123) -> (bit) 76543210
	int p = a/16;
	int q = a%16;
	string x = this->Hex_to_Binary(p);
	string y = this->Hex_to_Binary(q);
	GDTA = (y[2]-48)*2+(y[3]-48);
	GDTB = (y[0]-48)*2+(y[1]-48);
	GDTC = (x[2]-48)*2+(x[3]-48);
	GDTS = (x[0]-48)*2+(x[1]-48);
	//printf("十六進位 = %d %d \n", p,q);
	//printf("二進位 = %s %s\n",x.c_str(),y.c_str());x[
	//printf("Finger A: %c%c  Finger B: %c%c  Finger C: %c%c  Scissor: %c%c \n",y[3],y[2],y[1],y[0],x[3],x[2],x[1],x[0]);
	printf("Finger A: %d  Finger B: %d  Finger C: %d  Scissor: %d \n",GDTA,GDTB,GDTC,GDTS);
	
	printf("Finger A: ");
	this->ROBOTIQ3finger_GDTA(GDTA);
	printf("Finger B: ");
	this->ROBOTIQ3finger_GDTB(GDTB);
	printf("Finger C: ");
	this->ROBOTIQ3finger_GDTC(GDTC);
	printf("Scissor: ");
	this->ROBOTIQ3finger_GDTS(GDTS);
	printf("\n");
}

void ROBOTIQ3finger_function::ROBOTIQ3finger_fault_read(int& GFLT){ 
	//fault status parameters
	//Please refer to the page 53 in the instruction manual(v.2014)
	//Or, refer to the page 45 in the instruction manual(v.2013)
	myData.clear();
	unsigned char order[] = {0x09,0x03,0x07,0xD1,0x00,0x01};
	int a = sizeof(order);
    for(int i = 0; i < a; i++) {
		myData.push_back(order[i]); }
	bool status=myProtocol->Protocol_communicate(myData,MODBUS);
		
	GFLT = myData[3];
	printf("Fault no.= 0%X \n",GFLT);
	this->ROBOTIQ3finger_fault(GFLT);
	printf("\n");
} 

void ROBOTIQ3finger_function::ROBOTIQ3finger_set_pos(double &pos){ 

	myData.clear();
	unsigned char order[] ={0x09,0x06,0x03,0xE9,0x00,0x00}; 
	int a = sizeof(order);

	//position
	int p;
	if (pos>255)
		p=255;
	else if (pos<0)
		p=0;
	else
		p=pos;
	
	order[5]=p;

    for(int i = 0; i < a; i++) {
		myData.push_back(order[i]); }
	bool status=myProtocol->Protocol_communicate(myData,MODBUS);
    printf("\n");
} 
void ROBOTIQ3finger_function::ROBOTIQ3finger_set_all(double &pos ,double &vel, double &force){ 

	myData.clear();
	unsigned char order[] ={0x09,0x10,0x03,0xE9,0x00,0x02,0x04,0x00,0x00,0x00,0x00}; 
	int a = sizeof(order);

	//position
	int p;
	if (pos>255)
		p=255;
	else if (pos<0)
		p=0;
	else
		p=pos;
	order[8]=p;

	//velocity
	int v = (int)((vel-22)*255/88+0.5);
	if (v>255)
		v=255;
	else if (v<0)
		v=0;
	order[9]=v;

	//force
	int f = (int)((force-15)*255/45+0.5);
	if (f>255)
		f=255;
	else if (f<0)
		f=0;
	order[10]=f;

    for(int i = 0; i < a; i++) {
		myData.push_back(order[i]); }
	bool status=myProtocol->Protocol_communicate(myData,MODBUS);
    printf("\n");
}
void ROBOTIQ3finger_function::ROBOTIQ3finger_set_fingera_pos(double &pos){ 

	myData.clear();
	unsigned char order[] ={0x09,0x06,0x03,0xE9,0x00,0x00}; 
	int a = sizeof(order);

	//position
	int p;
	if (pos>255)
		p=255;
	else if (pos<0)
		p=0;
	else
		p=pos;
	order[5]=p;

    for(int i = 0; i < a; i++) {
		myData.push_back(order[i]); }
	bool status=myProtocol->Protocol_communicate(myData,MODBUS);
    printf("\n");
} 
void ROBOTIQ3finger_function::ROBOTIQ3finger_set_fingera_all(double &pos ,double &vel, double &force){ 

	myData.clear();
	unsigned char order[] ={0x09,0x10,0x03,0xE9,0x00,0x02,0x04,0x00,0x00,0x00,0x00}; 
	int a = sizeof(order);

	//position
	int p;
	if (pos>255)
		p=255;
	else if (pos<0)
		p=0;
	else
		p=pos;
	order[8]=p;

	//velocity
	int v = (int)((vel-22)*255/88+0.5);
	if (v>255)
		v=255;
	else if (v<0)
		v=0;
	order[9]=v;

	//force
	int f = (int)((force-15)*255/45+0.5);
	if (f>255)
		f=255;
	else if (f<0)
		f=0;
	order[10]=f;

    for(int i = 0; i < a; i++) {
		myData.push_back(order[i]); }
	bool status=myProtocol->Protocol_communicate(myData,MODBUS);
    printf("\n");
}
void ROBOTIQ3finger_function::ROBOTIQ3finger_set_fingerb_pos(double &pos){ 

	myData.clear();
	unsigned char order[] ={0x09,0x06,0x03,0xEB,0x00,0x00}; 
	int a = sizeof(order);

	//position
	int p;
	if (pos>255)
		p=255;
	else if (pos<0)
		p=0;
	else
		p=pos;
	order[4]=p;

    for(int i = 0; i < a; i++) {
		myData.push_back(order[i]); }
	bool status=myProtocol->Protocol_communicate(myData,MODBUS);
    printf("\n");
} 
void ROBOTIQ3finger_function::ROBOTIQ3finger_set_fingerb_all(double &pos ,double &vel, double &force){ 

	myData.clear();
	unsigned char order[] ={0x09,0x10,0x03,0xEB,0x00,0x02,0x04,0x00,0x00,0x00,0x00}; 
	int a = sizeof(order);

	//position
	int p;
	if (pos>255)
		p=255;
	else if (pos<0)
		p=0;
	else
		p=pos;
	order[7]=p;

	//velocity
	int v = (int)((vel-22)*255/88+0.5);
	if (v>255)
		v=255;
	else if (v<0)
		v=0;
	order[8]=v;

	//force
	int f = (int)((force-15)*255/45+0.5);
	if (f>255)
		f=255;
	else if (f<0)
		f=0;
	order[9]=f;
	double fingerc_pr;
	double fingerc_pn;
	double fingerc_current;
	ROBOTIQ3finger_function::ROBOTIQ3finger_c_action_status_read(fingerc_pr,fingerc_pn,fingerc_current );
	int fingerc_p= (int)((255-(fingerc_pr*255.0/87.0))+0.5);
	order[10]=fingerc_p;
    for(int i = 0; i < a; i++) {
		myData.push_back(order[i]); }
	bool status=myProtocol->Protocol_communicate(myData,MODBUS);
    printf("\n");
}
void ROBOTIQ3finger_function::ROBOTIQ3finger_set_fingerc_pos(double &pos){ 

	myData.clear();
	unsigned char order[] ={0x09,0x06,0x03,0xEC,0x00,0x00}; 
	int a = sizeof(order);

	//position
	int p;
	if (pos>255)
		p=255;
	else if (pos<0)
		p=0;
	else
		p=pos;
	order[5]=p;

    for(int i = 0; i < a; i++) {
		myData.push_back(order[i]); }
	bool status=myProtocol->Protocol_communicate(myData,MODBUS);
    printf("\n");
} 
void ROBOTIQ3finger_function::ROBOTIQ3finger_set_fingerc_all(double &pos ,double &vel, double &force){ 

	myData.clear();
	unsigned char order[] ={0x09,0x10,0x03,0xEC,0x00,0x02,0x04,0x00,0x00,0x00,0x00}; 
	int a = sizeof(order);

	//position
	int p;
	if (pos>255)
		p=255;
	else if (pos<0)
		p=0;
	else
		p=pos;
	order[8]=p;

	//velocity
	int v = (int)((vel-22)*255/88+0.5);
	if (v>255)
		v=255;
	else if (v<0)
		v=0;
	order[9]=v;

	//force
	int f = (int)((force-15)*255/45+0.5);
	if (f>255)
		f=255;
	else if (f<0)
		f=0;
	order[10]=f;

    for(int i = 0; i < a; i++) {
		myData.push_back(order[i]); }
	bool status=myProtocol->Protocol_communicate(myData,MODBUS);
    printf("\n");
}

void ROBOTIQ3finger_function::ROBOTIQ3finger_set_fingerabc_all(double &posA ,double &velA, double &forceA, double &posB ,double &velB, double &forceB, double &posC ,double &velC, double &forceC){ 

	myData.clear();
	unsigned char order[] ={0x09,0x10,0x03,0xE9,0x00,0x05,0x0A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; 
	int a = sizeof(order);

	//position
	int pA;
	if (posA>255)
		pA=255;
	else if (posA<0)
		pA=0;
	else
		pA=posA;
	order[8]=pA;

	//velocity
	int vA = (int)((velA-22)*255/88+0.5);
	if (vA>255)
		vA=255;
	else if (vA<0)
		vA=0;
	order[9]=vA;

	//force
	int fA = (int)((forceA-15)*255/45+0.5);
	if (fA>255)
		fA=255;
	else if (fA<0)
		fA=0;
	order[10]=fA;

	//position
	int pB;
	if (posB>255)
		pB=255;
	else if (posB<0)
		pB=0;
	else
		pB=posB;
	order[11]=pB;

	//velocity
	int vB = (int)((velB-22)*255/88+0.5);
	if (vB>255)
		vB=255;
	else if (vB<0)
		vB=0;
	order[12]=vB;

	//force
	int fB = (int)((forceB-15)*255/45+0.5);
	if (fB>255)
		fB=255;
	else if (fB<0)
		fB=0;
	order[13]=fB;

	//position
	int pC;
	if (posC>255)
		pC=255;
	else if (posC<0)
		pC=0;
	else
		pC=posC;
	order[14]=pC;

	//velocity
	int vC = (int)((velC-22)*255/88+0.5);
	if (vC>255)
		vC=255;
	else if (vC<0)
		vC=0;
	order[15]=vC;

	//force
	int fC = (int)((forceC-15)*255/45+0.5);
	if (fC>255)
		fC=255;
	else if (fC<0)
		fC=0;
	order[16]=fC;

    for(int i = 0; i < a; i++) {
		myData.push_back(order[i]); }
	bool status=myProtocol->Protocol_communicate(myData,MODBUS);
    printf("\n");
}

void ROBOTIQ3finger_function::ROBOTIQ3finger_set_scissor_pos(double &pos){ 

	myData.clear();
	unsigned char order[] ={0x09,0x06,0x03,0xEE,0x00,0x00}; 
	int a = sizeof(order);

	//position
	int p;
	if (pos>255)
		p=255;
	else if (pos<0)
		p=0;
	else
		p=pos;
	order[4]=p;

    for(int i = 0; i < a; i++) {
		myData.push_back(order[i]); }
	bool status=myProtocol->Protocol_communicate(myData,MODBUS);
    printf("\n");	
} 
void ROBOTIQ3finger_function::ROBOTIQ3finger_set_scissor_all(double &pos ,double &vel, double &force){ 

	myData.clear();
	unsigned char order[] ={0x09,0x10,0x03,0xEE,0x00,0x02,0x04,0x00,0x00,0x00,0x00}; 
	int a = sizeof(order);

	//position
	int p;
	if (pos>255)
		p=255;
	else if (pos<0)
		p=0;
	else
		p=pos;
	order[7]=p;

	//velocity
	int v = (int)((vel-22)*255/88+0.5);
	if (v>255)
		v=255;
	else if (v<0)
		v=0;
	order[8]=v;

	//force
	int f = (int)((force-15)*255/45+0.5);
	if (f>255)
		f=255;
	else if (f<0)
		f=0;
	order[9]=f;

    for(int i = 0; i < a; i++) {
		myData.push_back(order[i]); }
	bool status=myProtocol->Protocol_communicate(myData,MODBUS);
    printf("\n");
}
void ROBOTIQ3finger_function::ROBOTIQ3finger_glove_mode(){ 

	myData.clear();
	unsigned char order[6];
	
	switch(ROBOTIQ3finger_function::ROBOTIQ3finger_mode())
	{
	case 0:
		//glove in Basic Mode
		order[0]=0x09;
		order[1]=0x06;
		order[2]=0x03;
		order[3]=0xE8;
		order[4]=0x09;
		order[5]=0x01; 
		break;
	case 1:
		//glove in Pinch Mode
		order[0]=0x09;
		order[1]=0x06;
		order[2]=0x03;
		order[3]=0xE8;
		order[4]=0x0B;
		order[5]=0x01; 
		break;
	case 2:
		//glove in Wide Mode
		order[0]=0x09;
		order[1]=0x06;
		order[2]=0x03;
		order[3]=0xE8;
		order[4]=0x0D;
		order[5]=0x01; 
		break;
	case 3:
		//glove in Scissor Mode
		order[0]=0x09;
		order[1]=0x06;
		order[2]=0x03;
		order[3]=0xE8;
		order[4]=0x0F;
		order[5]=0x01; 
		break;
	} 
	int a = sizeof(order);
    for(int i = 0; i < a; i++) {
		myData.push_back(order[i]); }
	bool status=myProtocol->Protocol_communicate(myData,MODBUS);
    printf("\n");	
}
void ROBOTIQ3finger_function::ROBOTIQ3finger_stop(){ 

	myData.clear();
	unsigned char order[6];
	
	switch(ROBOTIQ3finger_function::ROBOTIQ3finger_mode())
	{
	case 0:
		//stop in Basic Mode
		order[0]=0x09;
		order[1]=0x06;
		order[2]=0x03;
		order[3]=0xE8;
		order[4]=0x01;
		order[5]=0x00; 
		break;
	case 1:
		//stop in Pinch Mode
		order[0]=0x09;
		order[1]=0x06;
		order[2]=0x03;
		order[3]=0xE8;
		order[4]=0x03;
		order[5]=0x00; 
		break;
	case 2:
		//stop in Wide Mode
		order[0]=0x09;
		order[1]=0x06;
		order[2]=0x03;
		order[3]=0xE8;
		order[4]=0x05;
		order[5]=0x00; 
		break;
	case 3:
		//stop in Scissor Mode
		order[0]=0x09;
		order[1]=0x06;
		order[2]=0x03;
		order[3]=0xE8;
		order[4]=0x07;
		order[5]=0x00; 
		break;
	}
	int a = sizeof(order);
    for(int i = 0; i < a; i++) {
		myData.push_back(order[i]); }
	bool status=myProtocol->Protocol_communicate(myData,MODBUS);
    printf("\n");
		
}

void ROBOTIQ3finger_function::ROBOTIQ3finger_auto_release(){ 

	myData.clear();
	unsigned char order[6];
	
	switch(ROBOTIQ3finger_function::ROBOTIQ3finger_mode())
	{
	case 0:
		//auto_release in Basic Mode
		order[0]=0x09;
		order[1]=0x06;
		order[2]=0x03;
		order[3]=0xE8;
		order[4]=0x19;
		order[5]=0x00; 
		break;
	case 1:
		//auto_elease in Pinch Mode
		order[0]=0x09;
		order[1]=0x06;
		order[2]=0x03;
		order[3]=0xE8;
		order[4]=0x1B;
		order[5]=0x00; 
		break;
	case 2:
		//auto_release in Wide Mode
		order[0]=0x09;
		order[1]=0x06;
		order[2]=0x03;
		order[3]=0xE8;
		order[4]=0x1D;
		order[5]=0x00; 
		break;
	case 3:
		//auto_release in Scissor Mode
		order[0]=0x09;
		order[1]=0x06;
		order[2]=0x03;
		order[3]=0xE8;
		order[4]=0x1F;
		order[5]=0x00; 
		break;
	}
	int a = sizeof(order);
    for(int i = 0; i < a; i++) {
		myData.push_back(order[i]); }
	bool status=myProtocol->Protocol_communicate(myData,MODBUS);
    printf("\n");
	
	
}

void ROBOTIQ3finger_function::ROBOTIQ3finger_auto_centering(){ 

	myData.clear();
	unsigned char order[6];
	int GMOD=ROBOTIQ3finger_function::ROBOTIQ3finger_mode();
		
	switch(GMOD)
	{
	case 0:
		//auto_centering in Basic Mode
		order[0]=0x09;
		order[1]=0x06;
		order[2]=0x03;
		order[3]=0xE8;
		order[4]=0x09;
		order[5]=0x02; 
		break;
	case 1:
		//auto_centering in Pinch Mode
		order[0]=0x09;
		order[1]=0x06;
		order[2]=0x03;
		order[3]=0xE8;
		order[4]=0x0B;
		order[5]=0x02; 
		break;
	case 2:
		//auto_centering in Wide Mode
		order[0]=0x09;
		order[1]=0x06;
		order[2]=0x03;
		order[3]=0xE8;
		order[4]=0x0D;
		order[5]=0x02; 
		break;
	case 3:
		//auto_centering in Scissor Mode
		order[0]=0x09;
		order[1]=0x06;
		order[2]=0x03;
		order[3]=0xE8;
		order[4]=0x0F;
		order[5]=0x02; 
		break;
	} 
	int a = sizeof(order);
	for(int i = 0; i < a; i++) {
		myData.push_back(order[i]); }
	bool status=myProtocol->Protocol_communicate(myData,MODBUS);
	printf("\n");
}
	


void ROBOTIQ3finger_function::ROBOTIQ3finger_individual_control_fingerabc(){ 

	myData.clear();
	unsigned char order[6];
	
	switch(ROBOTIQ3finger_function::ROBOTIQ3finger_mode())
	{
	case 0:
		//individual_control_fingerabc in Basic Mode
		order[0]=0x09;
		order[1]=0x06;
		order[2]=0x03;
		order[3]=0xE8;
		order[4]=0x09;
		order[5]=0x04; 
		break;
	case 1:
		//individual_control_fingerabc in Pinch Mode
		order[0]=0x09;
		order[1]=0x06;
		order[2]=0x03;
		order[3]=0xE8;
		order[4]=0x0B;
		order[5]=0x04; 
		break;
	case 2:
		//individual_control_fingerabc in Wide Mode
		order[0]=0x09;
		order[1]=0x06;
		order[2]=0x03;
		order[3]=0xE8;
		order[4]=0x0D;
		order[5]=0x04;
		break;
	case 3:
		//individual_control_fingerabc in Scissor Mode
		order[0]=0x09;
		order[1]=0x06;
		order[2]=0x03;
		order[3]=0xE8;
		order[4]=0x0F;
		order[5]=0x04; 
		break;
	} 
	int a = sizeof(order);
    for(int i = 0; i < a; i++) {
		myData.push_back(order[i]); }
	bool status=myProtocol->Protocol_communicate(myData,MODBUS);
    printf("\n");	
}
void ROBOTIQ3finger_function::ROBOTIQ3finger_individual_control_scissor(){ 

	myData.clear();
	unsigned char order[6];
	
	switch(ROBOTIQ3finger_function::ROBOTIQ3finger_mode())
	{
	case 0:
		//individual_control_scissor in Basic Mode
		order[0]=0x09;
		order[1]=0x06;
		order[2]=0x03;
		order[3]=0xE8;
		order[4]=0x09;
		order[5]=0x08; 
		break;
	case 1:
		//individual_control_scissor in Pinch Mode
		order[0]=0x09;
		order[1]=0x06;
		order[2]=0x03;
		order[3]=0xE8;
		order[4]=0x0B;
		order[5]=0x08; 
		break;
	case 2:
		//individual_control_scissor in Wide Mode
		order[0]=0x09;
		order[1]=0x06;
		order[2]=0x03;
		order[3]=0xE8;
		order[4]=0x0D;
		order[5]=0x08; 
		break;
	case 3:
		//individual_control_scissor in Scissor Mode
		order[0]=0x09;
		order[1]=0x06;
		order[2]=0x03;
		order[3]=0xE8;
		order[4]=0x0F;
		order[5]=0x08;
		break;
	} 
	int a = sizeof(order);
    for(int i = 0; i < a; i++) {
		myData.push_back(order[i]); }
	bool status=myProtocol->Protocol_communicate(myData,MODBUS);
    printf("\n");
}

int ROBOTIQ3finger_function::ROBOTIQ3finger_mode()
{
	myData.clear();
	unsigned char order[] = {0x09,0x03,0x07,0xD0,0x00,0x01};
	int size = sizeof(order);
    for(int i = 0; i < size; i++) {
		myData.push_back(order[i]); }
	bool status=myProtocol->Protocol_communicate(myData,MODBUS);

	int a = myData[3];//(d):a -> (hex):pq -> (b) x(0123)y(0123) -> (bit) 76543210
	int p = a/16;
	int q = a%16;
	string x = this->Hex_to_Binary(p);
	string y = this->Hex_to_Binary(q);
	//printf("十六進位 = %d %d \n", p,q);
	//printf("二進位 = %s %s\n",x.c_str(),y.c_str());
	myData.clear();
	return ((y[1]-48)*2+(y[2]-48));
}
//----------------------------------------------------

void ROBOTIQ3finger_function::ROBOTIQ3finger_fault(int &a){
	a = a%16;
	switch (a){
	case 0: 
		printf("No Fault\n"); 
		break;
	case 5: 
        printf("Priority Fault:\n Action delayed, activation (reactivation) must be completed prior to action\n");
		break;
	case 6: 
        printf("Priority Fault:\n Action delayed, mode change must be completed prior to action\n");
		break;
	case 7:
		printf("Priority Fault:\n The activation bit must be set prior to action\n");
		break;
	case 9:
	    printf("Minor Fault (red LED continuous):\n The communication chip is not ready (may be booting)\n");
		break;
	case 10: 
        printf("Minor Fault (red LED continuous):\n Changing mode fault, interferences detected on Scissor (for less than 20 sec)\n");
		break;
	case 11:
		printf("Minor Fault (red LED continuous):\n Automatic release in progress\n");
		break;
	case 13:
		printf("Major Fault (red LED blinking): Reset is required!!!\n Activation fault, verify that no interference or other error occured\n");
		break;
	case 14:
		printf("Major Fault (red LED blinking): Reset is required!!!\n Changing mode fault, interferences detected on Scissor (for more than 20 sec)\n");
		break;
	case 15:
		printf("Major Fault (red LED blinking): Reset is required!!!\n Automatic release completed. Reset and activation is required.\n");
		break;
	default:
		printf("system error\n");
		break;
	}
}
void ROBOTIQ3finger_function::ROBOTIQ3finger_GACT(int &a){
	switch (a){
	case 0: 
		printf("Gripper reset \n"); 
		break;
	case 1: 
        printf("Gripper activation \n");
		break;
	default:
		printf("system error \n");
		break;
	}
}
void ROBOTIQ3finger_function::ROBOTIQ3finger_GMOD(int &a){
	switch (a){
	case 0: 
		printf("Basic Mode \n"); 
		break;
	case 1: 
        printf("Pinch Mode \n");
		break;
	case 2:
		printf("Wide Mode \n");
		break;
	case 3:
	    printf("Scissor Mode \n");
		break;
	default:
		printf("system error\n");
		break;
	}
}
void ROBOTIQ3finger_function::ROBOTIQ3finger_GGTO(int &a){
	switch (a){
	case 0: 
		printf("Stopped (or performing activation / grasping mode change / automatic release) \n"); 
		break;
	case 1: 
        printf("Go to Position Request \n");
		break;
	default:
		printf("system error \n");
		break;
	}
}
void ROBOTIQ3finger_function::ROBOTIQ3finger_GIMC(int &a){
		switch (a){
	case 0: 
		printf("Gripper is in reset (or automatic release) state. see Fault Status if Gripper is activated.\n"); 
		break;
	case 1: 
        printf("Activation in progress.\n");
		break;
	case 2:
		printf("Mode change in progress.\n");
		break;
	case 3:
	    printf("Activation and mode change are completed.\n");
		break;
	default:
		printf("system error\n");
		break;
		}
}
void ROBOTIQ3finger_function::ROBOTIQ3finger_GSTA(int &a){
		switch (a){
	case 0: 
		printf("Gripper is in motion towards requested position (only meaningful if gGTO = 1). \n"); 
		break;
	case 1: 
        printf("Gripper is stopped. One or two fingers stopped before requested position. \n");
		break;
	case 2:
		printf("Gripper is stopped. All fingers stopped before requested position.\n");
		break;
	case 3:
	    printf("Gripper is stopped. All fingers reached requested position.\n");
		break;
	default:
		printf("system error\n");
		break;
		}
}
void ROBOTIQ3finger_function::ROBOTIQ3finger_GDTA(int &a){
	switch (a){
	case 0: 
		printf("Finger A is in motion (only meaningful if gGTO = 1) \n"); 
		break;
	case 1: 
        printf("Finger A has stopped due to a contact while opening \n");
		break;
	case 2:
		printf("Finger A has stopped due to a contact while closing \n");
		break;
	case 3:
	    printf("Finger A is at requested position \n");
		break;
	default:
		printf("system error\n");
		break;
	}
}
void ROBOTIQ3finger_function::ROBOTIQ3finger_GDTB(int &a){
	switch (a){
	case 0: 
		printf("Finger B is in motion (only meaningful if gGTO = 1) \n"); 
		break;
	case 1: 
        printf("Finger B has stopped due to a contact while opening \n");
		break;
	case 2:
		printf("Finger B has stopped due to a contact while closing \n");
		break;
	case 3:
	    printf("Finger B is at requested position \n");
		break;
	default:
		printf("system error\n");
		break;
	}
}
void ROBOTIQ3finger_function::ROBOTIQ3finger_GDTC(int &a){
	switch (a){
	case 0: 
		printf("Finger C is in motion (only meaningful if gGTO = 1) \n"); 
		break;
	case 1: 
        printf("Finger C has stopped due to a contact while opening \n");
		break;
	case 2:
		printf("Finger C has stopped due to a contact while closing \n");
		break;
	case 3:
	    printf("Finger C is at requested position \n");
		break;
	default:
		printf("system error\n");
		break;
	}
}
void ROBOTIQ3finger_function::ROBOTIQ3finger_GDTS(int &a){
	switch (a){
	case 0: 
		printf("Scissor is in motion (only meaningful if gGTO = 1) \n"); 
		break;
	case 1: 
        printf("Scissor has stopped due to a contact while opening \n");
		break;
	case 2:
		printf("Scissor has stopped due to a contact while closing \n");
		break;
	case 3:
	    printf("Scissor is at requested position \n");
		break;
	default:
		printf("system error\n");
		break;
	}
}
string ROBOTIQ3finger_function::Hex_to_Binary(int ch){
	char bin[5]={0};
	string tmp;
	switch (ch){
	case 0: 
		bin[0] ='0'; bin[1] ='0'; bin[2] ='0'; bin[3] ='0';	bin[4] ='\0';
		tmp.assign(bin);      return tmp;
	case 1: 
		bin[0] ='0'; bin[1] ='0'; bin[2] ='0'; bin[3] ='1';	bin[4] ='\0';
		tmp.assign(bin);      return tmp;
	case 2:
		bin[0] ='0'; bin[1] ='0'; bin[2] ='1'; bin[3] ='0';	bin[4] ='\0';
		tmp.assign(bin);      return tmp;
	case 3:
	    bin[0] ='0'; bin[1] ='0'; bin[2] ='1'; bin[3] ='1';	bin[4] ='\0';
		tmp.assign(bin);      return tmp;
	case 4:
		bin[0] ='0'; bin[1] ='1'; bin[2] ='0'; bin[3] ='0';	bin[4] ='\0';
		tmp.assign(bin);      return tmp;
	case 5:
		bin[0] ='0'; bin[1] ='1'; bin[2] ='0'; bin[3] ='1';	bin[4] ='\0';
		tmp.assign(bin);      return tmp;
	case 6:
		bin[0] ='0'; bin[1] ='1'; bin[2] ='1'; bin[3] ='0';	bin[4] ='\0';
		tmp.assign(bin);      return tmp;
	case 7:
		bin[0] ='0'; bin[1] ='1'; bin[2] ='1'; bin[3] ='1';	bin[4] ='\0';
		tmp.assign(bin);      return tmp;
	case 8:
		bin[0] ='1'; bin[1] ='0'; bin[2] ='0'; bin[3] ='0';	bin[4] ='\0';
		tmp.assign(bin);      return tmp;
	case 9:
		bin[0] ='1'; bin[1] ='0'; bin[2] ='0'; bin[3] ='1';	bin[4] ='\0';
		tmp.assign(bin);      return tmp;
	case 10:
		bin[0] ='1'; bin[1] ='0'; bin[2] ='1'; bin[3] ='0';	bin[4] ='\0';
		tmp.assign(bin);      return tmp;
	case 11:
		bin[0] ='1'; bin[1] ='0'; bin[2] ='1'; bin[3] ='1';	bin[4] ='\0';
		tmp.assign(bin);      return tmp;
	case 12:
		bin[0] ='1'; bin[1] ='1'; bin[2] ='0'; bin[3] ='0';	bin[4] ='\0';
		tmp.assign(bin);      return tmp;
	case 13:
		bin[0] ='1'; bin[1] ='1'; bin[2] ='0'; bin[3] ='1';	bin[4] ='\0';
		tmp.assign(bin);      return tmp;
	case 14:
		bin[0] ='1'; bin[1] ='1'; bin[2] ='1'; bin[3] ='0';	bin[4] ='\0';
		tmp.assign(bin);      return tmp;
	case 15:
		bin[0] ='1'; bin[1] ='1'; bin[2] ='1'; bin[3] ='1';	bin[4] ='\0';
		tmp.assign(bin);      return tmp;
	default:	
		bin[0] ='x'; bin[1] ='x'; bin[2] ='x'; bin[3] ='x';	bin[4] ='\0';
		tmp.assign(bin);      return tmp;
	}
	
}


// deconstructor 
ROBOTIQ3finger_function::~ROBOTIQ3finger_function() 
{
	//serial port close
	myProtocol->close();

	delete myProtocol;
}
