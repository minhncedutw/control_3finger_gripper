#include <QCoreApplication>
#include <iostream>
#include <stdio.h>
#include <conio.h>
#include <string.h>
#include <boost/thread.hpp>
#include <boost/crc.hpp>
#include <boost/cstdint.hpp>
#include "MyRobot_Protocol.h"
#include "ROBOTIQ3finger_function.h"
#include <QThread>

#include "tcpserver.h"

enum eThreeFingerMode
{
    BASIC,
    PINCH,
    WIDE,
    SCISSOR
};

class Sleeper : public QThread
{
public:
    static void usleep(unsigned long usecs){QThread::usleep(usecs);}
    static void msleep(unsigned long msecs){QThread::msleep(msecs);}
    static void sleep(unsigned long secs){QThread::sleep(secs);}
};

void waitUntilFinishConfiguring(void);
void waitUntilFinishMoving(void);

//select the right com port
int port=4;
std::string port_num="COM"+std::to_string(static_cast<long long>(port));
//construct ROBOTIQ3finger_function class
ROBOTIQ3finger_function myROBOTIQ3finger_function(port_num);
bool _3finger_enabled = false;

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    TCPServer server;

    myROBOTIQ3finger_function.ROBOTIQ3finger_active_on(1);
    waitUntilFinishConfiguring();
    myROBOTIQ3finger_function.ROBOTIQ3finger_active_off();

    return a.exec();
}

void waitUntilFinishConfiguring() {
    int GACT, GMOD, GGTO,GIMC,GSTA;
    do {
        myROBOTIQ3finger_function.ROBOTIQ3finger_gripper_status_read(GACT,GMOD,GGTO,GIMC,GSTA);
    } while (GIMC<3); // While GIMC==1 or 2, it means that the gripper is activating; When GIMC=3, activation is finished
}

void waitUntilFinishMoving() {
    int GACT, GMOD, GGTO,GIMC,GSTA;
    do {
        myROBOTIQ3finger_function.ROBOTIQ3finger_gripper_status_read(GACT,GMOD,GGTO,GIMC,GSTA);
    } while (GSTA==0); // While GSTA==0, it means that the gripper is moving; When GSTA=3, activation is finished
}

void TCPServer::newConnection() {
    qDebug() << "===========================================";
    QTcpSocket *socket = server->nextPendingConnection();

    // Get request json
    QJsonObject json_obj_request;
    receiveJsonObj(socket, &json_obj_request);

    // Analyse json to get command
    QString cmd = json_obj_request.value("cmd").toString();
    qDebug() << "Command:" << cmd;

    if (cmd == "enable") {
        // Execute command
        myROBOTIQ3finger_function.ROBOTIQ3finger_active_on(0);

        waitUntilFinishConfiguring();

        // Create response command
        QJsonObject json_obj_response;
        json_obj_response["ret"] = "OK";
        // Send command
        sendJsonObj(socket, json_obj_response);
    } else if (cmd == "disable") {
        // Execute command
        myROBOTIQ3finger_function.ROBOTIQ3finger_active_off();

        // Create response command
        QJsonObject json_obj_response;
        json_obj_response["ret"] = "OK";
        // Send command
        sendJsonObj(socket, json_obj_response);
    } else if (cmd == "mode") {
        // Execute command
        eThreeFingerMode mode = eThreeFingerMode(json_obj_request.value("mode").toInt());
        myROBOTIQ3finger_function.ROBOTIQ3finger_active_on(mode);

        waitUntilFinishConfiguring();

        // Create response command
        QJsonObject json_obj_response;
        json_obj_response["ret"] = "OK";
        // Send command
        sendJsonObj(socket, json_obj_response);
    } else if (cmd == "move") {
        // Execute command
        double pos_request = json_obj_request.value("position").toDouble();
        double speed_request = json_obj_request.value("speed").toDouble();
        double force_request = json_obj_request.value("force").toDouble();

        // Move the fingers
        myROBOTIQ3finger_function.ROBOTIQ3finger_set_all(pos_request, speed_request, force_request);
        myROBOTIQ3finger_function.ROBOTIQ3finger_auto_centering();

        waitUntilFinishMoving();

        // Create response command
        QJsonObject json_obj_response;
        json_obj_response["ret"] = "OK";
        // Send command
        sendJsonObj(socket, json_obj_response);
    } else { // If uncorrect command, respond Wrong
        // Create response command
        QJsonObject json_obj_response;
        json_obj_response["ret"] = "Uncorrect Command";
        // Send command
        sendJsonObj(socket, json_obj_response);
    }

    socket->close();
}
