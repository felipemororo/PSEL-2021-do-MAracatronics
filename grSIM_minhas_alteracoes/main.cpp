#include <QCoreApplication>
#include <chrono>
#include <thread>
#include <modules/actuator/actuator.h>
#include <modules/vision/vision.h>
#include <math.h>


float getPlayerRotateAngleTo(float x, float y, SSL_DetectionRobot Robot) {
    float componentX = (x - Robot.x());
    float componentY = (y - Robot.y());
    float distToTarget = sqrt(pow(componentX, 2) + pow(componentY, 2));

    componentX = componentX / distToTarget;

    // Check possible divisions for 0
    if(isnanf(componentX)) {
        return 0.0f;
    }

    float angleOriginToTarget; // Angle from field origin to targetPosition
    float angleRobotToTarget;  // Angle from robot to targetPosition

    if(componentY < 0.0f) {
        angleOriginToTarget = 2*M_PI - acos(componentX); // Angle that the target make with x-axis to robot
    } else {
        angleOriginToTarget = acos(componentX); // Angle that the target make with x-axis to robot
    }

    angleRobotToTarget = angleOriginToTarget - Robot.orientation();

    // Adjusting to rotate the minimum possible
    if(angleRobotToTarget > M_PI) angleRobotToTarget -= 2.0 * M_PI;
    if(angleRobotToTarget < -M_PI) angleRobotToTarget += 2.0 * M_PI;

    return angleRobotToTarget;
}

float getvxSaida(float x, float y, SSL_DetectionRobot yellowRobot){
    float dx = ( x - yellowRobot.x());
    float dy = ( y - yellowRobot.y());
    float vxSaida = (dx * cos(yellowRobot.orientation()) + dy * sin(yellowRobot.orientation()));

    return vxSaida;
}

float getvySaida(float x, float y, SSL_DetectionRobot yellowRobot){
    float dx = ( x - yellowRobot.x());
    float dy = ( y - yellowRobot.y());
    float vySaida = (dy * cos(yellowRobot.orientation()) - dx * sin(yellowRobot.orientation()));

    return vySaida;
}

float getPlayerDistanceTo(float x, float y, SSL_DetectionRobot yellowRobot) {
    return sqrt(pow( x - yellowRobot.x(), 2) + pow( y - yellowRobot.y(), 2));
}

void goToLookTo (float targetToGoX, float targetToGoY, SSL_DetectionRobot robot, bool isyellow, float targetToLookX, float targetToLookY, Actuator *actuator, bool kick) {
    float vxToGo = getvxSaida(targetToGoX, targetToGoY, robot);
    float vyToGo = getvySaida(targetToGoX, targetToGoY, robot);
    float angleToLook = getPlayerRotateAngleTo(targetToLookX, targetToLookY, robot);

    if (kick){
        actuator->sendCommand(isyellow, 0, vxToGo/100, vyToGo/100, 10*angleToLook, true, 6.0f, false);
    } else {
       actuator->sendCommand(isyellow, 0, vxToGo/1150, vyToGo/1150, angleToLook, true, 0.0f, false);
    }


}

int main(int argc, char *argv[]) {
    QCoreApplication a(argc, argv);

    Vision *vision = new Vision("224.5.23.2", 10020);
    Actuator *actuator = new Actuator("127.0.0.1", 20011); //172.20.10.2

    // Desired frequency
    int desiredFrequency = 60;

    while(true) {


        // TimePoint
        std::chrono::high_resolution_clock::time_point beforeProcess = std::chrono::high_resolution_clock::now();

        // Process vision and actuator commands
        vision->processNetworkDatagrams();
        //actuator->sendCommand(true, 0, 1.0, 0.0, 0.0);

        SSL_DetectionBall ball = vision->getLastBallDetection();

        SSL_DetectionRobot yellowRobot = vision->getLastRobotDetection(true, 0);
        SSL_DetectionRobot blueRobot = vision->getLastRobotDetection(false, 0);


        //yellowRobot
        float targetX = 2600;
        float targetY = 0;
        float targetToKickY = 400;
        float targetToKickX = 4500;

        float angleRobotToBall = getPlayerRotateAngleTo(ball.x(), ball.y() , yellowRobot);
        float vxSaidaBall = getvxSaida(ball.x(), ball.y() , yellowRobot);
        float vySaidaBall = getvySaida(ball.x(), ball.y() , yellowRobot);
        float distToBall = getPlayerDistanceTo(ball.x(), ball.y() , yellowRobot);


        //float angleRobotToTarget = getPlayerRotateAngleTo(targetX, targetY , yellowRobot);
       // float vxSaidaToTarget = getvxSaida(targetX, targetY, yellowRobot);
       // float vySaidaToTarget = getvySaida(targetX, targetY, yellowRobot);

        float distToTarget = getPlayerDistanceTo(targetX, targetY, yellowRobot);

     //   float angleRobotToKick = getPlayerRotateAngleTo(targetToKickX, targetToKickY, yellowRobot);

        if (ball.x() > 3500 && (ball.y() < 1000 && ball.y() > -1000 )){
              goToLookTo(targetX, targetY, yellowRobot, true, targetToKickX, targetToKickY, actuator, false);
        }else {

        if (angleRobotToBall > 0.2 || angleRobotToBall < -0.2){
            actuator->sendCommand(true, 0, vxSaidaBall/1000, vySaidaBall/1000, angleRobotToBall, true, 0.0f, false);
        } else if (distToBall > 115){
            actuator->sendCommand(true, 0, vxSaidaBall/1000, vySaidaBall/1000, 0, true, 0.0f, false);
        } else if (distToTarget > 50){
            goToLookTo(targetX, targetY, yellowRobot, true, targetToKickX, targetToKickY, actuator, false);
        } else {
            actuator->sendCommand(true, 0, 0, 0, 0, true, 6.0f, false);
        }
    }


       // goToLookTo(targetX, targetY, yellowRobot, true, targetToKickX, targetToKickY, actuator, false);

        //blueRobot goalkeeper
        //float componentX = (getPlayerDistanceTo(ball.x(), ball.y(), blueRobot))*sin(fabs(blueRobot.orientation()) - M_PI/2);
        //float targetGoalkeeperX =  componentX + ball.x();
        //float componentY = (getPlayerDistanceTo(ball.x(), ball.y(), blueRobot))*cos(fabs(blueRobot.orientation()) - M_PI/2);
        //float targetGoalKeeperY = fabs(ball.y()) - componentY;

        float targetGoalKeeperY;
        float targetGoalkeeperX;
        if (ball.x() > 3500 && (ball.y() < 1000 && ball.y() > -1000 )) {
            targetGoalKeeperY = ball.y();
            targetGoalkeeperX = ball.x();
        } else if (ball.y() < 500 && ball.y() > -500){
            targetGoalKeeperY = ball.y();
            targetGoalkeeperX = 4400;
        } else if (ball.y() >= 600) {
            targetGoalKeeperY = 250;
            targetGoalkeeperX = 4400;
        } else if (ball.y() < -600){
            targetGoalKeeperY = -250;
            targetGoalkeeperX =4400;
        } else {
            targetGoalKeeperY = 0;
            targetGoalkeeperX = 4400;
        }


        goToLookTo(targetGoalkeeperX, targetGoalKeeperY, blueRobot, false, ball.x(), ball.y(), actuator, true);

        //std::cout << " distToTarget " << distToBall << std::endl;
        //std::cout << "angleRobotToTarget " << angleRobotToTarget << std::endl;
        std::cout << " ball.x() " << ball.x() << std::endl;
        std::cout << " ball.y() " << ball.y() << std::endl;


        // TimePoint
        std::chrono::high_resolution_clock::time_point afterProcess = std::chrono::high_resolution_clock::now();

        // Sleep thread
        long remainingTime = (1000 / desiredFrequency) - (std::chrono::duration_cast<std::chrono::milliseconds>(afterProcess - beforeProcess)).count();
        std::this_thread::sleep_for(std::chrono::milliseconds(remainingTime));

        }
    return a.exec();
}

/*
 *        if (angleRobotToBall > 0.1 || angleRobotToBall < -0.1){
            actuator->sendCommand(true, 0, vxSaidaBall/1000, vySaidaBall/1000, angleRobotToBall, true, 0.0f, false);
        } else if (distToBall > 113){
            actuator->sendCommand(true, 0, vxSaidaBall/1000, vySaidaBall/1000, 0, true, 0.0f, false);
        } else {
            if (angleRobotToTarget > 0.03 || angleRobotToTarget < -0.03){
                actuator->sendCommand(true, 0, vxSaidaToTarget/1000, vySaidaToTarget/1000, angleRobotToTarget, true, 0.0f, false);
            } else if (distToTarget > 10){
                actuator->sendCommand(true, 0, vxSaidaToTarget/1000, vySaidaToTarget/1000, 0, true, 0.0f, false);
            } else if (angleRobotoToKick > 0.1 || angleRobotoToKick < -0.1){
                actuator->sendCommand(true, 0, 0.0, 0.0, angleRobotoToKick, true, 0.0f, false);
            } else {
                actuator->sendCommand(true, 0, 0.0, 0.0, 0, true, 5.0f, false);
            }
        }

*/




/*
if (angleRobotToTarget > 0.03 || angleRobotToTarget < -0.03){
    actuator->sendCommand(true, 0, vxSaida/1000, vySaida/1000, angleRobotToTarget, false, 0.0f, false);
} else if (distToBall > 103){
    actuator->sendCommand(true, 0, vxSaida/1000, vySaida/1000, 0, false, 0.0f, false);
} else {
    actuator->sendCommand(true, 0, 0.0, 0.0, 1, true, 0.0f, false);
}
*/



/*
//begin getPlayerRotateAngleTo
        float componentX = (targetPositionX - yellowRobot.x());
        float componentY = (targetPositionY - yellowRobot.y());
        float distToTarget = sqrt(pow(componentX, 2) + pow(componentY, 2));

        componentX = componentX / distToTarget;

        // Check possible divisions for 0
            if(isnanf(componentX)) {
                return 0.0f;
            }
        float angleOriginToTarget; // Angle from field origin to targetPosition
        float angleRobotToTarget;  // Angle from robot to targetPosition

        if (componentY < 0.0f){
            angleOriginToTarget = 2*M_PI - acos(componentX); // Angle that the target make with x-axis to robot
        } else {
            angleOriginToTarget = acos(componentX); // Angle that the target make with x-axis to robot
        }

        angleRobotToTarget = angleOriginToTarget - yellowRobot.orientation();

        // Adjusting to rotate the minimum possible
        if(angleRobotToTarget > M_PI) angleRobotToTarget -= 2.0 * M_PI;
        if(angleRobotToTarget < -M_PI) angleRobotToTarget += 2.0 * M_PI;
 //end getPlayerRotateAngleTo
*/


/*
//start goTo
       float dx = (targetPositionX - yellowRobot.x());
       float dy = (targetPositionY - yellowRobot.y());
       float vxSaida = (dx * cos(yellowRobot.orientation()) + dy * sin(yellowRobot.orientation()));
       float vySaida = (dy * cos(yellowRobot.orientation()) - dx * sin(yellowRobot.orientation()));
//end goTo
*/

