#include <QCoreApplication>
#include <chrono>
#include <thread>
#include <modules/actuator/actuator.h>
#include <modules/vision/vision.h>
#include <math.h>

float getPlayerRotateAngleTo(float x, float y, fira_message::Robot Robot) {
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

float getPlayerDistanceTo(float x, float y, fira_message::Robot Robot) {
    return sqrt(pow( x - Robot.x(), 2) + pow( y - Robot.y(), 2));
}


float getvelAngularRight (float _velX, float _teta){
    float L = 75; //milimetros
    float r = 20;  //milimetros
    float wRight = ((2* _velX) + (L* _teta)) / 2*r;

    return wRight;

}

float getvelAngularLeft (float velX, float teta){
    float L = 75; //milimetros
    float r = 20; //milimetros
    float wLeft = ((2*velX) - (L*teta)) / 2*r;

    return wLeft;
}

float getVxToTarget(float targetX, float targetY, fira_message::Robot Robot){

    // Define a velocidade do robô para chegar na bola
    float dx = (targetX - Robot.x());
    float dy = (targetY - Robot.y());

    // Pegando modulo do vetor distancia
    float distanceMod = sqrtf(powf(dx, 2.0) + powf(dy, 2.0));

    return distanceMod;
}

int main(int argc, char *argv[]) {
    QCoreApplication a(argc, argv);

    Vision *vision = new Vision("224.0.0.1", 10002);
    Actuator *actuator = new Actuator("127.0.0.1", 20011);

    // Desired frequency
    int desiredFrequency = 60;

    while(true) {
        // TimePoint
        std::chrono::high_resolution_clock::time_point beforeProcess = std::chrono::high_resolution_clock::now();

        // Process vision and actuator commands
        vision->processNetworkDatagrams();
        //actuator->sendCommand(true, 0, 20.0, 20.0);

        fira_message::Robot yellowRobot = vision -> getLastRobotDetection(true, 0);
        fira_message::Robot blueRobot = vision -> getLastRobotDetection(false, 0);
        fira_message::Ball ball = vision->getLastBallDetection();


        float yellowVeloX;
        float tetaToBall = getPlayerRotateAngleTo(ball.x(), ball.y(), yellowRobot);

        if (tetaToBall > -1.5 && tetaToBall < 1.5 ){
            yellowVeloX = 100;
        } else {
            yellowVeloX = -100;
        }

        float wRightToBall = getvelAngularRight(yellowVeloX, tetaToBall);
        float wLeftToBall = getvelAngularLeft(yellowVeloX, tetaToBall);
        float distanceToBall = getPlayerDistanceTo(ball.x(), ball.y(), yellowRobot);

       // float tetaToTarget = getPlayerRotateAngleTo(-1000, 0, yellowRobot);
       // float wRightToTarget = getvelAngularRight(90, tetaToTarget);
       // float wLeftToTarget = getvelAngularLeft(90, tetaToTarget);


        if(100*distanceToBall > 8){
           actuator->sendCommand(true, 0, (wLeftToBall/50), (wRightToBall/50));
        } else if (yellowRobot.x() < -0.5 && 100*distanceToBall < 8  ){
           actuator->sendCommand(true, 0, 700, -700);
        }



        //goalkeeper team Blue


        float veloX, lookToX, lookToY;
        float tetaKeeper = getPlayerRotateAngleTo(lookToX, lookToY, blueRobot);
        float wRightKeeper = getvelAngularRight(veloX, tetaKeeper);
        float wLeftKeeper = getvelAngularLeft(veloX, tetaKeeper);

        if (blueRobot.x() > -1 || blueRobot.x() < -1.2){
            if (tetaKeeper > -1.5 && tetaKeeper < 1.5 ){
                veloX = 80;
                lookToX = -1.05;
                lookToY = 0;
            } else {
                veloX = -80;
                lookToX = -1.05;
                lookToY = 0;
            }

        } else {
            lookToX = -1.05;
            lookToY = 1;
            if (ball.y() < 0.23 && ball.y() > -0.23){
                veloX = 1000*(ball.y()-blueRobot.y());
            } else if (ball.y() > 0.23){
                veloX = 1000*(0.15-blueRobot.y());
            } else if (ball.y() < -0.23){
                veloX = 1000*(-0.15-blueRobot.y());
            } else {
                veloX = 0;
            }

        }


        if(tetaKeeper > 0.1 || tetaKeeper < -0.1){
            actuator ->sendCommand(false, 0, wLeftKeeper/25, wRightKeeper/25);
        } else {
            actuator ->sendCommand(false, 0, wLeftKeeper/25, wRightKeeper/25);
        }



        //std::cout << " velX=  " << velX << std::endl;
        std::cout << " tetaToBall= " << tetaToBall<< std::endl;

        //std::cout << " robot.y()= " << blueRobot.y() << std::endl;
        //std::cout << " robot.x()= " << blueRobot.x() << std::endl;

        // TimePoint
        std::chrono::high_resolution_clock::time_point afterProcess = std::chrono::high_resolution_clock::now();

        // Sleep thread
        long remainingTime = (1000 / desiredFrequency) - (std::chrono::duration_cast<std::chrono::milliseconds>(afterProcess - beforeProcess)).count();
        std::this_thread::sleep_for(std::chrono::milliseconds(remainingTime));
    }

    return a.exec();
}
