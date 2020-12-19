//author  Renato Sousa, 2018
//#include <QtNetwork>
#include <stdio.h>
#include <iostream>
#include "net/robocup_ssl_client.h"
#include "net/grSim_client.h"
#include "util/timer.h"
#include "util/util.h"
#include <math.h>
#include <time.h>


class Objective{
    double m_x;
    double m_y;
    double m_angle;

public:
    Objective(double t_x, double t_y, double t_angle): m_x(t_x),m_y(t_y),m_angle(t_angle){};


    void setY(double value){
        m_y = value;
    }

    void setAngle(double value){
        m_angle = value;
    }

    void setX(double value){
        m_x = value;
    }
    double x(){
        return m_x;
    }
    double y(){
        return m_y;
    }
    double angle(){
        return m_angle;
    }
};


void printRobotInfo(const fira_message::sim_to_ref::Robot & robot) {

    printf("ID=%3d \n",robot.robot_id());

    printf(" POS=<%9.2f,%9.2f> \n",robot.x(),robot.y());
    printf(" VEL=<%9.2f,%9.2f> \n",robot.vx(),robot.vy());

    printf("ANGLE=%6.3f \n",robot.orientation());
    printf("ANGLE VEL=%6.3f \n",robot.vorientation());
}

double to180range(double angle) {
  angle = fmod(angle, 2 * M_PI);
  if (angle < -M_PI) {
    angle = angle + 2 * M_PI;
  } else if (angle > M_PI) {
    angle = angle - 2 * M_PI;
  }
  return angle;
}

double smallestAngleDiff(double target, double source) {
  double a;
  a = fmod(target + 2*M_PI, 2 * M_PI) - fmod(source + 2*M_PI, 2 * M_PI);

  if (a > M_PI) {
    a = a - 2 * M_PI;
  } else if (a < -M_PI) {
    a = a + 2 * M_PI;
  }
  return a;
}

// PID GOLEIRO
void PID(fira_message::sim_to_ref::Robot robot, Objective objective, int index,GrSim_Client* grSim_client, bool isYellow = false) 
{

    double Kp = 30;
    double Kd = 2.5;
    static double lastError = 0;

    double rightMotorSpeed;
    double leftMotorSpeed;

    bool reversed = false;

    double angle_rob = robot.orientation();


    double angle_obj = atan2( objective.y() - robot.y(),  objective.x() - robot.x()) ;

    
    double error = smallestAngleDiff(angle_rob, angle_obj);

    if(fabs(error) > M_PI/2.0 + M_PI/20.0) {
          reversed = true;
          angle_rob = to180range(angle_rob+M_PI);
          // Calculates the error and reverses the front of the robot
          error = smallestAngleDiff(angle_rob,angle_obj);
    }

    double motorSpeed = (Kp*error) + (Kd * (error - lastError));// + 0.2 * sumErr;
    lastError = error;



    double baseSpeed = 30;

    

    // Normalize
    motorSpeed = motorSpeed > 30 ? 30 : motorSpeed;
    motorSpeed = motorSpeed < -30 ? -30 : motorSpeed;

    if (motorSpeed > 0) {
      leftMotorSpeed = baseSpeed ;
      rightMotorSpeed = baseSpeed - motorSpeed;
    } else {
      leftMotorSpeed = baseSpeed + motorSpeed;
      rightMotorSpeed = baseSpeed;
    }


    if (reversed) {
      if (motorSpeed > 0) {
        leftMotorSpeed = -baseSpeed + motorSpeed;
        rightMotorSpeed = -baseSpeed ;
      } else {
        leftMotorSpeed = -baseSpeed ;
        rightMotorSpeed = -baseSpeed - motorSpeed;
      }
    }
    grSim_client->sendCommand(leftMotorSpeed,rightMotorSpeed, isYellow, index);
}







//PID ATACANTE
void PID2(fira_message::sim_to_ref::Robot robot, Objective objective, int index,GrSim_Client* grSim_client, bool isYellow = false)
{
    double Kp = 20;
    double Kd = 2.5;
    static double lastError = 0;

    double rightMotorSpeed;
    double leftMotorSpeed;

    bool reversed = false;

    double angle_rob = robot.orientation();


    double angle_obj = atan2( objective.y() - robot.y(),  objective.x() - robot.x()) ;


    double error = smallestAngleDiff(angle_rob, angle_obj);

    if(fabs(error) > M_PI/2.0 + M_PI/20.0) {
          reversed = true;
          angle_rob = to180range(angle_rob+M_PI);
          // Calculates the error and reverses the front of the robot
          //error = smallestAngleDiff(angle_rob,angle_obj);
    }

    double motorSpeed = (Kp*error) + (Kd * (error - lastError));// + 0.2 * sumErr;
    lastError = error;



    double baseSpeed = 30;

    // Normalize
    motorSpeed = motorSpeed > 30 ? 30 : motorSpeed;
    motorSpeed = motorSpeed < -30 ? -30 : motorSpeed;

    if (motorSpeed > 0) {
      leftMotorSpeed = baseSpeed ;
      rightMotorSpeed = baseSpeed - motorSpeed;
    } else {
      leftMotorSpeed = baseSpeed + motorSpeed;
      rightMotorSpeed = baseSpeed;
    }


    if (reversed) {
      if (motorSpeed > 0) {
        leftMotorSpeed = -baseSpeed + motorSpeed;
        rightMotorSpeed = -baseSpeed ;
      } else {
        leftMotorSpeed = -baseSpeed ;
        rightMotorSpeed = -baseSpeed - motorSpeed;
      }
    }
    grSim_client->sendCommand(leftMotorSpeed,rightMotorSpeed, isYellow, index);
}

double x,y,ang;

// GOLEIRO AZUL
Objective defineObjective(fira_message::sim_to_ref::Robot robot, fira_message::sim_to_ref::Ball ball)   
{   
    //Implementar a estratégia aqui
    //Deve retornar o objetivo do robô
    printf("BIGODEEEEEEEEEEEEEEEEEEEE %f %f\n\n", robot.x(), robot.y());
    double gol_x = 151;   // X da barra(gol)
    x = gol_x;
    y = ball.y();
    ang = M_PI/2;

    if (y < 43){    // Limite para nao sair do gol
      return Objective(x, 40, M_PI/2);
     
    }
    if (y > 90){
      return Objective(x, 90, M_PI/2);
    }
    
    ang = M_PI/2;
    
    printf("TANGENTEEEEE %f\n\n", ang);

    return Objective(x, y, ang);
}


// DEFENSOR AZUL
Objective defineObjective5(fira_message::sim_to_ref::Robot robot, fira_message::sim_to_ref::Ball ball)   
{   
    //Implementar a estratégia aqui
    //Deve retornar o objetivo do robô
    printf("BIGODEEEEEEEEEEEEEEEEEEEE %f %f\n\n", robot.x(), robot.y());

    // Calcular a distancia da bola
    double DistanciaBola = sqrt( pow( robot.x() - ball.x(),2) + pow(robot.y() - ball.y(),2) );
    double defender_x = 125;   // X da linha de defesa
    

    if (y < 15){    // Limite para nao ficar preso na parede de baixo
      return Objective(defender_x, 15, M_PI/2);
     
    }
    if (y > 120){   // Limite para nao ficar preso na parede de cima
      return Objective(defender_x, 120, M_PI/2);
    }
    
    if (DistanciaBola < 25 && ball.x() < 125){  // Jogar a bola p frente caso esteja mt perto
      x = ball.x();
      y = ball.y();
      if ( ball.x() < 65){
        x = defender_x;
      }
    } else{
      x = defender_x;
      y = ball.y();
    }
    
    printf("TANGENTEEEEE %f\n\n", DistanciaBola);

    return Objective(x, y, ang);
}



// ATANCANTE AZUL////////////////////////////////////////////////////////////////////////////////////////////
Objective defineObjective2(fira_message::sim_to_ref::Robot robot, fira_message::sim_to_ref::Ball ball) 
{   
    printf("BIGODEEEEEEEEEEEEEEEEEEEE %f %f\n\n", robot.x(), robot.y());
    double DistanciaBola = sqrt( pow( robot.x() - ball.x(),2) + pow(robot.y() - ball.y(),2) );//Calcular distancia da bola
    
    x = ball.x();
    y = ball.y();

    if(ball.x() > robot.x() && robot.y() > ball.y()){  //Dar a volta na bola por cima
      y = ball.y() + 10;
      x = ball.x();
    }

    if (ball.x() > robot.x() && robot.y() < ball.y()){   //Dar a volta na bola por baixo
      y = ball.y() - 10;
      x = ball.x();   
    }


    // Por que deixar o robô longe das paredes?
    // Para não ficar preso nas mesmas
    
    if (robot.x() < 30){   // Deixa o robô longe da área e da parede
      x = 30;}
    
    if (robot.y() < 15){  // Deixa o robô longe da parede de baixo
      y = 15;}
    
    if (robot.y() > 115){  // Deixa o robô longe da parede de cima
      y = 115;}

    if (robot.x() > 135){  // Deixa o robô longe do próprio gol
      x = 135;}
    
    if (DistanciaBola > 25 && ball.y() < 85 && ball.y() > 50 && ball.x() < robot.x() - 20){ //Aliando o robo com a bola
      return Objective(robot.x() - 10, ball.y(), ang);
    }
    
    ang = M_PI/2;
    printf("TANGENTEEEEE %f\n\n", DistanciaBola);

    return Objective(x, y, ang);
}



// GOLEIRO AMARELO //////////////////////////////////////////////////////////////////////////////////////////
Objective defineObjective3(fira_message::sim_to_ref::Robot robot, fira_message::sim_to_ref::Ball ball) 
{   
    printf("BIGODEEEEEEEEEEEEEEEEEEEE %f %f\n\n", robot.x(), robot.y());
    double gol_x = 18;   // X da barra(gol)
    x = gol_x;
    y = ball.y();
    ang = M_PI/2;

    if (y < 43){    // Limite para nao sair do gol
      return Objective(x, 40, M_PI/2);
    }
    if (y > 90){
      return Objective(x, 90, M_PI/2);
    }
    ang = M_PI/2;
    
    printf("TANGENTEEEEE %f\n\n", ang);

    return Objective(x, y, ang);
}


// ATANCANTE AMARELO ///////////////////////////////////////////////////////////////////////////////////////
Objective defineObjective4(fira_message::sim_to_ref::Robot robot, fira_message::sim_to_ref::Ball ball) 
{   
    printf("BIGODEEEEEEEEEEEEEEEEEEEE %f %f\n\n", robot.x(), robot.y());
    double DistanciaBola = sqrt( pow( robot.x() - ball.x(),2) + pow(robot.y() - ball.y(),2) );//Calcular distancia da bola
    
    x = ball.x();
    y = ball.y();

    if(ball.x() < robot.x() && robot.y() > ball.y()){  //Dar a volta na bola por cima
      y = ball.y() + 10;
      x = ball.x();
    }

    if (ball.x() < robot.x() && robot.y() < ball.y()){   //Dar a volta na bola por baixo
      y = ball.y() - 10;
      x = ball.x();
        
    }
    // Por que deixar o robô longe das paredes?
    // Para não ficar preso nas mesmas
    
    if (robot.x() > 145){   // Deixa o robô longe da área e da parede
      x = 145;
    }
    if (robot.y() < 15){  // Deixa o robô longe da parede de baixo
      y = 15;
    }
    if (robot.y() > 115){  // Deixa o robô longe da parede de cima
      y = 115;
    }
    if (robot.x() < 35){  // Deixa o robô longe do próprio gol
      x = 35;
    }

    if (DistanciaBola > 25 && ball.y() < 85 && ball.y() > 50 && ball.x() > robot.x() - 20){ //Aliando o robo com a bola
      return Objective(robot.x() + 10, ball.y(), ang);
    }
    ang = M_PI/2;
    printf("TANGENTEEEEE %f\n\n", ang);

    return Objective(x, y, ang);
}


// DEFENSOR AMARELO
Objective defineObjective6(fira_message::sim_to_ref::Robot robot, fira_message::sim_to_ref::Ball ball)   
{   
    //Implementar a estratégia aqui
    //Deve retornar o objetivo do robô
    printf("BIGODEEEEEEEEEEEEEEEEEEEE %f %f\n\n", robot.x(), robot.y());

    double DistanciaBola = sqrt( pow( robot.x() - ball.x(),2) + pow(robot.y() - ball.y(),2) );//Calcular a distancia da bola
    double defender_x = 44;   // X da linha de defesa
    

    if (y < 15){    // Limite para nao ficar preso na parede de baixo
      return Objective(defender_x, 15, M_PI/2);
     
    }
    if (y > 120){   // Limite para nao ficar preso na parede de cima
      return Objective(defender_x, 120, M_PI/2);
    }
    
    if (DistanciaBola < 25 && ball.x() > 28){  // Jogar a bola p frente caso esteja mt perto
      x = ball.x();
      y = ball.y();
      if ( ball.x() > 100){
        x = defender_x;
      }
    } else{
      x = defender_x;
      y = ball.y();
    }
    
    printf("TANGENTEEEEE %f\n\n", DistanciaBola);

    return Objective(x, y, ang);
}






int main(int argc, char *argv[]){
    (void)argc;
    (void)argv;
    //define your team color here
    bool my_robots_are_yellow = false;
    
    // the ip address need to be in the range 224.0.0.0 through 239.255.255.255
    RoboCupSSLClient *visionClient = new RoboCupSSLClient("224.0.0.1", 10002);
    visionClient->open(false);


    GrSim_Client *commandClient = new GrSim_Client();
    

    fira_message::sim_to_ref::Environment packet;

    while(true) {
        if (visionClient->receive(packet)) {
            printf("-----Received Wrapper Packet---------------------------------------------\n");
            //see if the packet contains a robot detection frame:
            if (packet.has_frame()) {
                fira_message::sim_to_ref::Frame detection = packet.frame();



                int robots_blue_n =  detection.robots_blue_size();
                int robots_yellow_n =  detection.robots_yellow_size();
                double width,length;
                width = 1.3/2.0;
                length = 1.7/2.0;

                //Ball info:

                fira_message::sim_to_ref::Ball ball = detection.ball();
                ball.set_x((length+ball.x())*100);
                ball.set_y((width+ball.y())*100);
                printf("-Ball:  POS=<%9.2f,%9.2f> \n",ball.x(),ball.y());



                //Blue robot info:
                for (int i = 0; i < robots_blue_n; i++) {
                    fira_message::sim_to_ref::Robot robot = detection.robots_blue(i);
                    robot.set_x((length+robot.x())*100);//convertendo para centimetros
                    robot.set_y((width+robot.y())*100);
                    robot.set_orientation(to180range(robot.orientation()));
                    printf("-Robot(B) (%2d/%2d): ",i+1, robots_blue_n);
                    printRobotInfo(robot);
                    if(i==0){ //ATACANTE
                        Objective o = defineObjective2(robot,ball);
                        PID2(robot,o,i,commandClient, false);
                    }
                    if(i==1){ //GOLEIRO
                        Objective o = defineObjective(robot,ball);
                        PID(robot,o,i,commandClient, false);
                    }
                    if(i==2){ //DEFENSOR
                        Objective o = defineObjective5(robot,ball);
                        PID(robot,o,i,commandClient, false);
                    }
                }

                //Yellow robot info:
                for (int i = 0; i < robots_yellow_n; i++) {
                    fira_message::sim_to_ref::Robot robot = detection.robots_yellow(i);
                    robot.set_x((length+robot.x())*100);//convertendo para centimetros
                    robot.set_y((width+robot.y())*100);
                    robot.set_orientation(to180range(robot.orientation()));
                    printf("-Robot(Y) (%2d/%2d): ",i+1, robots_yellow_n);
                    printRobotInfo(robot);
                    if(i==0){ //GOLEIRO
                        Objective o = defineObjective3(robot,ball);
                        PID(robot,o,i,commandClient, true);
                    }
                    if(i==1){ //ATACANTE
                        Objective o = defineObjective4(robot,ball);
                        PID2(robot,o,i,commandClient, true);
                    }
                    if(i==2){ //DEFENSOR
                        Objective o = defineObjective6(robot,ball);
                        PID(robot,o,i,commandClient, true);
                    }
                }

            }

            //see if packet contains geometry data:
            /*if (packet.has_field()){
                printf("-[Geometry Data]-------\n");

                const fira_message::sim_to_ref::Field & field = packet.field();
                printf("Field Dimensions:\n");
                printf("  -field_length=%f (mm)\n",field.length());
                printf("  -field_width=%f (mm)\n",field.width());
                printf("  -goal_width=%f (mm)\n",field.goal_width());
                printf("  -goal_depth=%f (mm)\n",field.goal_depth());



            }*/
        }
    }

    return 0;
}


