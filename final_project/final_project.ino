#include <Sparki.h>

#define FINDBALL 0
#define GRABBALL 1
#define POSITION 2
#define SHOOT 3

int state = FINDBALL;
int servoAngle = -90;
int distance = 0;
bool detected = false;
int objectDistance [20];
int angleDetected [20];
int i = 0;
bool first_detect = false;
int avgDistance = 0;
int angleToObject = 0;
int counter = 0;
   
void setup() {
  // put your setup code here, to run once:
  sparki.gripperOpen();
  sparki.servo(servoAngle); 
}

void loop() {
  // put your main code here, to run repeatedly:

  switch(state)
  {
    case FINDBALL:
      distance = 100;
      while (not detected)
      {
        
        distance = sparki.ping();
        
        if (distance < 27)
        {
          first_detect = true;
          if (i < 20)
          {
            objectDistance[i] = distance;
            angleDetected[i] = servoAngle;
            i += 1;
          }
        }

       if (distance > 30 and first_detect)
       {
         detected = true;
       }
        
        servoAngle = servoAngle + 1;
        sparki.servo(servoAngle);
        Serial.print("distance: ");
        Serial.println(distance);
        Serial.print("servoAngle: ");
        Serial.println(servoAngle); 
      }
  
      for (i = 0; i < 20; i++)
      {
         if (objectDistance[i] < 30)
         {
           avgDistance = avgDistance + objectDistance[i];       
           angleToObject = angleToObject + angleDetected[i];
           counter++;
         } 
      }
    
      avgDistance = avgDistance/counter - 4; //account for tall object behind ball
      angleToObject = angleToObject/counter; 
      sparki.clearLCD();

      sparki.print("distance to object: ");
      sparki.println(avgDistance);
      sparki.print("angle to object: ");
      sparki.println(angleToObject);

      sparki.updateLCD();
      state = GRABBALL;
      break;
      

    case GRABBALL:
      sparki.print("in case GRABBALL");
      if (servoAngle < 0)
      {
        sparki.moveLeft(servoAngle);
        sparki.print("turning to the left");
      }
      else sparki.moveRight(servoAngle);

      sparki.moveForward(distance);

      sparki.updateLCD();
      
      sparki.gripperClose();
      delay(1000);
      sparki.gripperStop();

      state = POSITION;
      break;

     default:
        sparki.RGB(RGB_RED);
        break;
  }
  
}
