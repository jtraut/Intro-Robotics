#include <Sparki.h> // include the sparki library
float time1;
float time2;
float timer;

float phidot = 36.1;//37.414258989815;   //degrees / sec
float xdot = 0.0278241513638; // meters / sec
float ydot = 0;  // meters / sec

float xold = 0;  //to keep track of real(x and y) 
float xnew = 0;
float y = 0;  
float phi = 0;
float realx = 0;
float realy = 0;

void setup() {
  // put your setup code here, to run once:
  sparki.clearLCD();
  realx = 0;
  realy = 0;
  phi = 0;

  float destx = 5;
  float desty = 8;
  float destphi = 180;

  float r = 0.0254; //(meters) radius of the robots wheels 
  float d = 0.086; //(meters) distance between the robots wheels 
  


  //Calc RHO and ALPHA
  RHO = ((realx - destx)^2 + (realy - desty)^2)^(1/2);
  alpha = atan2(destx, desty) * (180/PI);
}

turnSpeed = 0.1 * (alpha - phidot) + 0.01(destphi - phidot)
int rwheelSpeed = (2 * xdot + (d* phi)) / (2 * r); //for turning 
int lwheelSpeed = (2 * xdot - (d* phi)) / (2 * r);  //for turning 

alpha = atan2(destx, desty) * (180/PI);
motorRotate(MOTOR_LEFT, DIR_CCW, lwheelspeed);
motorRotate(MOTOR_RIGHT, DIR_CW, rwheelspeed);


float forwardSpeed = 0.1 * RHO;


NOTES
One spot to move forward 
calc wheel speed.  Smooth stuff.  Recalculate position and alpha every 100ms.  








void loop() {
epislon = 0.1;
while ( alpha - phi < epislon )   //turn until facing objective 
{
  if ( alpha - phi  < 0 ) // turn left - negative angle 
  {  
    time1 = millis();
    sparki.moveLeft(); // turn left
    delay(100);
    sparki.moveStop();
    time2 = millis();

    //Calculate movement in phi
    timer = time2 - time1;
    phi = phi - ((timer / 1000) * phidot);
  }

  else ( alpha - phi > 0 ) // turn right - positive angle 
  {  
    time1 = millis();
    sparki.moveRight(); // turn right
    delay(100);
    sparki.moveStop();
    time2 = millis();

    //Calculate movement in phi
    timer = time2 - time1;
    phi = phi + ((timer / 1000) * phidot);
  }
}

//drive forward RHO
sparki.moveForward()


while (destx - realx > epislon
  {
    time1 = millis();
    sparki.moveForward(); // move forward
    delay(100);
    sparki.moveStop();
    time2 = millis();

    //Calculate movement in x
    timer = time2 - time1;
    xnew = xold + ((timer/10) * xdot);
    float xchange = xnew - xold;
    realx = realx + cos(phi*3.14159/180)*xchange;
    realy = realy + sin(phi*3.14159/180)*xchange;
    xold = xnew;
  }  

  sparki.clearLCD(); // wipe the screen

  sparki.print("Line Left: "); // show left line sensor on screen
  sparki.println(lineLeft);

  sparki.print("Line Center: "); // show center line sensor on screen
  sparki.println(lineCenter);

  sparki.print("Line Right: "); // show right line sensor on screen
  sparki.println(lineRight);

  sparki.print("Phi: ");
  sparki.println(phi);

  sparki.print("X-Pos: ");
  sparki.println(realx);
  sparki.print("Y-Pos:  ");
  sparki.println(realy);

  sparki.updateLCD(); // display all of the information written to the screen

}

