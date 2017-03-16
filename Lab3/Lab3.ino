 #include <Sparki.h> // include the sparki library
#include <math.h>
double time1;
double time2;
double timer = 100;

double thetaDOT = .65; //radians / sec   //37.414258989815 degrees / sec
double thetadot;
double xDOT = 0.0278241513638; // meters / sec
double ydot = 0;  // meters / sec
double wdot = 1.094; //radians / sec for wheel rotation (62.76 degrees / s)
double xdot;

double theta = 0;
double realx = 0;
double realy = 0;
double rpower; //wheel rotation speed percentage (0-100)
double lpower;
double rdir = 1; //direction wheels are rotating, 1 CW and -1 CCW
double ldir = -1; 
double headn; //the head value calculated as theta goal - theta real, this is the direction the robot is facing 
double bearing; //direction of destination relative to current course; an updated alpha 
double epsilon = 0.05;

double r = 0.0254; //(meters) radius of the robots wheels 
double d = 0.0857; //(meters) distance between the robots wheels

double alpha;
double rho;

double rwheelSpeed = 0;
double lwheelSpeed = 0;

double destx = 20; //cm (code will convert to meters)
double desty = 15; //cm ("")
double destTheta = 75; //degrees (code will convert to radians)

void setup() {
  // put your setup code here, to run once:
  sparki.clearLCD();
  realx = 0;
  realy = 0;
  theta = 0;
  destx = destx / 100; //convert destination to meters 
  desty = desty / 100;
  destTheta = destTheta * (PI/180); //convert to radians 

  //Calc RHO and ALPHA
  rho = sqrt(pow(realx - destx, 2) + pow(realy - desty, 2));
  alpha = atan2(desty, destx);
}

void loop() {    
    time1 = millis();
    
    //FEEDBACK CONTROL (3.5.1 TEXT)
    headn = destTheta - theta; 
    bearing = theta - atan2(realy - desty, realx - destx);
    
    //USE THESE (xdot thetadot) TO DETERMINE INDIVIDUAL WHEEL SPEEDS FOR FOLLOWING THE CURVE
    xdot = 0.1 * rho;
    if (xdot > xDOT) xdot = xDOT; //xDOT is maximum velocity 
    
    thetadot = 0.1 * (bearing - theta) + 0.01 * (headn); //defined as bearing calculation in textbook and alpha-theta on instructions 
    if (abs(thetadot) > thetaDOT) //thetaDOT maximum turn speed
    {
      if (thetadot > 0) thetadot = thetaDOT; 
      else thetadot = -1 * thetaDOT;
    }
    
    rwheelSpeed = (2 * xdot + (d * thetadot)) / (2 * r ); //for turning (thetadot rather than theta) 
    lwheelSpeed = (2 * xdot - (d * thetadot)) / (2 * r );  //for turning       

    if (abs(rwheelSpeed) > wdot) //wdot max wheel rotational velocity 
    {
       if (rwheelSpeed < 0) rwheelSpeed = -wdot; 
       else rwheelSpeed = wdot;
    }
    if (abs(lwheelSpeed) > wdot)
    {
      if (lwheelSpeed < 0) lwheelSpeed = -wdot;
      else lwheelSpeed = wdot;
    }
    
    rdir = 1; //DIR_CW (rotate forward)
    ldir = -1; //DIR_CCW (rotate forward)

    rpower = (100 - -100)*(rwheelSpeed - -wdot)/(wdot - -wdot) + -100; //scale rad / s to percentage
    lpower = (100 - -100)*(lwheelSpeed - -wdot)/(wdot - -wdot) + -100;

    if (rwheelSpeed < 0)
    {
      rdir = -1; //rotate backward
      rpower = -1 * rpower;
    }
    if (lwheelSpeed < 0)
    {
      ldir = 1; //rotate backward 
      lpower = -1 * lpower;
    }
    if (rho > epsilon)
    {
      sparki.motorRotate(MOTOR_RIGHT, rdir, rpower);  
      sparki.motorRotate(MOTOR_LEFT, ldir, lpower);
    }

    thetadot = r*(rwheelSpeed)/d - r*(lwheelSpeed)/d;
    theta = theta + thetadot / 10; //change over 100ms 
    
    //Calculate movement in x and y
    xdot = r*(lwheelSpeed)/2 + r*(rwheelSpeed)/2; //equation 3.36  
    realx = realx + cos(theta) * xdot / 10; //change over 100ms 
    realy = realy + sin(theta) * xdot / 10;
    
    rho = sqrt(pow(realx - destx, 2) + pow(realy - desty, 2));
     
    sparki.clearLCD(); // wipe the screen
 
   // sparki.print("Left speed: ");
   // sparki.println(lwheelSpeed);
   // sparki.println(lpower);
    
   // sparki.print("Right speed: ");
   // sparki.println(rwheelSpeed);
    
    //sparki.print("Alpha: ");
    //sparki.println(alpha);
    
    sparki.print("X dot: ");
    sparki.println(xdot);
    
    sparki.print("theta dot: ");
    sparki.println(thetadot*(180/PI));
    
    sparki.print("theta: ");
    sparki.println(theta*(180/PI)); //print in degrees
    
    sparki.print("RHO: ");
    sparki.println(rho);
      
    sparki.print("X-Pos: ");
    sparki.println(realx*100); //print in cm
    sparki.print("Y-Pos:  ");
    sparki.println(realy*100);
    
    sparki.updateLCD(); // display all of the information written to the screen 
   
   if (rho < epsilon)
    {
      sparki.moveStop();
      sparki.clearLCD();
      sparki.print("Made it to destination");
      sparki.updateLCD();
      exit(0);
    }

   time2 = millis(); 
   timer = 100 - (time2 - time1);
   if (timer > 0) delay(timer);
   else if (timer < 0) sparki.RGB(RGB_RED); //loop took over 100ms 
}


