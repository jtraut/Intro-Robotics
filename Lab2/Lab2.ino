#include <Sparki.h> // include the sparki library
float time1;
float time2;
float timer;

float phidot = 36.1;//37.414258989815;   //degrees / sec
float xdot = 0.0278241513638; // meters / sec
float ydot = 0;  // meters / sec

float xold = 0;
float xnew = 0;
float y = 0;
float phi = 0;
float realx = 0;
float realy = 0;

void setup() {
  // put your setup code here, to run once:
  sparki.clearLCD();
  Serial.begin(9600);
}



void loop() {
  int threshold = 500;

  int lineLeft   = sparki.lineLeft();   // measure the left IR sensor
  int lineCenter = sparki.lineCenter(); // measure the center IR sensor
  int lineRight  = sparki.lineRight();  // measure the right IR sensor

  if (lineRight < threshold && lineCenter < threshold && lineLeft < threshold) 
  {
    sparki.println("START DETECTED");
    sparki.updateLCD();
    realx = 0;
    realy = 0;
    phi = 0;
    sparki.moveForward(2);
  }
  
  else if ( lineLeft < threshold ) // if line is below left line sensor
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

  else if ( lineRight < threshold ) // if line is below right line sensor
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

  // if the center line sensor is the only one reading a line -> move forward
  else
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

