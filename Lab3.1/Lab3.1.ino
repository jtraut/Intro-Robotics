#include <Sparki.h> // include the sparki library
#include <math.h>

float time1;
float time2;
float timer = 100.0;

float phidot = 36.1;//37.414258989815;   //degrees / sec
float xdot = 0.0278241513638; // meters / sec
float ydot = 0;  // meters / sec

float xold = 0;
float xnew = 0;
float y = 0;
float phi = 0;
float realx = 0;
float realy = 0;

float objx = 0;
float objy = 0;

int ping = -1;

int nav[4][4];

int xForward = 100;
int xBackward = 100;
int yUp = 100;
int yDown = 100;
int bestPath;

int goalIndex = 15;

struct Coords {
  int x;
  int y;
};

struct Coords goalxy;

void setup() {
  // put your setup code here, to run once:
  sparki.clearLCD();
  sparki.servo(-90);

  goalxy = indexToxy[goalIndex];

  for (int i = 0; i < 4; i++) {
    for (int k = 0; k < 4; k++) {
      nav[i][k] = 1; // no obstacles found yet
    }
  }
}

// TODO use the three functions at the end to implement Dijkstra's algorithm
// TODO check that objx and objy are returning correct values (Lines 92 and 93)
void loop() {
  time1 = millis();
  sparki.RGB(RGB_OFF);
  int threshold = 500;

  int lineLeft   = sparki.lineLeft();   // measure the left IR sensor
  int lineCenter = sparki.lineCenter(); // measure the center IR sensor
  int lineRight  = sparki.lineRight();  // measure the right IR sensor

  if (lineRight < threshold && lineCenter < threshold && lineLeft < threshold) 
  {
    sparki.moveForward();
  }
  
  else if ( lineLeft < threshold ) // if line is below left line sensor
  {  
    sparki.moveLeft(); // turn left
    phi = phi + (0.2 * phidot); // change this back to 0.1 if timer/loop goes back to 100ms
  }

  else if ( lineRight < threshold ) // if line is below right line sensor
  {  
    sparki.moveRight(); // turn right
    phi = phi - (0.2 * phidot); // change this back to 0.1 if timer/loop goes back to 100ms
  }

  // if the center line sensor is the only one reading a line -> move forward
  else
  {
    sparki.moveForward(); 
  }  

  xnew = xold + ((200/10) * xdot);
  float xchange = xnew - xold;
  realx = realx + cos(phi*3.14159/180)*xchange;
  realy = realy + sin(phi*3.14159/180)*xchange;
  xold = xnew;

  ping = sparki.ping();
  if (ping < 20) 
  {
    // The following need to return values between 0 and 60 for x and 0 and 40 for y (check!!)
    objx = realx + cos((phi-90)*3.14159/180) * ping;
    objy = realy + sin((phi-90)*3.14159/180) * ping;
  }

  // call first function to get object index
  if (objx > 60) objx = 60;
  if (objy > 40) objy = 40;
  int objIndex = xyToIndex(objx, objy);

  // call first function to get real index
  if (realx > 60) realx = 60;
  if (realy > 40) realy = 40;
  int sparkiIndex = xyToIndex(realx, realy);

  int direction[4] = {100, 100, 100, 100}; //index 0 forward, 1 back, 2 down, 3 up
  if (sparkiIndex % 4 != 0) // if sparki isn't at the furthest column (it can move forward)
  { 
    direction[0] = cost(sparkiIndex + 1, objIndex); 
  }
  if (sparkiIndex % 4 != 1) // if sparki isn't at the first column (it can move "backward")
  {
    direction[1] = cost(sparkiIndex - 1, objIndex); 
  } 
  if (sparkiIndex > 4) // if sparki isn't at the bottom row (it can move down)
  {
    direction[2] = cost(sparkiIndex - 4, objIndex);
  } 
  if (sparkiIndex < 12) // if sparki isn't at the top row (it can move up)
  { 
    direction[3] = cost(sparkiIndex + 4, objIndex); 
  }
  // get the lowest cost 
  int sparkiGoTo, dir;
  bestPath = 1000;
  struct Coords coord;
  for (int i = 0; i < 4; i++)
  {
    if (direction[i] == 1)
    {
      if (i == 0) dir = sparkiIndex + 1; //forward
      else if (i == 1) dir = sparkiIndex - 1; //backward
      else if (i == 2) dir = sparkiIndex -4; //down
      else dir = sparkiIndex + 4; //up 

      coord = indexToxy(dir);
      int x = coord.x;
      int gx = goalxy.x;
      int y = coord.y;
      int gy = goalxy.y;
      float dist = sqrt(pow(x - gx, 2) + pow(y - gy, 2));
      if (dist < bestPath)
      {
        bestPath = dist;
        sparkiGoTo = dir; 
      }
      
    }
  }
  

  sparki.clearLCD(); // wipe the screen

//  sparki.print("ObjX: ");
//  sparki.println(objx);
//
//  sparki.print("Objy: ");
//  sparki.println(objy);
//
//  sparki.print("Phi: ");
//  sparki.println(phi);
//
  sparki.print("X-Pos: ");
  sparki.println(realx); // 60
//  sparki.print("Y-Pos:  "); // 40
//  sparki.println(realy);
//
//  sparki.print("Ping: ");
//  sparki.println(ping);

  sparki.updateLCD(); // display all of the information written to the screen

  time2 = millis();
  timer = 200 - (time2 - time1);
  if (timer > 0) delay(timer);
  else if(timer < 0) sparki.RGB(RGB_RED);

}

int xyToIndex(float objx, float objy) {
  int xIndex = ceil(objx / 15); 
  int yIndex = ceil(objy / 10);

  nav[xIndex - 1][yIndex - 1] = 0;

  int index = 0;

  for (int i = 1; i < yIndex; i++) 
  {
    index = xIndex + 4;
  }
  
  return index;
}

struct Coords indexToxy(int index) {
  struct Coords coords;

  if (index > 0 && index < 5) coords.y = 0;
  else if (index > 4 && index < 9) coords.y = 1;
  else if (index > 8 && index < 13) coords.y = 2;
  else coords.y = 3;

  if (index % 4 == 1) coords.x = 0;
  else if (index % 4 == 2) coords.x = 1;
  else if (index % 4 == 3) coords.x = 2;
  else if (index % 4 == 0) coords.x = 3;
  
  return coords;
}

int cost(int destIndex, int objIndex) {
  struct Coords destCoords = indexToxy(destIndex); 

  // if we have detected an object at that index
  if (nav[destCoords.x][destCoords.y] == 1) return 99;
  return 1;
}

