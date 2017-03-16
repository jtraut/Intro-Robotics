#include <Sparki.h> // include the sparki library
#include <math.h>

#define MOVE 0
#define CLOSE 1
#define STOP 2

#define FORWARD 1 //positive y
#define BACKWARD 2 //negative y
#define UP 3 //neg x
#define DOWN 4 //pos x 

int STATE = 0, DIR = 1;
int prevDIR = 1;

double dist = 0.0841; //distance between the wheels
double rad = 0.0258; //radius of the wheels
double xDot, thetaDot, phiR, phiL;
double xR = .075; // hardcoded from starting node 
double yR = .05; // hardcoded from starting node 
double thetaR = PI/2;

double xDest, yDest, tDest;
double rpower, lpower;
double rdir = 1; //direction wheels are rotating, 1 CW and -1 CCW
double ldir = -1; 

double rho, alpha, threshold, bearing, heading;

int tempParent;
int prevNode;

double thetaDOT = .65; //radians / sec     //37.414258989815;   //degrees / sec
double xDOT = 0.0278241513638; // meters / sec
double ydot = 0;  // meters / sec
double wdot = 1.094; //radians / sec for wheel rotation (62.76 degrees / s)

double xold = 0;
double xnew = 0;
double y = 0;
double phi = 0;
double xGoal, yGoal;

double objx = 0;
double objy = 0;

double time1, time2, timer;

int nav[4][4];
int shortestParent[15];
int testDist[15];
int currNode = 0; // starting point on map
int destNode = 11; // goal on map

struct Coords {
  int x;
  int y;
};

struct RealCoords {
  double x; 
  double y;
};

struct RealCoords realcoords;
struct RealCoords destcoords;

void setup() {
  // put your setup code here, to run once:
  sparki.clearLCD();
  //sparki.servo(-90);

  for (int i = 0; i < 4; i++) {
    for (int k = 0; k < 4; k++) {
      nav[i][k] = 1; // no obstacles found yet
    }
  }

  nav[1][0] = 0;
  nav[1][1] = 0;
  nav[1][2] = 0;
  nav[3][0] = 0;
  nav[3][1] = 0;
  nav[3][3] = 0;

  for (int i = 0; i <= 15; i++) {
    shortestParent[i] = -1;
  }

  destcoords = indexToxRy(destNode); //store the x,y goal 
  xGoal = destcoords.x;
  yGoal = destcoords.y;
  
  dij(15, destNode, testDist, shortestParent);
  if (shortestParent[0] != -1) {
    currNode = shortestParent[0];
    destcoords = indexToxRy(currNode);
    xDest = destcoords.x;
    yDest = destcoords.y;
  }
  
  if (xR > xDest){
    DIR = DOWN; 
    tDest = 0;
  }
  else if (yR < yDest){
    DIR = FORWARD;
    tDest = PI/2;
  }
  else if (xR < xDest){
    DIR = UP;
    tDest = PI;
  }
  else if (yR > yDest){
    DIR = BACKWARD;
    tDest = -PI/2;
  }
}

void loop() {
  time1 = millis();  
  switch (STATE) {
    case MOVE:
      threshold = 0.01; //1 cm
      
      rho = sqrt((pow((xR - xDest),2))+(pow((yR - yDest),2)));
      if (rho <= threshold) {
        STATE = CLOSE;
        break;
      }

      // 1. Calculate wheelspeeds to reach a certain goal
      // Input: goal and current pose
      // Output: xDot and thetaDot
      
      alpha = atan2((yR-yDest),(xR-xDest)) + PI;
      
      bearing = thetaR - alpha;
      heading = tDest - thetaR;
      
      xDot = 0.5*rho;
      if (xDot > xDOT) xDot = xDOT; //xDOT is maximum velocity 

      if (abs(bearing) >= 2*PI)
      {
        if (bearing > 0) bearing = bearing - 2*PI;
        else bearing = bearing + 2*PI;
      }
      
      if (abs(bearing) > 10*PI/180) thetaDot = (0.3*(bearing) + 0.075*(heading));
      else thetaDot = 0.1*(bearing) + 0.01*(heading);
      
      if (abs(thetaDot) > thetaDOT) //thetaDOT maximum turn speed
      {
        if (thetaDot > 0) thetaDot = thetaDOT; 
        else thetaDot = -thetaDOT;
      }
      
      Serial.print("X: "); Serial.print(xR,4); Serial.print(" Y: "); Serial.print(yR,4); 
      Serial.print(" alpha: "); Serial.println(alpha*180/PI,4);
      Serial.print(" thetaR: "); Serial.print(thetaR*180/PI,4);
      Serial.print(" atan: "); Serial.print(atan2(yR-yDest, xR-xDest)*180/PI);
      Serial.print("   bearing: "); Serial.print(bearing*180/PI,4); Serial.print(" RHO: "); Serial.println(rho);
       
      // 2. Use inverse kinematics to calculate required wheelspeeds
      // Input: xDot and thetaDot
      // Output: phiL and phiR
       
      phiL = ((2*xDot)-(thetaDot*dist))/(2*rad);  
      phiR = ((2*xDot)+(thetaDot*dist))/(2*rad);
      
     if (abs(phiR) > wdot) //wdot max wheel rotational velocity 
     {
       if (phiR < 0) phiR = -wdot; 
       else phiR = wdot;
     } 
     if (abs(phiL) > wdot)
     {
       if (phiL < 0) phiL = -wdot;
       else phiL = wdot;
     }

    // 3. Calculate motor power levels
    // Input: phiL and phiR
    // Output: powerleft and powerright scaled to a 100%
    
    rdir = DIR_CW; //DIR_CW (rotate forward)
    ldir = DIR_CCW; //DIR_CCW (rotate forward)

    rpower=phiR/wdot*100; //scale rad/s to percentage for wheel power 
    lpower=phiL/wdot*100;
    
    if (phiR < 0)
    {
      rdir = DIR_CCW; //rotate backward 
      rpower = abs(rpower);
    }
 
    if (phiL < 0)
    {
      ldir = DIR_CW; //rotate backward 
      lpower = abs(lpower);
    }
   
    sparki.motorRotate(MOTOR_RIGHT, rdir, rpower);  
    sparki.motorRotate(MOTOR_LEFT, ldir, lpower);
    
    // 4. Odometry
    // Input: current positition and xDot and thetaDot
    // Output: new position

    thetaDot = rad*(phiR)/dist - rad*(phiL)/dist; //these recalculations provide more accuracy for odometry
    xDot = rad*(phiL)/2 + rad*(phiR)/2; //equation 3.36  
       
    thetaR = thetaR + thetaDot / 10.0; //change over 100ms 
    
    if (abs(thetaR) >= 2*PI)
    {
      if (thetaR > 0) thetaR = thetaR - 2*PI;
      else thetaR = thetaR + 2*PI;
    }

    xR = xR + cos(thetaR) * xDot / 10.0; //change over 100ms 
    yR = yR + sin(thetaR) * xDot / 10.0; 

    break;
      
    case CLOSE:
      if (currNode == destNode) {
        STATE = STOP;
        break;
      }
      prevNode = currNode;
      currNode = shortestParent[currNode];
      realcoords = indexToxRy(currNode);
      xDest = realcoords.x;
      yDest = realcoords.y;

      prevDIR = DIR;
      if (prevNode - currNode == -4) {
        tDest = PI/2;
        DIR = FORWARD;
      }
      else if (prevNode - currNode == 4) {
        tDest = -PI/2;
        DIR = BACKWARD;
      }
      else if (prevNode - currNode == -1) {
        tDest = 0;
        if (prevDIR == BACKWARD) tDest = -PI;
        DIR = DOWN;
      }
      else if (prevNode - currNode == 1) {
        tDest = 180;
        DIR = UP;
      }
      STATE = MOVE;
      break;
      
    case STOP:
      sparki.RGB(RGB_GREEN);
      sparki.moveStop();
      break;
  }

  sparki.clearLCD();

  sparki.print("X dest: ");
  sparki.println(xDest,3);

  sparki.print("Y dest: ");
  sparki.println(yDest,3);

  sparki.print("Theta dest: ");
  sparki.println(tDest*180/PI);

  sparki.print("Theta real: ");
  sparki.println(thetaR*180/PI);
  
  sparki.print("next node: ");
  sparki.println(currNode);

  sparki.print("X real: ");
  sparki.println(xR,4);

  sparki.print("Y real: ");
  sparki.println(yR,4);
  
  sparki.updateLCD(); // display all of the information written to the screen
    
  time2 = millis(); 
  timer = 100 - (time2 - time1);
  if (timer > 0) delay(timer);
  else if (timer < 0) sparki.RGB(RGB_RED); //loop took over 100ms 
  
}

int xyToIndex(double objx, double objy) {
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

struct RealCoords indexToxRy(int index) {
  struct RealCoords realcoords;

  if (index % 4 == 3) realcoords.x = .525;
  else if (index % 4 == 2) realcoords.x = .375;
  else if (index % 4 == 1) realcoords.x = .225;
  else realcoords.x = .075;

  if (index >= 0 && index < 4) realcoords.y = .05;
  else if (index >= 4 && index < 8) realcoords.y = .15;
  else if (index >= 8 && index < 12) realcoords.y = .25;
  else realcoords.y = .35;
  
  return realcoords;
}

struct Coords indexToxy(int index) {
  struct Coords coords;

  if (index >= 0 && index < 4) coords.y = 0;
  else if (index >= 4 && index < 8) coords.y = 1;
  else if (index >= 8 && index < 12) coords.y = 2;
  else coords.y = 3;

  if (index % 4 == 0) coords.x = 0;
  else if (index % 4 == 1) coords.x = 1;
  else if (index % 4 == 2) coords.x = 2;
  else if (index % 4 == 3) coords.x = 3;
  
  return coords;
}

int cost(int destIndex, int objIndex) {
  struct Coords destCoords = indexToxy(destIndex); 
  struct Coords objCoords = indexToxy(objIndex);

  // if we have detected an object at that index, explode 
  if (nav[objCoords.x][objCoords.y] == 0) return 99;

  // else calc if two nodes are adjacent 
  if (objIndex == destIndex){
    return 0;
  }
  else if (objIndex % 4 == 0) {
    if (objIndex + 1 == destIndex or objIndex + 4 == destIndex or objIndex - 4 == destIndex) {
      return 1;
    }
  }
  else if (objIndex % 4 == 3) { 
    if (objIndex - 1 == destIndex or objIndex + 4 == destIndex or objIndex - 4 == destIndex) {
      return 1;
    }
  } 
  else if (objIndex + 1 == destIndex or objIndex - 1 == destIndex or objIndex + 4 == destIndex or objIndex - 4 == destIndex) {
    return 1;
  }

  return 99;
}

void dij(int nodeCount, int v, int dist[], int shortestParent[]) {
  int i, count, w, flag[nodeCount], min;
  int u = 0;
  
  for(i=0; i<=nodeCount; i++) {
    flag[i] = 0;
    dist[i] = cost(v, i);
  }
  flag[v] = 1;
  count=0;
  while(count <= nodeCount) {
    min = 99;
    u = 0;
    for(w = 0; w <= nodeCount; w++) {
      if(dist[w] < min && !flag[w]) {
        min = dist[w];
        u = w;
      }
    }
    flag[u] = 1;
    int minAdj = 100;
    if (u % 4 != 0 && dist[u - 1] < minAdj) {
      minAdj = dist[u - 1];
      shortestParent[u] = u - 1;
    }
    if (u % 4 != 3 && dist[u + 1] < minAdj) {
      minAdj = dist[u + 1];
      shortestParent[u] = u + 1;
    }
    if (u - 4 >= 0 && dist[u - 4] < minAdj) {
      minAdj = dist[u - 4];
      shortestParent[u] = u - 4;
    }
    if (u + 4 <= 15 && dist[u + 4] < minAdj) {
      minAdj = dist[u + 4];
      shortestParent[u] = u + 4;
    }
    
    count++;
    for(w = 0; w <= nodeCount; w++) {
      if((dist[u] + cost(u, w) < dist[w]) && !flag[w]) {
        dist[w] = dist[u] + cost(u, w);
      }
    }
  }
}


