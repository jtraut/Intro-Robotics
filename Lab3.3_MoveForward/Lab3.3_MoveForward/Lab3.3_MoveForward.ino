#include <Sparki.h> // include the sparki library
#include <math.h>

#define MOVE 0
#define CLOSE 1
#define STOP 2

#define FORWARD 1 //positive y
#define BACKWARD 2 //negative y
#define UP 3 //positive x
#define DOWN 4 //negative x 

int STATE = 0, DIR = 1;
int prevDIR = 1;

float dist = 0.0841; //distance between the wheels
float rad = 0.0258; //radius of the wheels
float xSpeed, thetaSpeed, phiR, phiL;
float xR = .525; // hardcoded 
float yR = .05; // hardcoded
float thetaR = -PI/2;

float xChange = 15; //in cm, from center of node to next 
float yChange = 10;

float xDest, yDest, tDest;

float rho;
float alpha;
float threshold;
float multiplier;
unsigned long start;

int tempParent;
int prevNode;

float phidot = 36.1;//37.414258989815;   //degrees / sec
float xdot = 0.0278241513638; // meters / sec
float ydot = 0;  // meters / sec

float xold = 0;
float xnew = 0;
float y = 0;
float phi = 0;
float realx = 0;
float realy = 0;
float xGoal, yGoal;

float objx = 0;
float objy = 0;

int PIng = -1;

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
  float x; 
  float y;
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

  destcoords = indexToRealxy(destNode); //store the x,y goal 
  xGoal = destcoords.x;
  yGoal = destcoords.y;
  
  dij(15, destNode, testDist, shortestParent);
  if (shortestParent[0] != -1) {
    currNode = shortestParent[0];
    destcoords = indexToRealxy(currNode);
    xDest = destcoords.x;
    yDest = destcoords.y;
  }
  
  if (xR > xDest){
    DIR = DOWN; 
    tDest = PI;
  }
  else if (yR < yDest){
    DIR = FORWARD;
    tDest = -PI/2;
  }
  else if (xR < xDest){
    DIR = UP;
    tDest = 0;
  }
  else if (yR > yDest){
    DIR = BACKWARD;
    tDest = PI/2;
  }
}
// TODO: set theta for going to different nodes -> change direction = change theta
// TODO: check if shortestParent = -1 
void loop() {
  switch (STATE) {
    case MOVE:
      threshold = 0.01;
 /*
      alpha = atan2((yR-yDest),(xR-xDest));
      rho = sqrt((pow((xDest- xR),2))+(pow((yDest-yR),2)));

      xSpeed = multiplier*rho;
      thetaSpeed = (1*(alpha-thetaR))+(0.01*(tDest - thetaR));
    
      phiL = ((2*xSpeed/rad)-(thetaSpeed*dist))/2;  
      phiR = ((2*xSpeed/rad)+(thetaSpeed*dist))/2; 
    
      xR = xR + (phiL*rad/2)+(phiR*rad/2);
      thetaR = (phiR*rad/dist) - (phiL*rad/dist);

      if (rho < threshold) {
        STATE = CLOSE;
      }
      
      if (phiR <= 0) {
        sparki.motorRotate(MOTOR_RIGHT, DIR_CCW, abs(phiL*100));
      }
      else {
        sparki.motorRotate(MOTOR_RIGHT, DIR_CW, (phiR*100));    
      }
      if (phiL <= 0) {
        sparki.motorRotate(MOTOR_LEFT, DIR_CW, abs(phiL*100));    
      }
      else {
        sparki.motorRotate(MOTOR_LEFT, DIR_CCW, (phiL*100));    
      }
   */

      //FORWARD KINEMATICS 
      if (DIR == FORWARD){
        if (prevDIR == FORWARD){
          sparki.moveForward(yChange); //10 cm in y direction till next block center 
        }
        else if (prevDIR == UP){
          sparki.moveRight(90);
          sparki.moveForward(yChange);
        }
        else if (prevDIR == DOWN){
          sparki.moveLeft(90);
          sparki.moveForward(yChange);
        }
        yR = yR + yChange/100;
        thetaR = -PI/2;    
      }
      else if (DIR == BACKWARD){
        if (prevDIR == BACKWARD){
          sparki.moveForward(yChange);
        }
        else if (prevDIR == UP){
          sparki.moveLeft(90);
          sparki.moveForward(yChange);
        }
        else if (prevDIR == DOWN){
          sparki.moveRight(90);
          sparki.moveForward(yChange);
        }
        yR = yR - yChange/100;
        thetaR = PI/2;
      }
      else if (DIR == UP){
        if (prevDIR == UP){
          sparki.moveForward(xChange);
        }
        else if (prevDIR == FORWARD){
          sparki.moveLeft(90);
          sparki.moveForward(xChange);
        }
        else if (prevDIR == BACKWARD){
          sparki.moveRight(90);
          sparki.moveForward(xChange);
        }
        thetaR = 0;
        xR = xR + xChange/100;
      }
      else if (DIR == DOWN){
        thetaR = PI;
        xR = xR - xChange/100;
        if (prevDIR == DOWN){
          sparki.moveForward(xChange);
        }
        else if (prevDIR == FORWARD){
          sparki.moveRight(90);
          sparki.moveForward(xChange);
          thetaR = -PI;
        }
        else if (prevDIR == BACKWARD){
          sparki.moveLeft(90);
          sparki.moveForward(xChange);
        }
      }

      rho = sqrt((pow((xDest- xR),2))+(pow((yDest-yR),2)));
      if (rho < threshold) {
        STATE = CLOSE;
      }
      
      break;
    case CLOSE:
      if (currNode == destNode) {
        STATE = STOP;
        break;
      }
      prevNode = currNode;
      currNode = shortestParent[currNode];
      realcoords = indexToRealxy(currNode);
      xDest = realcoords.x;
      yDest = realcoords.y;

      prevDIR = DIR;
      if (prevNode - currNode == -4) {
        tDest = -PI/2;
        DIR = FORWARD;
      }
      else if (prevNode - currNode == 4) {
        tDest = PI/2;
        DIR = BACKWARD;
      }
      else if (prevNode - currNode == -1) {
        tDest = PI;
        if (prevDIR == FORWARD) tDest = -PI;
        DIR = DOWN;
      }
      else if (prevNode - currNode == 1) {
        tDest = 0;
        DIR = UP;
      }
      STATE = MOVE;
      break;
    case STOP:
      sparki.RGB(RGB_GREEN);
      break;
  }

  sparki.clearLCD();
  
  sparki.print("current direction: ");
  sparki.println(DIR);

  sparki.print("X dest: ");
  sparki.println(xDest);

  sparki.print("Y dest: ");
  sparki.println(yDest);

  sparki.print("Theta dest: ");
  sparki.println(tDest);

  sparki.print("next node: ");
  sparki.println(currNode);

  sparki.print("X goal: ");
  sparki.println(xGoal);

  sparki.print("Y goal: ");
  sparki.println(yGoal);
  
  sparki.updateLCD(); // display all of the information written to the screen
  delay(100);
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

struct RealCoords indexToRealxy(int index) {
  struct RealCoords realcoords;

  if (index % 4 == 3) realcoords.x = .075;
  else if (index % 4 == 2) realcoords.x = .225;
  else if (index % 4 == 1) realcoords.x = .375;
  else realcoords.x = .525;

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


