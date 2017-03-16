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

struct Coords {
  int x;
  int y;
};

void setup() {
  // put your setup code here, to run once:
  sparki.clearLCD();
  sparki.servo(-90);

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
}

void loop() { 
  sparki.clearLCD();
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

  int testDist[15];
  int shortestParent[15];
  int currNode = 0; // starting point on map
  int destNode = 11; // goal on map
  dij(15, destNode, testDist, shortestParent);

// print out path from start (0) to finish (11) 
//  while (currNode != destNode) {
//    sparki.print(currNode);
//    sparki.print(" ");
//    currNode = shortestParent[currNode];
//    sparki.updateLCD();
//    delay(500);
//  }
//  sparki.print(destNode);
//  sparki.updateLCD();

//  for (int i=0; i < 15; i++){
//    if (i % 4 ==0){
//      sparki.println(testDist[i]);
//    }
//    else sparki.print(testDist[i]);
//  }
  sparki.updateLCD(); // display all of the information written to the screen
  delay(1000);
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
  int i, u, count, w, flag[nodeCount], min;
  
  for(i=0; i<=nodeCount; i++) {
    flag[i] = 0;
    dist[i] = cost(v, i);
  }
  flag[v] = 1;
  count=0;
  while(count <= nodeCount) {
    min = 99;
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
//  for (int i = 0; i <= nodeCount; i++) {
//    sparki.print(i);
//    sparki.print("   ");
//    sparki.println(shortestParent[i]);
//    sparki.updateLCD();
//    delay(500);
//  }
  
// beginning of fancy map
//    for (int i=0; i < 15; i++){
//      if (i % 4 == 0){
//        sparki.println(dist[i]);
//      }
//      else {
//        sparki.print(dist[i]);
//        sparki.print("   ");
//      }
//      sparki.updateLCD();
//    }
    delay(500);

}


