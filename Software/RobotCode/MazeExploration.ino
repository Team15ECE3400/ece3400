#include <Servo.h>
int QRE1113_PinL = 0; // connect left light sensor to analog pin 0
int QRE1113_PinR = 1; // connect left light sensor to analog pin 0
int onLine = 850;     // threshold value for sensor on line
int LFCrossed;          // returns boolean for statment "left sensor is on the line"
int RFCrossed;          // returns boolean for statment "right sensor is on the line"

int val = 100;        //normal motor speed
int offVal = 140;     //motor speed when sensor is off line, faster to correct
Servo left;           //set up left and right motor
Servo right;
int x=0;
int y=1;

const int Left = 3;
const int Right = 2;
int turnRight = 0;
int turnLeft = 0;
int turn180 = 0;
int goStraight = 0;

int LeftRear = A2;
int RightRear = A1;

int CheckAgain = 0;
int Traversing = 1;

// Speeds for motors
int Lforward = 95; // was 100
int Lbackward = 85; // was 80
int Stop = 90;
int Rforward = 86; // was 80
int Rbackward = 94; // was 100

#include "StackArray.h"
#include <stdio.h>
#define height 5 // Y-axis
#define width 4  // X-axis
struct Node {
  int visitedVar = 0; 
  int x;
  int y;
};

// IR Code
int AX;
int i = 0;

int IR_LEFT = A3;
int IR_RIGHT = A4;
int IR_FOR = A5;

float s1_vals[3] = {0,0,0};
float s2_vals[3] = {0,0,0};
float s3_vals[3] = {0,0,0};

int s1_index = 0;
int s2_index = 0;
int s3_index = 0;

int s1_outliers = 0;
int s2_outliers = 0;
int s3_outliers = 0;

float s1_dist_avg;
float s2_dist_avg;
float s3_dist_avg;

int sensorValue1;
int sensorValue2;
int sensorValue3;
float volts;
float distance;
float test_dist; // A tester variable which will come in handy soon, trust me.

int leftWallSensor = 0;
int rightWallSensor = 0;
int frontWallSensor = 0;
//Thus ends IR stuff.

//create stack
  StackArray <Node> visited; 
  int initNode =0; //push the first node to visited stack
  int step = 0; //keep track of total # of moves
  int go_north;
  int go_south;
  int go_east;
  int go_west;
  // 0 is North
  // 1 is East
  // 2 is West
  // 3 is South
  int Direction = 0;
  Node maze[width][height]; //stores maze 
  int wall_loc[width][height];

void setup() {
  Serial.begin(9600);
  left.attach(6);    //connect left servo to pin 11 and right servo to pin 10
  right.attach(5);
  left.write(90);            
  right.write(90);
  
  pinMode(Left, INPUT);
  pinMode(Right, INPUT);
  
  pinMode(LeftRear, INPUT);
  pinMode(RightRear, INPUT);
  
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
  pinMode(IR_FOR, INPUT);

  // Initialize Maze
  for (int i = 0; i < width; i++) {
    for (int j = 0; j < height; j++) {
      maze[i][j].visitedVar = 0;
      maze[i][j].x = i;
      maze[i][j].y = j;
    }
  }

  traverse(); // Start maze exploration algorithm

}

void loop() {
  
  if (turnRight) {
        left.write(Lforward);
        right.write(Rbackward); // verify this works.. want right wheel to go backwards
        //Serial.println("Turn 90 Degrees Right");
        delay(500);
        while(digitalRead(Right) ==0 ); // Wait here until right front sensor senses line
        //Serial.println("frontright Detected");
        turnRight = 0;
  } 
  if (turnLeft) {
        left.write(Lbackward);  // verify this works.. want left wheel to go backwards
        right.write(Rforward);
        //Serial.println("Turn 90 Degrees Left");
        delay(500);
        while(digitalRead(Left) == 0); // Wait here until left front sensor senses line
        //Serial.println("frontleft Detected");
        turnLeft = 0;
  }
  if (turn180) {
        left.write(Lforward);
        right.write(Rbackward); // verify this works.. want right wheel to go backwards
        //Serial.println("Turn 180 Degrees");
        delay(500);
        while(digitalRead(Right) ==0); // Wait here until right front sensor senses line
        //Serial.println("frontright Detected part 1");
        delay(500);
        while(digitalRead(Right) ==0); // Wait here until right front sensor senses line
        //Serial.println("frontright Detected part 2");
        turn180 = 0;
  }

  if (goStraight) {
        left.write(Lforward);
        right.write(Rforward); // verify this works.. want right wheel to go backwards
        //Serial.println("Go Straight");
        goStraight = 0;
  }

  LineFollowing();
  
}
///////////////////////////////////////////////////////
void IR() {

  for (int k = 0; k<3; k++) {
  
  if(i == 0)
    AX = IR_LEFT;
  else if (i == 1)
    AX = IR_RIGHT;
  else
    AX = IR_FOR;
    
  sensorValue1 = analogRead(AX);
  delay(33);
  sensorValue2 = analogRead(AX);
  delay(33);
  sensorValue3 = analogRead(AX);
  delay(33);
  
 
  volts = (sensorValue1 + sensorValue2 + sensorValue3)/3.0*(5.0/1024.0);
  distance = 11*(1/volts)-.42; //inverse number of distance plotted in datasheet- calculated slope of 11 from best fit line using 2 points

  if (i==0 && distance >20) {
    leftWallSensor = 0;
    //Serial.print("left wall not detected. Distance: ");
    //Serial.println(distance);
  }
  else if (i==0) {
    leftWallSensor = 1;
    //Serial.print("left wall detected. Distance: ");
    //Serial.println(distance);
  }

  if (i==1 && distance >20) {
    rightWallSensor = 0;
    //Serial.print("right wall not detected. Distance: ");
    //Serial.println(distance);
  }
  else if (i==1) {
    rightWallSensor = 1;
    //Serial.print("right wall detected. Distance: ");
    //Serial.println(distance);
  }

  if (i==2 && distance >20) {
    frontWallSensor = 0;
    //Serial.print("front wall not detected. Distance: ");
    //Serial.println(distance);
  }
  else if (i==2) {
    frontWallSensor = 1;
    //Serial.print("front wall detected. Distance: ");
    //Serial.println(distance);
  }
  
  i++;
  
  }
  i = 0;

}
///////////////////////////////////////////////////////
void LineFollowing() {

    if (digitalRead(Left)) LFCrossed = 1;
   else LFCrossed = 0;

   //determine line status for right sensor
   if (digitalRead(Right)) RFCrossed = 1;
   else RFCrossed = 0;
   
  //drive servos
   // if both sensors on either side of line
   if(!LFCrossed && !RFCrossed){
    left.write(Lforward);            
    right.write(Rforward);
   }

   else if(LFCrossed && !RFCrossed){
    left.write(Lforward);          
    right.write(Rforward + 2); // slow down right wheel slightly
   }

   else if(!LFCrossed && RFCrossed){
    left.write(Lforward - 2); // slow down left wheel slightly            
    right.write(Rforward);
   }

   else if(LFCrossed && RFCrossed) {
      CheckAgain = 1;
    // do nothing
  }
    //if either light sensor is off the line (has onStat_ == 0)  
  else{
    // Do nothing
  }
  
   if(analogRead(RightRear) > 850 && analogRead(LeftRear) > 850  && CheckAgain ) { // Make a decision on whether to go left, right, or straight.
      
      left.write(Stop);
      right.write(Stop);
      //Serial.println("Algorithm");
      CheckAgain = 0;
      
      traverse(); // this will assign variable "turnRight", "turnLeft", or "turn180" to 1
      
  }
   
}

//////////////////////////////////////////////////////////////////////////////////


int curr_x = 0; 
int curr_y = 0;        
Node curr_pos; 
Node next_pos;

void checkWalls(){
  if(Direction ==0){
    bitWrite(wall_loc[curr_x][curr_y], 0, frontWallSensor);
    bitWrite(wall_loc[curr_x][curr_y], 1, rightWallSensor);
    bitWrite(wall_loc[curr_x][curr_y], 2, leftWallSensor);
    bitWrite(wall_loc[curr_x][curr_y], 3, 0);
  }
  if(Direction ==1){
    bitWrite(wall_loc[curr_x][curr_y], 0, leftWallSensor);
    bitWrite(wall_loc[curr_x][curr_y], 1, frontWallSensor);
    bitWrite(wall_loc[curr_x][curr_y], 2, 0);
    bitWrite(wall_loc[curr_x][curr_y], 3, rightWallSensor);
  }
  if(Direction ==2){
    bitWrite(wall_loc[curr_x][curr_y], 2, frontWallSensor);
    bitWrite(wall_loc[curr_x][curr_y], 1, 0);
    bitWrite(wall_loc[curr_x][curr_y], 0, rightWallSensor);
    bitWrite(wall_loc[curr_x][curr_y], 3, leftWallSensor);
  }
  if(Direction == 3){
    bitWrite(wall_loc[curr_x][curr_y], 2, rightWallSensor);
    bitWrite(wall_loc[curr_x][curr_y], 3, frontWallSensor);
    bitWrite(wall_loc[curr_x][curr_y], 1, leftWallSensor);
    bitWrite(wall_loc[curr_x][curr_y], 0, 0);
  }
}


void traverse() {

  IR(); // check walls

  //Serial.println("Traverse"); 
  if(initNode == 0){
    visited.push(maze[width-1][0]); // Start at (4,0)
    initNode=1; //visited adds the corner node as the first element
  }




// Use this to check if maze traversing is complete
Traversing = 0;
  for (int i = 0; i < width; i++) {
    for (int j = 0; j < height; j++) {
      if (!maze[i][j].visitedVar) Traversing = 1;
    }
  }


  if (Traversing) {
  
  // if (!visited.isEmpty()) { // this doesn't work because we always push one node to the stack
    
    curr_pos = visited.peek(); //current position is peek of stack
    visited.pop(); 
    curr_x = curr_pos.x; 
    curr_y = curr_pos.y;
    curr_pos.visitedVar = maze[curr_x][curr_y].visitedVar = 1; //mark as visited
    Serial.print("Current Position Visited: ");Serial.println(curr_pos.visitedVar);
                //Serial.println("Just visited: %d, %d", curr_x, curr_y); 
                 Serial.print("X = ");Serial.print(curr_x); Serial.print(", Y = ");Serial.println(curr_y);
    //Look for next wall to visit
    checkWalls();
    //string wall_bin = bitset<4>(wall_loc[curr_x][curr_y]).to_string(); //to binary
    int wall_bin = wall_loc[curr_x][curr_y]; 
    //check if wall at north
    //if (bitRead(wall_bin,0) == 0) go_north = !(maze[curr_x][curr_y+1].visitedVar );
        if (bitRead(wall_bin,0) == 0) go_north = 1;

    else go_north = 0; 

    //check if wall at east
    //if (bitRead(wall_bin, 1) == 0) go_east = (!(maze[curr_x-1][curr_y].visitedVar & !go_north) );
            if (bitRead(wall_bin,1) == 0) go_east = 1;

    else go_east = 0; 

    //check if wall at west
    //if (bitRead(wall_bin, 2) == 0) go_west = (!maze[curr_x+1][curr_y].visitedVar & !go_east);
            if (bitRead(wall_bin,2) == 0) go_west = 1;

    else go_west = 0; 

    //check if wall at south
   // if (bitRead(wall_bin, 3) == 0) go_south = ((!maze[curr_x][curr_y - 1].visitedVar) & !go_west);
            if (bitRead(wall_bin,3) == 0) go_south = 1;
 
    else go_south = 0;

  Serial.print("Direction = ");Serial.println(Direction);
  Serial.print("North: ");Serial.println(go_north);
  Serial.print("East: ");Serial.println(go_east);
  Serial.print("West: ");Serial.println(go_west);
  Serial.print("South: ");Serial.println(go_south);

   if(Direction ==0){
    if (go_north) goStraight = 1;    
    else if (go_east) {
      turnRight = 1; 
      go_north =0;
    }
    else if (go_west){
      turnLeft = 1;
      go_north =0;
      go_east = 0;
    }
    else if (go_south){
      turn180 = 1;
      go_north = 0;
      go_east = 0;
      go_west = 0;
    }
  }
   if(Direction ==1){
    if (go_east){
      goStraight = 1;  
      go_north = 0;}  
    else if (go_south) {
      turnRight = 1;
      go_north=0;
      go_east =0;
      go_west = 0;
    }
    else if (go_north) turnLeft = 1;
    else if (go_west){
      turn180 = 1;
      go_east = 0;
      go_north = 0; }
  }
   if(Direction ==2){
    if (go_west) {
      goStraight = 1;   
      go_north =0;
      go_east = 0;
    } 
    else if (go_north) turnRight = 1;
    else if (go_south) {
      turnLeft = 1;
      go_north=0;
      go_east =0;
      go_west = 0;}
    else if (go_east) {
     turn180 = 1;
     go_north =0;
     }
  }
   if(Direction ==3){
    if (go_south) {
      goStraight = 1;  
      go_north=0;
      go_east =0;
      go_west = 0;}  
    else if (go_west) {
      turnRight = 1;
      go_north=0;
      go_east =0;
    }
    else if (go_east) {
      turnLeft = 1;
      go_north =0;}
    else if (go_north) turn180 = 1;
  }

  if (go_north) {
      next_pos = maze[curr_x][curr_y + 1];
      visited.push(next_pos);
      Direction = 0;
      Serial.println("Go North");
    }
    else if (go_east) {
      next_pos = maze[curr_x + 1][curr_y];
      visited.push(next_pos);
      Direction = 1;
      Serial.println("Go East");
    }
    else if (go_west) {
      next_pos = maze[curr_x - 1][curr_y];
      visited.push(next_pos);
      Direction = 2;
      Serial.println("Go West");
    }
    else if (go_south) {
      next_pos = maze[curr_x][curr_y - 1];
      visited.push(next_pos);
      Direction = 3;
      Serial.println("Go South");
    }

    else {
      next_pos = visited.peek(); 
      visited.pop(); 
    }

    for (int j = height - 1; j >=0; j--) {
      for (int i = 0; i < width; i++) {
      Serial.print(maze[i][j].visitedVar);Serial.print(", ");
      }
      Serial.println(" ");
    }
    Serial.println(" ");
  }

  else {
    Serial.print("Maze is complete");
    left.write(92);
    right.write(88);
    while(1);
  }
}
