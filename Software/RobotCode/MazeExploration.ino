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

// Speeds for motors
int Lforward = 94; // was 100
int Lbackward = 86; // was 80
int Stop = 90;
int Rforward = 86; // was 80
int Rbackward = 94; // was 100

#include "StackArray.h"
#include <stdio.h>
#define height 4
#define width 5
struct Node {
  char visitedVar = 0; 
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

int sensorValue;
float volts;
float distance;
float test_dist; // A tester variable which will come in handy soon, trust me.

int leftWallSensor = 0;
int rightWallSensor = 0;
int frontWallSensor = 0;
//Thus ends IR stuff.

//create stack
  StackArray <Node> visited; 
  int step = 0; //keep track of total # of moves
  char go_north;
  char go_south;
  char go_east;
  char go_west;
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
  

}

void loop() {

LineFollowing();
  
  if (turnRight) {
        left.write(Lforward);
        right.write(Rbackward); // verify this works.. want right wheel to go backwards
        Serial.println("Turn 90 Degrees Right");
        delay(500);
        while(analogRead(RightRear) < 850); // Wait here until left rear sensor senses line
        Serial.println("Left Rear Detected");
        turnRight = 0;
  } 
  if (turnLeft) {
        left.write(Lbackward);  // verify this works.. want left wheel to go backwards
        right.write(Rforward);
        Serial.println("Turn 90 Degrees Left");
        delay(500);
        while(analogRead(RightRear) < 850); // Wait here until left rear sensor senses line
        Serial.println("Right Rear Detected");
        turnLeft = 0;
  }
  if (turn180) {
        left.write(Lforward);
        right.write(Rbackward); // verify this works.. want right wheel to go backwards
        Serial.println("Turn 180 Degrees");
        delay(500);
        while(analogRead(RightRear) < 850); // Wait here until left rear sensor senses line
        Serial.println("Left Rear Detected");
        delay(500);
        while(analogRead(RightRear) < 850); // Wait here until left rear sensor senses line
        Serial.println("Left Rear Detected");
        turn180 = 0;
  }

  if (goStraight) {
        left.write(Lforward);
        right.write(Rforward); // verify this works.. want right wheel to go backwards
        Serial.println("Go Straight");
        goStraight = 0;
  }
  
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
    
  sensorValue = analogRead(AX);
  
 
  volts = sensorValue*(5.0/1024.0);
  distance = 11*(1/volts)-.42; //inverse number of distance plotted in datasheet- calculated slope of 11 from best fit line using 2 points

  if (i==0 && distance >20) {
    leftWallSensor = 0;
    Serial.print("left wall not detected. Distance: ");
    Serial.println(distance);
  }
  else if (i==0) {
    leftWallSensor = 1;
    Serial.print("left wall detected. Distance: ");
    Serial.println(distance);
  }

  if (i==1 && distance >20) {
    rightWallSensor = 0;
    Serial.print("right wall not detected. Distance: ");
    Serial.println(distance);
  }
  else if (i==1) {
    rightWallSensor = 1;
    Serial.print("right wall detected. Distance: ");
    Serial.println(distance);
  }

  if (i==2 && distance >20) {
    frontWallSensor = 0;
    Serial.print("front wall not detected. Distance: ");
    Serial.println(distance);
  }
  else if (i==2) {
    frontWallSensor = 1;
    Serial.print("front wall detected. Distance: ");
    Serial.println(distance);
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
  
    if(analogRead(RightRear) > 850 && analogRead(LeftRear) > 850 && CheckAgain) { // Make a decision on whether to go left, right, or straight.
      left.write(Stop);
      right.write(Stop);
      Serial.println("Algorithm");
      CheckAgain = 0;
      
      traverse(); // this will assign variable "turnRight", "turnLeft", or "turn180" to 1
      
  }
   
}

//////////////////////////////////////////////////////////////////////////////////


int curr_x = 0; 
  int curr_y = 0; 

void traverse() {

  IR(); // check walls

  Serial.println("Traverse"); 
  visited.push(maze[width][height]); 
  
        Node curr_pos; 
        Node next_pos;
  
  if (!visited.isEmpty()) {
    
    curr_pos = visited.peek(); //current position is peek of stack
    visited.pop(); 
    curr_x = curr_pos.x; 
    curr_y = curr_pos.y;
    curr_pos.visitedVar = 1; //mark as visited
                //Serial.println("Just visited: %d, %d", curr_x, curr_y); 
                 Serial.print(curr_x); Serial.println(curr_y);
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

  Serial.print(Direction);
  Serial.print("North: ");Serial.println(go_north);
  Serial.print("East: ");Serial.println(go_east);
  Serial.print("West: ");Serial.println(go_west);
  Serial.print("South: ");Serial.println(go_south);

    if (go_north) {
      next_pos = maze[curr_x][curr_y + 1];
      visited.push(next_pos);
      if (Direction == 0) goStraight = 1;
      if (Direction == 1) turnLeft = 1;
      if (Direction == 2) turnRight = 1;
      if (Direction == 3) turn180 = 1;
      
      Direction = 0;
      Serial.println(". Now go North");
      
    }
    else if (go_east) {
      next_pos = maze[curr_x - 1][curr_y];
      visited.push(next_pos);
      if (Direction == 0) turnRight = 1;
      if (Direction == 1) goStraight = 1;
      if (Direction == 2) turn180 = 1;
      if (Direction == 3) turnLeft = 1;
      Direction = 1;
      Serial.println("East");
    }
    else if (go_west) {
      next_pos = maze[curr_x - 1][curr_y];
      visited.push(next_pos);
      if (Direction == 0) turnLeft = 1;
      if (Direction == 1) turn180 = 1;
      if (Direction == 2) goStraight = 1;
      if (Direction == 3) turnRight = 1;
      Direction = 2;
      Serial.println("West");
    }
    else if (go_south) {
      next_pos = maze[curr_x][curr_y - 1];
      visited.push(next_pos);
      if (Direction == 0) turn180 = 1;
      if (Direction == 1) turnRight = 1;
      if (Direction == 2) turnLeft = 1;
      if (Direction == 3) goStraight = 1;
      Direction = 3;
      Serial.println("South");
    }
    else {
      next_pos = visited.peek(); 
      visited.pop(); 
    }

  }
  else {
    Serial.print("Maze is complete");
    left.write(100);
    right.write(100);
    while(1);
  }
  
}


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
