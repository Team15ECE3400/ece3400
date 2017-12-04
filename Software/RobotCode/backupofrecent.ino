#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
// Set up nRF24L01 radio on SPI bus plus pins 9 & 10
RF24 radio(9,10);
// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0x0000000036LL, 0x0000000037LL };
// The various roles supported by this sketch
typedef enum { role_ping_out = 1, role_pong_back } role_e;
// The debug-friendly names of those roles
const char* role_friendly_name[] = { "invalid", "Ping out", "Pong back"};
// The role of the current running sketch
role_e role = role_pong_back;
bool ok;
//int Direction; ////////////
int PlusMinus;
int xory;
int treasure = B10;
int wall;
int moved;                                  
int data_transmit;
int sent = 1;
bool done = 0;
//int wL, wR, wF;
//int m, n;
//int x;////////////
//int y;////////////


////////////END RADIO INITILIZATION////////////


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

int turnRight = 0;
int turnLeft = 0;
int turn180 = 0;
int goStraight = 0;

int LeftRear = A0;
int LeftFront = A1;
int RightFront = A2;

int CheckAgain = 1; // Make it 1 for the first check
bool doneMaze=0;

// Speeds for motors
//int Lforward = 95; // was 100
//int Lbackward = 85; // was 80
//int Stop = 90;
//int Rforward = 86; // was 80
//int Rbackward = 94; // was 100
int Lforward = 104; // was 100
int Lbackward = 75; // was 80
int Stop = 90;
int Rforward = 79; // was 80
int Rbackward = 105; // was 100

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
int start = 0;

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

int _7kHz = 8;
int _12kHz = 3;
int _17kHz = 4;
int _660Hz = 7;

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
  int explored[width][height];

int BackTrack[width][height]; //stores backtrack maze 

void setup() {

//delay(10000);
  
  Serial.begin(9600);
  left.attach(6);    //connect left servo to pin 11 and right servo to pin 10
  right.attach(5);
  left.write(90);            
  right.write(90);
  
  pinMode(LeftFront, INPUT);
  pinMode(RightFront, INPUT);
  
  pinMode(LeftRear, INPUT);
  
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
  pinMode(IR_FOR, INPUT);

  pinMode(2,INPUT);

  pinMode(_7kHz, INPUT);
  pinMode(_12kHz, INPUT);
  pinMode(_17kHz, INPUT);
  pinMode(_660Hz, INPUT);

  // Initialize Maze
  for (int i = 0; i < width; i++) {
    for (int j = 0; j < height; j++) {
      maze[i][j].visitedVar = 0;
      maze[i][j].x = i;
      maze[i][j].y = j;
      explored[i][j] =4;
     if(i==3&&j==0)
        explored[i][j]=3;
    }
    
  }


// Initialize Maze
  for (int i = 0; i < width; i++) {
    for (int j = 0; j < height; j++) {
      BackTrack[i][j] = -1;
    }
  }
  

//////////RADIO CODE////////////
  printf_begin();
  printf("\n\rRF24/examples/GettingStarted/\n\r");
  printf("ROLE: %s\n\r",role_friendly_name[role]);
  printf("*** PRESS 'T' to begin transmitting to the other node\n\r");

  radio.begin();

  // optionally, increase the delay between retries & # of retries
  radio.setRetries(15,15);
  radio.setAutoAck(true);
  // set the channel
  radio.setChannel(0x50);
  // set the power
  // RF24_PA_MIN=-18dBm, RF24_PA_LOW=-12dBm, RF24_PA_MED=-6dBM, and RF24_PA_HIGH=0dBm.
  radio.setPALevel(RF24_PA_MIN);
  //RF24_250KBPS for 250kbs, RF24_1MBPS for 1Mbps, or RF24_2MBPS for 2Mbps
  radio.setDataRate(RF24_250KBPS);

  if ( role == role_ping_out )
  {
    radio.openWritingPipe(pipes[0]);
    radio.openReadingPipe(1,pipes[1]);
  }
  else
  {
    radio.openWritingPipe(pipes[1]);
    radio.openReadingPipe(1,pipes[0]);
  }

  radio.startListening();

  radio.printDetails();

    x=0;///////
    y=0;///////

RADIO();

traverse(); // Start maze exploration algorithm



xory = 1;
PlusMinus = 1;
wall = 3;
moved = 1;

Serial.println("Waiting for 660Hz");
while(!digitalRead(_660Hz) && !digitalRead(2)); // Wait for push button to be pressed
Serial.println("660Hz Detected!");


}





void loop() {

//  while (1) {
//  if (digitalRead(_7kHz)) {
//    Serial.println("7 kHz detected");
//    treasure = 1; // B001
//  }
//  if (digitalRead(_12kHz)) {
//    Serial.println("12 kHz detected");
//    treasure = 2; // B010
//  }
//  if (digitalRead(_17kHz)) {
//    Serial.println("17 kHz detected");
//    treasure = 4; // B100
//  }
//
//  
//}

  
  if (turnRight) {
        left.write(Lforward - 5);
        right.write(Rbackward + 10); // verify this works.. want right wheel to go backwards
        delay(300);
        while(analogRead(RightFront) < 700);
       // while(analogRead(LeftFront) < 700); // Wait here until right front sensor senses line
        //Serial.println("frontright Detected");
        turnRight = 0;
  } 
  if (turnLeft) {
        left.write(Lbackward + 5);  // verify this works.. want left wheel to go backwards
        right.write(Rforward - 10);
        delay(300);
        while(analogRead(LeftFront) < 700);
      //  while(analogRead(RightFront) < 700); // Wait here until left front sensor senses line
        //Serial.println("frontleft Detected");
        turnLeft = 0;
  }
  if (turn180) {
        left.write(Lforward);
        right.write(Rbackward); // verify this works.. want right wheel to go backwards
        delay(100);
        while(analogRead(RightFront) < 700); // Wait here until right front sensor senses line
        while(analogRead(LeftFront) < 700);
        //Serial.println("frontright Detected part 1");
        delay(100);
        while(analogRead(RightFront) < 700); // Wait here until right front sensor senses line
        while(analogRead(LeftFront) < 700);
        //Serial.println("frontright Detected part 2");
        turn180 = 0;
  }

  if (goStraight) {
        left.write(Lforward);
        right.write(Rforward); // verify this works.. want right wheel to go backwards
        goStraight = 0;
  }

  LineFollowing();
  
}
///////////////////////////////////////////////////////
void IR() {

if (digitalRead(_7kHz)) {
  Serial.println("7 kHz detected");
  treasure = 1; // B001
}
  if (digitalRead(_12kHz)) {
    Serial.println("12 kHz detected");
    treasure = 2; // B010
  }
  if (digitalRead(_17kHz)) {
    Serial.println("17 kHz detected");
    treasure = 4; // B100
  }

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

    if (analogRead(LeftFront) > 700) LFCrossed = 1;
   else LFCrossed = 0;

   //determine line status for right sensor
   if (analogRead(RightFront) > 700) RFCrossed = 1;
   else RFCrossed = 0;
   
  //drive servos
   // if both sensors on either side of line
   if(!LFCrossed && !RFCrossed){
    left.write(Lforward);            
    right.write(Rforward);
   }

   else if(LFCrossed && !RFCrossed){
    left.write(90);          
    right.write(Rforward); // slow down right wheel slightly
   }

   else if(!LFCrossed && RFCrossed){
    left.write(Lforward); // slow down left wheel slightly            
    right.write(90);
   }

   else if(LFCrossed && RFCrossed) {
      CheckAgain = 1;
    // do nothing
  }
    //if either light sensor is off the line (has onStat_ == 0)  
  else{
    // Do nothing
  }
  
   if(analogRead(LeftRear) > 850  && CheckAgain ) { // Make a decision on whether to go left, right, or straight.

      start = 1;
      //Serial.println("about to traverse");
      left.write(Stop);
      right.write(Stop);

IR(); // check walls

RADIO();

//delay(3000);
      
      //Serial.println("Algorithm");
      CheckAgain = 0;
      moved = 1;
      traverse(); // this will assign variable "turnRight", "turnLeft", or "turn180" to 1

      
      
  }
   
}

//////////////////////////////////////////////////////////////////////////////////


int curr_x = 0; 
int curr_y = 0;        
Node curr_pos; 
Node prev_pos;
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

  //IR(); // check walls

  //Serial.println("Traverse"); 
  if(initNode == 0 && start == 1){
    visited.push(maze[width-1][0]); // Start at (4,0)
    initNode=1; //visited adds the corner node as the first element
    prev_pos = visited.peek();
  }

  if (!doneMaze && start == 1) {
  
  // if (!visited.isEmpty()) { // this doesn't work because we always push one node to the stack
    
    curr_pos = visited.peek(); //current position is peek of stack
    visited.pop(); 
    curr_x = curr_pos.x; 
    curr_y = curr_pos.y;

    int temp_North = go_north;
    int temp_East = go_east;
    int temp_West = go_west;
    int temp_South = go_south;

    if (maze[curr_x][curr_y].visitedVar == 0){
      if (temp_North) BackTrack[curr_x][curr_y] = 0;
      if (temp_East) BackTrack[curr_x][curr_y] = 1;
      if (temp_West) BackTrack[curr_x][curr_y] = 2;
      if (temp_South) BackTrack[curr_x][curr_y] = 3;
    }
    

    checkWalls();
    go_north = 0;
    go_east = 0;
    go_west = 0;
    go_south = 0;
    int wall_bin;
    bitWrite(wall_bin, 0, bitRead(wall_loc[curr_x][curr_y],0));
    bitWrite(wall_bin, 1, bitRead(wall_loc[curr_x][curr_y],1));
    bitWrite(wall_bin, 2, bitRead(wall_loc[curr_x][curr_y],2));
    bitWrite(wall_bin, 3, bitRead(wall_loc[curr_x][curr_y],3));


//    else if (bitRead(wall_bin,0) == 0 && explored[curr_x][curr_y+1]==0) 
//        go_north =1;
//    else if (bitRead(wall_bin,1) == 0 && explored[curr_x+1][curr_y]==0)
//        go_east =1;
//    else if (bitRead(wall_bin,2) == 0 && explored[curr_x-1][curr_y]==0)
//        go_west =1;
//    else if (bitRead(wall_bin,3) == 0 && explored[curr_x][curr_y-1]==0)
//        go_south=1;


//    else if (temp_North) go_south = 1;
//    else if (temp_East) go_west = 1;
//    else if (temp_West) go_east = 1;
//    else if (temp_South) go_north = 1;


    //Serial.println(go_west);
    //Serial.println(Direction);
    
    if ((bitRead(wall_bin,0) ||maze[curr_x][curr_y+1].visitedVar ==1) && maze[curr_x][curr_y].visitedVar == 0)
        explored[curr_x][curr_y] = explored[curr_x][curr_y] - 1;
   
    if ((bitRead(wall_bin,1) ||maze[curr_x+1][curr_y].visitedVar ==1) && maze[curr_x][curr_y].visitedVar == 0)
        explored[curr_x][curr_y] = explored[curr_x][curr_y] - 1;
    
    if ((bitRead(wall_bin,2) ||maze[curr_x-1][curr_y].visitedVar ==1) && maze[curr_x][curr_y].visitedVar == 0)
        explored[curr_x][curr_y] = explored[curr_x][curr_y] - 1;
        
    if ((bitRead(wall_bin,3) ||maze[curr_x][curr_y-1].visitedVar ==1) && maze[curr_x][curr_y].visitedVar == 0)
        explored[curr_x][curr_y] = explored[curr_x][curr_y] - 1;

    if (curr_x+1 < 4 && maze[curr_x+1][curr_y].visitedVar ==1 && maze[curr_x][curr_y].visitedVar==0 && bitRead(wall_bin,1) == 0){
        explored[curr_x+1][curr_y] = explored[curr_x+1][curr_y] - 1;
    }
    if (curr_x-1 > -1 && maze[curr_x-1][curr_y].visitedVar ==1 && maze[curr_x][curr_y].visitedVar==0 && bitRead(wall_bin,2) == 0) {
        explored[curr_x-1][curr_y] = explored[curr_x-1][curr_y] - 1;
    }
    if (curr_y+1 < 5 && maze[curr_x][curr_y+1].visitedVar ==1 && maze[curr_x][curr_y].visitedVar==0 && bitRead(wall_bin,0) == 0) {
        explored[curr_x][curr_y+1] = explored[curr_x][curr_y+1] - 1;
    }
    if (curr_y-1 > -1 && maze[curr_x][curr_y-1].visitedVar ==1 && maze[curr_x][curr_y].visitedVar==0 && bitRead(wall_bin,3) == 0) {
        explored[curr_x][curr_y-1] = explored[curr_x][curr_y-1] - 1;
    }
    
    if (bitRead(wall_bin,0) ||maze[curr_x][curr_y+1].visitedVar ==1){
        if (bitRead(wall_bin,1)|| maze[curr_x+1][curr_y].visitedVar ==1) { 
            if(bitRead(wall_bin,2) || maze[curr_x-1][curr_y].visitedVar ==1)  { 
                if (bitRead(wall_bin,3) || maze[curr_x][curr_y-1].visitedVar ==1){
                    explored[curr_x][curr_y] = 0;
                }
            }
        }
    }

    int checkBackTrack =0;

    if (bitRead(wall_bin,0) == 0 && explored[curr_x][curr_y+1]==4&& !(prev_pos.x == curr_x &&prev_pos.y==curr_y+1)) 
        go_north =1;
    else if (bitRead(wall_bin,1) == 0 && explored[curr_x+1][curr_y]==4 && !(prev_pos.x == curr_x+1 &&prev_pos.y==curr_y))
        go_east =1;
    else if (bitRead(wall_bin,2) == 0 && explored[curr_x-1][curr_y]==4 && !(prev_pos.x == curr_x-1 &&prev_pos.y==curr_y))
        go_west =1;
    else if (bitRead(wall_bin,3) == 0 && explored[curr_x][curr_y-1]==4 && !(prev_pos.x == curr_x &&prev_pos.y==curr_y-1))
        go_south=1;
    else if (bitRead(wall_bin,0) == 0 && explored[curr_x][curr_y+1]==3 &&  !(prev_pos.x == curr_x &&prev_pos.y==curr_y+1)) 
        go_north =1;
    else if (bitRead(wall_bin,1) == 0 && explored[curr_x+1][curr_y]==3 && !(prev_pos.x == curr_x+1 &&prev_pos.y==curr_y))
        go_east =1;
    else if (bitRead(wall_bin,2) == 0 && explored[curr_x-1][curr_y]==3 && !(prev_pos.x == curr_x-1 &&prev_pos.y==curr_y))
        go_west =1;
    else if (bitRead(wall_bin,3) == 0 && explored[curr_x][curr_y-1]==3 && !(prev_pos.x == curr_x &&prev_pos.y==curr_y-1))
        go_south=1;
    else if (bitRead(wall_bin,0) == 0 && explored[curr_x][curr_y+1]==2 &&  !(prev_pos.x == curr_x &&prev_pos.y==curr_y+1)) 
        go_north =1;
    else if (bitRead(wall_bin,1) == 0 && explored[curr_x+1][curr_y]==2 &&  !(prev_pos.x == curr_x+1 &&prev_pos.y==curr_y))
        go_east =1;
    else if (bitRead(wall_bin,2) == 0 && explored[curr_x-1][curr_y]==2 &&  !(prev_pos.x == curr_x-1 &&prev_pos.y==curr_y))
        go_west =1;
    else if (bitRead(wall_bin,3) == 0 && explored[curr_x][curr_y-1]==2 && !(prev_pos.x == curr_x &&prev_pos.y==curr_y-1))
        go_south=1;
    else if (bitRead(wall_bin,0) == 0 && explored[curr_x][curr_y+1]==1 &&  !(prev_pos.x == curr_x &&prev_pos.y==curr_y+1)) 
        go_north =1;
    else if (bitRead(wall_bin,1) == 0 && explored[curr_x+1][curr_y]==1 &&  !(prev_pos.x == curr_x+1 &&prev_pos.y==curr_y))
        go_east =1;
    else if (bitRead(wall_bin,2) == 0 && explored[curr_x-1][curr_y]==1 &&  !(prev_pos.x == curr_x-1 &&prev_pos.y==curr_y))
        go_west =1;
    else if (bitRead(wall_bin,3) == 0 && explored[curr_x][curr_y-1]==1 &&  !(prev_pos.x == curr_x &&prev_pos.y==curr_y-1))
        go_south=1;
    else{
      checkBackTrack=1;
      }
   curr_pos.visitedVar = maze[curr_x][curr_y].visitedVar = 1; //mark as visited

   if(checkBackTrack==1) {
      if (BackTrack[curr_x][curr_y] == 0) {
        //Serial.println("backtrack south");
        go_south = 1;
      }
      if (BackTrack[curr_x][curr_y] == 1) {
        go_west = 1;
        //Serial.println("backtrack west");
      }
      if (BackTrack[curr_x][curr_y] == 2) {
        go_east = 1;
        //Serial.println("backtrack east");
      }
      if (BackTrack[curr_x][curr_y] == 3) {
        go_north = 1;
        //Serial.println("backtrack north");
      }
    }
  if (go_north) {
    if(Direction==0){goStraight=1; 
      turnLeft=turnRight=turn180=0;}
    else if(Direction==1){ turnLeft=1;
      goStraight=turnRight=turn180=0; }
    else if(Direction==2){turnRight=1;
      turnLeft=goStraight=turn180=0; }
    else if(Direction==3){turn180=1;
      turnLeft=turnRight=goStraight=0;}
    xory=1;
    PlusMinus = 1;
      next_pos = maze[curr_x][curr_y + 1];
      visited.push(next_pos);
      Direction = 0;
      //Serial.println("Go North");
    }
  else if (go_east) {
     if(Direction==1){goStraight=1; 
      turnLeft=turnRight=turn180=0;}
     else if(Direction==3){ turnLeft=1;
      goStraight=turnRight=turn180=0; }
     else if(Direction==0){turnRight=1;
      turnLeft=goStraight=turn180=0; }
     else if(Direction==2){turn180=1;
      turnLeft=turnRight=goStraight=0;}
      xory=0;
      PlusMinus = 0;
      next_pos = maze[curr_x + 1][curr_y];
      visited.push(next_pos);
      Direction = 1;
      //Serial.println("Go East");
    }
  else if (go_west) {
    if(Direction==2){goStraight=1; 
      turnLeft=turnRight=turn180=0;Serial.println("wrong 2");}
    else if(Direction==0){ turnLeft=1;
      goStraight=turnRight=turn180=0;Serial.println("wrong 0");}
    else if(Direction==3){turnRight=1;
      turnLeft=goStraight=turn180=0;Serial.println("wrong 3");}
    else if(Direction==1){
      turn180=1;
      turnLeft=0;
      turnRight=0;
      goStraight=0;
      Serial.println("correct 1");
      }
      
      xory=0;
      PlusMinus = 1;
      next_pos = maze[curr_x - 1][curr_y];
      visited.push(next_pos);
      //Serial.println("Go West");
      //Serial.println(turn180);
      //Serial.println(Direction);
      Direction = 2;
      
    }
  else if (go_south) {
    if(Direction==3){goStraight=1; 
      turnLeft=turnRight=turn180=0;}
    else if(Direction==2){ turnLeft=1;
      goStraight=turnRight=turn180=0; }
    else if(Direction==1){turnRight=1;
      turnLeft=goStraight=turn180=0; }
    else if(Direction==0){turn180=1;
      turnLeft=turnRight=goStraight=0;}
      xory=1;
      PlusMinus = 0;
      next_pos = maze[curr_x][curr_y - 1];
      visited.push(next_pos);
      Direction = 3;
      //Serial.println("Go South");
    }
  
    prev_pos = curr_pos;
    doneMaze =1;
    for (int j = height - 1; j >=0; j--) {
      for (int i = 0; i < width; i++) {
      //Serial.print(maze[i][j].visitedVar);Serial.print(", ");
      doneMaze=doneMaze&&(explored[i][j]==4||explored[i][j]==0);
    }
    if (doneMaze == 1)
      //done = 1;
    //Serial.println(" ");
      done = 1; // Send done signal
  }

  //Serial.println("Explored Array");
        
//          for (int n=4; n>=0; n--) {
//            for (int m=0; m<4; m++) {
//          
//             printf("%d, ",explored[m][n]);
//            
//          }
//          printf("\n");
//        }

  
 }
  else if(doneMaze){
    Serial.print("Maze is complete");
    left.write(92);
    right.write(88);
    while(1);
  }
}

void RADIO(void)
{
  //
  // Ping out role.  Repeatedly send the current time
  //
  //wall =  frontWallSensor<< 2| leftWallSensor<<1 | rightWallSensor;
  wall =  leftWallSensor<< 2| rightWallSensor<<1 | frontWallSensor;



  if (role == role_ping_out)
  {
    // First, stop listening so we can talk.
    radio.stopListening();

        data_transmit = xory<<7 | PlusMinus<<6 | treasure<<4 | wall<<1 | moved;

        if (done) data_transmit = 3<<4 | 1;
        
        //Serial.print("Now sending: ");
        //Serial.println(data_transmit,BIN);
        
        ok = radio.write( &data_transmit, sizeof(unsigned char) ); // maze[m][n]
     //   n++;

        moved = 0;

        if (PlusMinus) Serial.print("+");
        else Serial.print("-");
        if (xory) Serial.print("y");
        else Serial.print("x");
        if (wall & 0x4) Serial.print(", left wall");
        if (wall & 0x2) Serial.print(", right wall");
        if (wall & 0x1) Serial.print(", front wall");
        //Serial.println(" ");

    if (ok)
      printf("ok...");
    else
      printf("failed.\n\r");

    // Now, continue listening
    radio.startListening();

    // Wait here until we get a response, or timeout (250ms)
    unsigned long started_waiting_at = millis();
    bool timeout = false;
    while ( ! radio.available() && ! timeout )
      if (millis() - started_waiting_at > 200 )
        timeout = true;

    // Describe the results
    if ( timeout )
    {
      //printf("Failed, response timed out.\n\r");
    }
    else
    {
      // Grab the response, compare, and send to debugging spew
      unsigned long got_info;
      radio.read( &got_info, sizeof(unsigned long) );

      // Spew it
      //printf("Got response %lu, round-trip delay: %lu\n\r",got_info,millis()-got_info);
    }

    // Try again 1s later
    delay(250);

    }
       
  if ( role == role_pong_back )
  {
    // if there is data ready
    if ( radio.available() )
    {
      // Dump the payloads until we've gotten everything
      unsigned long got_info;
      bool done = false;
      while (!done)
      {

        // Fetch the payload, and see if this was the last one.
        done = radio.read( &got_info, sizeof(unsigned long) );

        // Spew it
        //printf("Got payload %d...\n",got_info);

//        for (int m=0; m<4; m++) {
//          for (int n=0; n<5; n++) {
//          
//             printf("%d, ",maze[m][n]);
//            
//          }
//          printf("\n");
//        }

        delay(20);

      }

      // First, stop listening so we can talk
      radio.stopListening();

      // Send the final one back.
      radio.write( &got_info, sizeof(unsigned long) );
      printf("Sent response.\n\r");

      // Now, resume listening so we catch the next packets.
      radio.startListening();
    }
  }

  if (1)
    {
      //printf("*** CHANGING TO TRANSMIT ROLE -- PRESS 'R' TO SWITCH BACK\n\r");

      // Become the primary transmitter (ping out)
      role = role_ping_out;
      radio.openWritingPipe(pipes[0]);
      radio.openReadingPipe(1,pipes[1]);
    }

}


