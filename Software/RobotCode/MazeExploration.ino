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
int wL, wR, wF;
int m;
int n;
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
int Traversing = 1;

// Speeds for motors
int Lforward = 95; // was 100
int Lbackward = 85; // was 80
int Stop = 90;
int Rforward = 85; // was 80
int Rbackward = 95; // was 100

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

int _7kHz = 2;
int _12kHz = 3;
int _17kHz = 4;
int _660Hz = 8;

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

//delay(10000);
  
  Serial.begin(9600);
  left.attach(6);
  right.attach(5);
  left.write(90);            
  right.write(90);
  
  pinMode(LeftFront, INPUT);
  pinMode(RightFront, INPUT);
  
  pinMode(LeftRear, INPUT);
  //pinMode(RightRear, INPUT);
  
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
  pinMode(IR_FOR, INPUT);

  pinMode(2,INPUT);

  // Initialize Maze
  for (int i = 0; i < width; i++) {
    for (int j = 0; j < height; j++) {
      maze[i][j].visitedVar = 0;
      maze[i][j].x = i;
      maze[i][j].y = j;
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
while(!digitalRead(_660Hz) || !digitalRead(2)); // Wait for push button to be pressed
Serial.println("660Hz Detected!");
}

void loop() {
  
  if (turnRight) {
        left.write(Lforward);
        right.write(Rbackward); // verify this works.. want right wheel to go backwards
        //Serial.println("Turn 90 Degrees Right");
        delay(300);
        while(analogRead(RightFront) < 700); // Wait here until right front sensor senses line
        //Serial.println("frontright Detected");
        turnRight = 0;
  } 
  if (turnLeft) {
        left.write(Lbackward);  // verify this works.. want left wheel to go backwards
        right.write(Rforward);
        //Serial.println("Turn 90 Degrees Left");
        delay(300);
        while(analogRead(LeftFront) < 700); // Wait here until left front sensor senses line
        //Serial.println("frontleft Detected");
        turnLeft = 0;
  }
  if (turn180) {
        left.write(Lforward);
        right.write(Rbackward); // verify this works.. want right wheel to go backwards
        //Serial.println("Turn 180 Degrees");
        delay(300);
        while(analogRead(RightFront) < 700); // Wait here until right front sensor senses line
        //Serial.println("frontright Detected part 1");
        delay(300);
        while(analogRead(RightFront) < 700); // Wait here until right front sensor senses line
        //Serial.println("frontright Detected part 2");
        turn180 = 0;
  }

  if (goStraight) {
        left.write(Lforward);
        right.write(Rforward); // verify this works.. want right wheel to go backwards
        //Serial.println("Go Straight");
        goStraight = 0;
  }

Serial.println("Line Following");
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
    Serial.print("left wall detected. Distance: ");
    Serial.println(distance);
  }

  if (i==1 && distance >20) {
    rightWallSensor = 0;
    //Serial.print("right wall not detected. Distance: ");
    //Serial.println(distance);
  }
  else if (i==1) {
    rightWallSensor = 1;
    Serial.print("right wall detected. Distance: ");
    Serial.println(distance);
  }

  if (i==2 && distance >20) {
    frontWallSensor = 0;
    //Serial.print("front wall not detected. Distance: ");
    //Serial.println(distance);
  }
  else if (i==2) {
    frontWallSensor = 1;
    Serial.print("front wall detected. Distance: ");
  }
  
  i++;
  
  }
  i = 0;

}
///////////////////////////////////////////////////////
void LineFollowing() {
  

    if (analogRead(LeftFront) > 700) LFCrossed = 1;
    else LFCrossed = 0;

   if (LFCrossed) Serial.println("Left Front crossed");

   //determine line status for right sensor
   if (analogRead(RightFront) > 700) RFCrossed = 1;
   else RFCrossed = 0;

   if (RFCrossed) Serial.println("Right Front crossed");
   
  //drive servos
   // if both sensors on either side of line
   if(!LFCrossed && !RFCrossed){
    left.write(Lforward);            
    right.write(Rforward);
    //Serial.println("drive1");
   }

   else if(LFCrossed && !RFCrossed){
    left.write(Lforward);          
    right.write(Rforward - 4); // slow down right wheel slightly
    //delay(100);
    Serial.println("drive2");
   }

   else if(!LFCrossed && RFCrossed){
    left.write(Lforward + 4); // slow down left wheel slightly            
    right.write(Rforward);
    //delay(100);
    Serial.println("drive3");
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

      
      left.write(Stop);
      right.write(Stop);

IR(); // check walls

RADIO();
      
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
    xory=1;
    PlusMinus = 1;
      next_pos = maze[curr_x][curr_y + 1];
      visited.push(next_pos);
      Direction = 0;
      Serial.println("Go North");
    }
    else if (go_east) {
      xory=0;
      PlusMinus = 0;
      next_pos = maze[curr_x + 1][curr_y];
      visited.push(next_pos);
      Direction = 1;
      Serial.println("Go East");
    }
    else if (go_west) {
       xory=0;
       PlusMinus = 1;
      next_pos = maze[curr_x - 1][curr_y];
      visited.push(next_pos);
      Direction = 2;
      Serial.println("Go West");
    }
    else if (go_south) {
      xory=1;
      PlusMinus = 0;
      next_pos = maze[curr_x][curr_y - 1];
      visited.push(next_pos);
      Direction = 3;
      Serial.println("Go South");
    }

   else{
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
   xory=1;
   PlusMinus = 1;
      next_pos = maze[curr_x][curr_y + 1];
      visited.push(next_pos);
      Direction = 0;
      Serial.println("Go North back");
    }
    else if (go_east) {
       xory=0;
       PlusMinus = 0;
      next_pos = maze[curr_x + 1][curr_y];
      visited.push(next_pos);
      Direction = 1;
      Serial.println("Go East back");
    }
    else if (go_west) {
       xory=0;
       PlusMinus = 1;
      next_pos = maze[curr_x - 1][curr_y];
      visited.push(next_pos);
      Direction = 2;
      Serial.println("Go West back");
    }
    else if (go_south) {
       xory=1;
       PlusMinus = 0;
      next_pos = maze[curr_x][curr_y - 1];
      visited.push(next_pos);
      Direction = 3;
      Serial.println("Go South back");
    }

}

    for (int j = height - 1; j >=0; j--) {
      for (int i = 0; i < width; i++) {
      Serial.print(maze[i][j].visitedVar);Serial.print(", ");
      }
      Serial.println(" ");
    }
    Serial.println(" ");
  }

  else if(Traversing==0){
    Serial.print("Maze is complete");
    left.write(92);
    right.write(88);
    //while(1);
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
        Serial.print("Now sending: ");
        Serial.println(data_transmit,BIN);
        
        ok = radio.write( &data_transmit, sizeof(unsigned char) ); // maze[m][n]
        n++;

        moved = 0;

        if (PlusMinus) Serial.print("+");
        else Serial.print("-");
        if (xory) Serial.print("y");
        else Serial.print("x");
        if (wall & 0x4) Serial.print(", left wall");
        if (wall & 0x2) Serial.print(", right wall");
        if (wall & 0x1) Serial.print(", front wall");
        Serial.println(" ");
        

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
      printf("Failed, response timed out.\n\r");
    }
    else
    {
      // Grab the response, compare, and send to debugging spew
      unsigned long got_info;
      radio.read( &got_info, sizeof(unsigned long) );

      // Spew it
      printf("Got response %lu, round-trip delay: %lu\n\r",got_info,millis()-got_info);
    }

    // Try again 1s later
    delay(250);

    }
       




////////////////////////////////////////////////
  //
  // Pong back role.  Receive each packet, dump it out, and send it back
  //

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

        //if (Direction && xory && moved) x++;
        //if (!Direction && xory && moved) x--;
        //if (Direction && !xory && moved) y++;
        //if (!Direction && !xory && moved) y--;

        // Spew it
        printf("Got payload %d...\n",got_info);

        for (int m=0; m<4; m++) {
          for (int n=0; n<5; n++) {
          
             printf("%d, ",maze[m][n]);
            
          }
          printf("\n");
        }


        // Delay just a little bit to let the other unit
        // make the transition to receiver
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
/////////
///////////////////////////////////////
  //
  // Change roles
  //

  if (1)
    {
      printf("*** CHANGING TO TRANSMIT ROLE -- PRESS 'R' TO SWITCH BACK\n\r");

      // Become the primary transmitter (ping out)
      role = role_ping_out;
      radio.openWritingPipe(pipes[0]);
      radio.openReadingPipe(1,pipes[1]);
    }

}
