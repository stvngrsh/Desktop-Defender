
/* --------------------------------------------------------------------------
 * SimpleOpenNI
 * --------------------------------------------------------------------------
 * Processing Wrapper for the OpenNI/Kinect 2 library
 * http://code.google.com/p/simple-openni
 * --------------------------------------------------------------------------
 * prog:  Max Rheiner / Interaction Design / Zhdk / http://iad.zhdk.ch/
 * date:  12/12/2012 (m/d/y)
 * ----------------------------------------------------------------------------
 */
import SimpleOpenNI.*;

//Default processing library for serial communication
import processing.serial.*;

//New object for serial port
Serial myPort;

//New object for kinect sensor
SimpleOpenNI  kinect;

//Sets colors for images of people in the sensor
//This chunk of code copied from SimpleOpenNI example code
color[]       userClr = new color[]{ color(255,0,0),
                                     color(0,255,0),
                                     color(0,0,255),
                                     color(255,255,0),
                                     color(255,0,255),
                                     color(0,255,255)
                                   };
                                   
//Initialize variables for keypad control
boolean manualMode = false;
boolean leftKey = false;
boolean rightKey = false;
boolean upKey = false;
boolean downKey = false;
boolean space = false;

//Values for size of window
int screenx = 640;
int screeny = 480;

//Initialize variables for basic velocity calculation
int prevX = 0;
int xVel = 0;
int time = 0;
int prevTime = 0;

//Setup block - runs once at start
void setup()
{
  //Sets size of window to above values
  size(screenx, screeny);
  
  //Initialze kinect as a new SimpleOpenNI object
  kinect = new SimpleOpenNI(this);
  
  //Stops code from running and exits program if kinect doesn't initialize
  if(kinect.isInit() == false)
  {
     println("Kinect failed to open. Exiting program."); 
     exit();
     return;  
  }
  
  //Enable depthMap generation (for Kinect mode)
  kinect.enableDepth();
  //Enable RGB camera (for manual mode)
  kinect.enableRGB();
   
  //Enable human tracking
  kinect.enableUser();
 
  //Set background color
  background(200,0,0);

  //Set stroke color
  stroke(255,255,255);
  strokeWeight(2);
  
  //Print list of serial ports
  println(Serial.list());
  
  //Set myPort to USB Serial port
  myPort = new Serial(this, Serial.list()[11], 9600);
  myPort.clear(); 
}

//Main loop of program
void draw()
{
  
  //Fetch new data from kinect
  kinect.update();
  
  //If in Kinect mode (auto tracking mode)
  if(manualMode == false){
    
    //Display userImage on screen
    //Userimage is a depth-based image with color overlaying people
    //Flip image because kinect is upside down
    pushMatrix();
    scale(1.0,-1.0);
    image(kinect.userImage(),0,-kinect.userImage().height);
    popMatrix();
    
    //Draw a small rectangle in center of screen
    rect(screenx/2, screeny/2,15,15);
    
    //Aquire a list of people being tracked
    int[] userList = kinect.getUsers();
    
    //If at least one person is being tracked
    if(userList.length > 0)
    {
      //Create a variable for holding 3D coordinates
      PVector xyreal = new PVector();
  
      int userToTrack = 0;
      //Start this as something overly large
      float zDist = 10000;
      
      //Cycle though all users on screen
      //Tracks user that is closest to kinect
      for(int i = 1; i < userList.length; i++){
         //Get center of mass coordinate for user
         kinect.getCoM(userList[i],xyreal);
         //Set user to track as current user if they are closer
         if(xyreal.z < zDist){
           zDist = xyreal.z;
           userToTrack = i;
         }
      }
      
      //Get center of mass of the user found above
      kinect.getCoM(userList[userToTrack],xyreal);
      print(userList.length + "   ");
      
      //Create variable for holding 3D coordinates
      PVector xyproj = new PVector();
      
      //Convert kinect coordinates which are based in 3D space
      //to coordinates based off of 2D screen
      kinect.convertRealWorldToProjective(xyreal, xyproj);
      
      //Create individual values for each 3D position
      float x = xyproj.x;
      float y = screeny-xyproj.y;
      float z = xyproj.z;
     
      //Change fill color
      fill(255,255,255);      
      
      //Create coordinates based about the center point on the screen
      int xcenter = (int)x-screenx/2;
      int ycenter = (int)((-1)*(y-screeny/2));
      
      //Very quick and inaccurate velocity calculation
      //Provides a little compensation for moving targets
      //This should be beefed up if time permits
      time = millis();
      if(prevX != 0) {
        xVel = (xcenter - prevX)+90/(time - prevTime);
      }
      
      //Create values to send to the Arduino
      //X value is position + velocity compensation
      int xToSend = xcenter + xVel;
      
      //Y value is position plus constant of 80
      //Value of 80 roughty positions gun at torso
      //As you get far away from kinect, value of 80 should be less
      //But due to bullet drop, gun needs to be raised higher
      int yToSend = ycenter + 120;
      
      //Display coordinates on screen
      text("x: " + xcenter, 300, 100);
      text("y: " + ycenter, 300, 150);
      
      //Display a circle over the tracked target
      fill(255,0,0);
      ellipse(x,y,25,25);
  
      //Create a string to be sent to the Arduino 
      //Format is:
      //[Initialization character][x coordinate][,][y coordinate][F][0 or 1][\n]
      String toSend;
      
      //Value of 3500 is cutoff for when target is in range
      if(z < 3500){
        toSend = "Q" + xToSend + "," + yToSend + "F1"+ "\n";
      }
      else {
        toSend = "Q" + xToSend + "," + yToSend + "F0"+ "\n";
      }
  
      //Accounts for random spikes in kinect readings
      //Don't send data if kinect says x position is something 
      //is out of its range
      if(abs(xcenter) < 315){
        //Accounts for irratic movement when kinect is almost on 
        //target. Small tolerance given.
        if(abs(xcenter) > 15 || abs(ycenter) > 15){
          
            //Send the string to the Arduino
            myPort.write(toSend);
            println(toSend);

        }
      }
      
      //Displays a bunch of values on screen for testing
      textSize(32);
      text(z, 100, 25);
      text(xcenter-prevX, 100, 150);
      text(time-prevTime, 100, 200);
      fill(0,255,100);
      fill(255,0,255);
      //Dispaly circle over user with velocity compensation
      ellipse(x+xVel,y,25,25);
      
      //Set previous values for velocity compensation
      prevTime = time;
      prevX = xcenter;
    }
    
    //Reset velocity comp to 0 if user is lost
    else {
      prevX = 0;
      xVel = 0;
    }
    textSize(32);
    text(xVel, 100, 300);
    text(prevX, 100, 350);
    
  }
  
  //If manual mode is activated
  else{
    
    //Display flipped RGB camera image in screen
    pushMatrix();
    scale(1.0,-1.0);
    image(kinect.rgbImage(),0,-kinect.rgbImage().height);
    popMatrix();
    
    //Draw a small rectangle in center of screen
    rect(screenx/2, screeny/2,15,15);
    
    //Create string to send to Arduino
    //M is initialization character for manual mode
    String toSend = "M";
    
    //Add x coordinate to string based on left/right arrow keys
    if(leftKey){
      toSend = toSend + "100,";
    }
    else if(rightKey){
      toSend = toSend + "-100,";
    }
    else{
      toSend = toSend + "0,";
    }
   
    //Add y coordinate to string based on up/down arrow keys
    if(upKey){
      toSend = toSend + "100F";
    }
    else if(downKey){
      toSend = toSend + "-100F";
    }
    else{
      toSend = toSend + "0F";
    }
    
    //Add 0 or 1 to string based on space bar
    if(space){
      toSend = toSend + "1";
    }
    else{
      toSend = toSend + "0";
    }
    
    //Finish string with newline
    toSend = toSend + "\n";
    
    //Send string to Arduino
    myPort.write(toSend);
    println(toSend);
  }
}



// SimpleOpenNI events - copied from example code
// The only one that actually affects code is the first one
void onNewUser(SimpleOpenNI curContext, int userId)
{
  println("onNewUser - userId: " + userId);
  println("\tstart tracking skeleton");
  
  curContext.startTrackingSkeleton(userId);
}

void onLostUser(SimpleOpenNI curContext, int userId)
{
  println("onLostUser - userId: " + userId);
}

void onVisibleUser(SimpleOpenNI curContext, int userId)
{
  //println("onVisibleUser - userId: " + userId);
}

//Triggers when keys are pressed
//Used for manual mode
void keyPressed()
{
  //Enter/Return key enables/disables manualMode
  if (keyCode == RETURN || keyCode == ENTER)
  {
    manualMode = !manualMode;
  }
  
  if (keyCode == LEFT)
  {
    leftKey = true;
  }
  
  if (keyCode == RIGHT)
  {
    rightKey = true;
  }
  
  if (keyCode == UP)
  {
    upKey = true;
  }
  
  if (keyCode == DOWN)
  {
    downKey = true;
  }
  
  if (keyCode == ' ')
  {
    space = true;
  }
  
}

//Triggers when keys are released
//Used for manual mode
void keyReleased()
{
  if (keyCode == LEFT)
  {
    leftKey = false;
  }
  
  if (keyCode == RIGHT)
  {
    rightKey = false;
  }
  
  if (keyCode == UP)
  {
    upKey = false;
  }
  
  if (keyCode == DOWN)
  {
    downKey = false;
  }
  
  if (keyCode == ' ')
  {
    space = false;
  }
  
}

