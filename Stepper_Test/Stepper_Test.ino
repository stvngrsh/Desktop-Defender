/*
Steven Gresh 
Christopher Sneeder
ENME351 - 0101
Final Lab Project

I pledge on my honor that I have not given or recieved any unauthorized
assistance on this assignment. 
*/

//Advanced stepper motor control for multiple motors at one time
//Copyright (C) 2010 Mike McCauley
#include <AccelStepper.h>

//Default Arduino library for basic servo motor control
#include <Servo.h>

//Create object for both stepper motors
AccelStepper tilt(1, 7, 6);
AccelStepper pan(1, 2, 3);

//Create object for the servo motor
Servo trigger;

//Variables for pan movement when in scan mode
int scanMode = 1;
int wait = 0;

//Variables for trigger and fire control
//Fireing requires multiple checks to ensure gun doesn't jam
boolean fired = false;
boolean reloading = false;
boolean fireInit = false;
boolean demoMode = true;
long fireTime = 0;
long reloadTime = 0;

//Variables defining output pins for LED strip colors
int red = 8;
int blue = 12;
int green = 13;

//Setup block - runs once at start
void setup() 
{
  //Start all LEDs in OFF state
  digitalWrite(red, LOW);
  digitalWrite(blue, LOW);
  digitalWrite(green, LOW);
  
  //Set max speed and accelerations for stepper motors
  pan.setMaxSpeed(600);
  pan.setAcceleration(500);
  tilt.setMaxSpeed(600);
  tilt.setAcceleration(500);
  
  //Initialize the stepper current positions as zero position
  pan.setCurrentPosition(0);
  tilt.setCurrentPosition(0);

  //Initialize digital input pins for switches
  pinMode(A0, INPUT); //Bumper
  pinMode(A1, INPUT); //Switch
  
  //Initialize digital output pins for LED strip
  pinMode(red, OUTPUT); //red
  pinMode(blue, OUTPUT); //blue
  pinMode(green, OUTPUT); //green

  //Initialize digital pin for servo motor
  trigger.attach(11);
  
  //Begin serial communication at 9600bps
  Serial.begin(9600);
  
  //Calibrate the zero position of the tilt moto
  //Move down slowly until bumper is pressed
  while(digitalRead(A0) == LOW){
    tilt.move(1);
    tilt.run();
  }
  tilt.setCurrentPosition(0);
}

//Main loop of program
void loop()
{ 
  //demoMode disables firing - state of switch determines this
  //Switch also controls gun internals (hardwired no code for that)
  if(digitalRead(A1) == LOW){
    demoMode = true;  //Gun can NOT fire
  }
  if(digitalRead(A1) == HIGH){
    demoMode = false;  //Gun can fire
  }
  
  //Initialize variables for serial communication
  String s;
  String sx;
  String sy;
  String sf;
  int xpos;
  int ypos;
  
  //Reset all LED states to OFF
  digitalWrite(red, LOW);
  digitalWrite(blue, LOW);
  digitalWrite(green, LOW);
  
  //LED color indicates gun status
  if(demoMode == true){
    digitalWrite(green, HIGH); //Green for safe (no firing)
  }
  else{
    digitalWrite(red, HIGH);  //Red for warning (gun can fire)
  }
  
  //Check for data on serial port
  if(Serial.available()){
    
    //Fill string with serial data until newline
    s = Serial.readStringUntil('\n');
    
    //Make sure full string is read by checking for an initialization character
    if(s.startsWith("Q") || s.startsWith("M")){
      
      //Reset no-data wait timer to 0
      wait = 0;
      
      //Turn on blue LEDs if in manual mode
      if(s.startsWith("M")){
        digitalWrite(blue, HIGH);
      }    
      
      //Fill substrings with appropriate data from processing
      sx = s.substring(1, s.indexOf(","));
      sy = s.substring(s.indexOf(",")+1, s.indexOf("F"));
      sf = s.substring(s.indexOf("F")+1);
      
      int xpos = sx.toInt(); //X position data (pan)
      int ypos = sy.toInt(); //Y position data (tilt)
      int f = sf.toInt();    //0 or 1 if target is in range to fire
      
      if(xpos != 0) {
        //Flip sign because of upsidedown kinect 
        //Divide by 2.5 to convert kinect resolution to physical steps
        int toMovePan = -xpos/(2.5);
        pan.move(toMovePan);
          
      }
      
      if(ypos != 0) {
        //Flip sign because of upsidedown kinect 
        //Divide by 2.5 to convert kinect resolution to physical steps
        int toMoveTilt = -ypos/(2.5);
        tilt.move(toMoveTilt);
       
      }
      
      //Run command steps the motors at most once per loop
      //Speed/Accel based on previous move command
      pan.run();
      tilt.run();
      
      //Check if target is in range
      if(f == 1){
        //If gun is not currently reloading or currently firing
        if(reloading == false && fired == false){
            //Begin firing sequence
            fired = true;
            if(s.startsWith("M")){
              fireInit = true;
              fireTime = millis();
            }
        }
      }
    } 
  }
  
  //If no data over serial, gun is in scan mode and periodically pans
  //left and right to check for other targets.
  else {
    
    if(scanMode == 1){
      //Pan 200 steps to the left
      pan.move(100);  
      //Move tilt position to zero
      tilt.moveTo(0);
    
      //Move on to next block of code
      scanMode++;
    }
    
    //Wait 5 seconds before paning to right
    //Timer resets if target is found
    else if(scanMode == 2){
      wait++;
      delay(1);
      if(wait > 5000){
        wait = 0;
        scanMode++;
        
      }
    }
    
    
    else if(scanMode == 3){
      //Pan 200 to the right
      pan.move(-100);
      //Move tilt position to zero
      tilt.moveTo(0);  
      
      //Move on to next block of code
      scanMode++;
    }
    
    //Wait 5 seconds before paning to left
    //Timer resets if target is found
    else if(scanMode == 4){
      wait++;
      delay(1);
      if(wait > 5000){
        wait = 0;
        scanMode = 1;
        
      }
    }
    
    //Steps motors based on previous move commands
    pan.run();
    tilt.run();
  }
  
  //If gun has been told to fire
  if(fired == true){
    
    //This waits to start firing sequence until gun is within 5 steps of target
    if(fireInit == false && pan.distanceToGo() < 5 && pan.distanceToGo() > 0){
      //Begins timer for firing
      fireTime = millis();
      
      //Firing sequence has started
      fireInit = true;
    }
    
    //If firing sequence has started
    if(fireInit == true){
      //Time between start of firing and now
      long tempTime = millis() - fireTime;
      
      //Gun takes 560 ms to fire
      if(tempTime < 560){
        
        //Don't actually shoot if in demoMode
        if(demoMode == false){
          trigger.write(0);
        }
      }
      
      //After 560 ms
      else{
        //Disable firing
        fired = false;       
        fireInit = false;

        //Begin reload sequence and timer
        reloading = true;
        reloadTime = millis();
      }
    }
  }
  
  //Reload sequence has started
  else if(reloading == true){
    //Time between start of reloading and now
    long tempTime = millis() - reloadTime;
    //Gun takes 560 ms to reload
    if(reloadTime < 560){
      //Don't actually reload if in demoMode
      if(demoMode == false){
        trigger.write(90);
      }
    }
    
    //After 560 ms
    else{
      //End reloading sequence
      reloading = false;
    }
  }
  
  //For any reason the gun is not firing and not reloading but the
  //Firing mechanism is not in the ready position.  
  else{
    if(demoMode == false){
      trigger.write(90);
    }
  }
  
  //If bumper switch is pressed
  //Kill tilt movement to decrease motor skipping
  if(digitalRead(A0) == HIGH){
    tilt.setCurrentPosition(0);
    tilt.moveTo(0);
  }  
  
}


