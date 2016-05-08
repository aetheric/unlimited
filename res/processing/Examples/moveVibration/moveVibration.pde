import PUnlimitedHand.*;
import processing.serial.*;
UnlimitedHand uh;

void setup(){
  size(200,200);
  noLoop();
  uh = new UnlimitedHand(this, 5); //Developer's TODO: specify the portNum
}

void draw(){
}

void mousePressed(){
  println('t');
  uh.moveVibration();
}