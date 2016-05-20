// Rahul A Thawal Social Force Model Three Objects Passing By.

import processing.opengl.*;
import processing.core.*;
import processing.video.*;
import peasy.*;


PVector x1,y1,y2,x2,r,normal,G;

PVector start,goal,start11,goal11,start22,goal22,obstacle;

PVector[] obstacles = new PVector[10];

PVector x,x_11,x_22,X,X_11,X_22;

PeasyCam cam;


void setup(){
 
   size(1000, 1000, P3D); 
   cam = new PeasyCam(this, width/2, height/2, 0 ,1000);
   
   start = new PVector(10,10);
   goal = new PVector(1000,1000);
   
   start11 = new PVector(1000,1000);
   goal11 = new PVector(10,10);
   
   
   start22 = new PVector(10,1000);
   goal22 = new PVector(1000,10);
   
  
   obstacles[0] = new PVector(300,100);
   obstacles[1] = new PVector(300,400);
   obstacles[2] = new PVector(800,200);
   obstacles[3] = new PVector(700,400);
   obstacles[4] = new PVector(200,800);
   obstacles[5] = new PVector(400,700);
   obstacles[6] = new PVector(800,600);
   obstacles[7] = new PVector(900,800);
 //  obstacles[8] = new PVector(900,800);
 //  obstacles[9] = new PVector(800,400);
  
   
  

  frameRate(50);
} 
  


void draw(){
    background(245, 238, 184);
    fill(246, 225, 65);
    
 
  Draw_Cylinder();
  
  // Calculating Gradient Descent 
  x     =  grad(start,goal,obstacle);
  x_11  =  grad(start11,goal11,obstacle); 
  x_22  =  grad(start22,goal22,obstacle);
  
  // Lambda is 5
  X      =    PVector.mult(x,5);
  X_11   =    PVector.mult(x_11,5);
  X_22   =    PVector.mult(x_22,5);
  
 
  // Subtract starting position points 
  start    =  PVector.sub(start,(X));
  start11  =  PVector.sub(start11,(X_11));
  start22  =  PVector.sub(start22,(X_22));
 
  // Moving each coordinates of X & Y axis.
  
  
  // First Block Moving
  fill(204, 102, 0);
  pushMatrix();
  translate(start.x,start.y,10);
  rect(5, 20, 55, 55);
  popMatrix();
 
  
 // Second Block Moving
  fill(204, 102, 0);
  pushMatrix();
  translate(start11.x,start11.y,10);
  rect(30, 20, 55, 55);
  popMatrix();
 
   // First Block Moving
  fill(204, 102, 0);
  pushMatrix();
  translate(start22.x,start22.y,10);
  rect(30, 20, 55, 55);
  popMatrix();
 
  
 
  delay(10);
   
   
}



// Code to Draw Cylinder.

void drawCylinder(float topRadius, float bottomRadius, float tall, int sides) {
  float angle = 0;
  float angleIncrement = TWO_PI / sides;
  beginShape(QUAD_STRIP);
  for (int i = 0; i < sides + 1; ++i) {
    vertex(topRadius*cos(angle), 0, topRadius*sin(angle));
    vertex(bottomRadius*cos(angle), tall, bottomRadius*sin(angle));
    angle += angleIncrement;
  }
  endShape();
  
  // If it is not a cone, draw the circular top cap
  if (topRadius != 0) {
   angle = 0;
  beginShape(TRIANGLE_FAN);
    //Center point
   vertex(0, 0, 0);
   for (int i = 0; i < sides + 1; i++) {
     vertex(topRadius * cos(angle), 0, topRadius * sin(angle));
      angle += angleIncrement;
    }
    endShape();
  }

  // If it is not a cone, draw the circular bottom cap
  if (bottomRadius != 0) {
    angle = 0;
    beginShape(TRIANGLE_FAN);    
    // Center point;
    vertex(0, tall, 0);
    for (int i = 0; i < sides + 1; i++) {
      vertex(bottomRadius * cos(angle), tall, bottomRadius * sin(angle));
      angle += angleIncrement;
    }
    endShape();
  }
}


void Draw_Cylinder()
{
  fill(255, 100, 255); 
  fill(127,0,0);
  pushMatrix();
  translate(300,100,0);
  rotateX(PI/2); //Determine the position of cylinder
  drawCylinder(40, 40, 120, 64); 
  popMatrix();
  
  pushMatrix();
  translate(300,400,0);
  rotateX(PI/2);
  drawCylinder(40, 40, 120, 64); 
  popMatrix();
  
  
 pushMatrix();
 translate(800,200,0);
 rotateX(PI/2); //Determine the position of cylinder
 drawCylinder(50, 50, 130, 64); 
 popMatrix();
  
  
 pushMatrix();
 translate(700,400,0);
 rotateX(PI/2); //Determine the position of cylinder
 drawCylinder(50, 50, 130, 64); 
 popMatrix();

 pushMatrix();
 translate(200,800,0);
 rotateX(PI/2); //Determine the position of cylinder
 drawCylinder(50, 50, 130, 64);
 popMatrix();  
  
 pushMatrix();
 translate(400,700,0);
 rotateX(PI/2); //Determine the position of cylinder
 drawCylinder(50, 50, 130, 64); 
 popMatrix();  
  
 pushMatrix();
 translate(800,600,0);
 rotateX(PI/2); //Determine the position of cylinder
 drawCylinder(50, 50, 130, 64); 
 popMatrix();  
  
  pushMatrix();
  translate(900,800,0);
  rotateX(PI/2);//Determine the position of cylinder
  drawCylinder(50, 50, 130, 64); 
  popMatrix();   
  
 
}



PVector grad(PVector p,PVector q,PVector o){
// Calculate the cost of moving to locations of a 4-size neighborhood
//
//            ^ 
//            |
//           y1
//            |
//  <-- x2 --- p --- x1 --> 
//            |
//            y2
//            |
//
  y1 =new PVector(p.x,p.y+1);
  y2 =new PVector(p.x,p.y-1); 
  x1 =new PVector(p.x+1,p.y); 
  x2 =new PVector(p.x-1,p.y);
  
  float cx = Cpathplan( x1, q, o ) - Cpathplan( x2, q, o ); 
  float cy = Cpathplan( y1, q, o ) - Cpathplan( y2, q, o ); 
 
  
  r = new PVector(cx,cy); 
  
  float normal = sqrt(pow(r.x,2)+pow(r.y,2));
  
  G = new PVector();
  
  PVector.div(r,normal,G); 
 
  return G;
}

float Cpathplan(PVector p,PVector q,PVector o)
{
 double c1=PVector.dist(p,q)*0.02;
 double c2=0;
 double count=0;
  for (int i = 0; i < 8; i++) 
  { 
     float dist_object=PVector.dist(p,obstacles[i]);
     
     if(dist_object<=168 && dist_object>0)
     {
        count=log(168/dist_object); 
  
     }
     else if (dist_object>168) 
     {
        count=0;
     }  
     
     c2=c2+count;
  }
 return (float)(c1+c2);
}




