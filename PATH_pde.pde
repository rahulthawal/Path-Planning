// Rahul A Thawal
// path Planning Algorithm.

import processing.opengl.*;
import processing.core.*;
import processing.video.*;
import peasy.*;


PVector y1,y2,x1,x2,r,normal,G;
PVector start,goal,obstacle;
PVector[] obstacles = new PVector[10];
PVector x,X;
int R = 160;

//Grid grid;
PeasyCam cam;


void setup(){
 
   size(1000, 1000, P3D); 
   cam = new PeasyCam(this, width/2, height/2, 0 ,1000);
   
   start = new PVector(10,10);
   goal = new PVector(1000,1000);
   
  
   obstacles[0] = new PVector(300,200);
   obstacles[1] = new PVector(200,400);
   obstacles[2] = new PVector(600,300);
   obstacles[3] = new PVector(400,500);
   obstacles[4] = new PVector(400,700);
   obstacles[5] = new PVector(200,800);
   obstacles[6] = new PVector(500,900);
   obstacles[7] = new PVector(600,700);
   obstacles[8] = new PVector(700,500);
   obstacles[9] = new PVector(800,400);
  
  
  frameRate(60);
} 
  


void draw(){
  
  
  background(245, 238, 184);
  fill(246, 225, 65);
  
  Draw_Cylinder();
  
  // Calculating Gradient Descent 
  x=grad(start,goal,obstacle);
  
  // Lambda is 5
  X=PVector.mult(x,5);
  
  // Subtract Starting position points 
  start=PVector.sub(start,(X));
  
  // Moving each coordinates of X & Y axis.
  translate(start.x,start.y,10);
  
  println(start);
  
  // Object thats needs to find path.
  fill(204, 102, 0);
  rect(30, 20, 55, 55);
  

     
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
    // Center point
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
    // Center point
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
  //fill(255, 100, 255); 
  fill(127,0,0);
  pushMatrix();
  translate(300,200,0);
  rotateX(PI/2); //Determine the position of cylinder
  drawCylinder(40, 40, 120, 64); 
  popMatrix();
  
  pushMatrix();
  translate(200,400,0);
  rotateX(PI/2);
  drawCylinder(40, 40, 120, 64); 
  popMatrix();
  
  
  pushMatrix();
  translate(400,500,0);
  rotateX(PI/2); //Determine the position of cylinder
  drawCylinder(50, 50, 130, 64); 
  popMatrix();
  
  
  pushMatrix();
  translate(400,700,0);
  rotateX(PI/2); //Determine the position of cylinder
  drawCylinder(50, 50, 130, 64); 
  popMatrix();

  pushMatrix();
  translate(200,800,0);
  rotateX(PI/2); //Determine the position of cylinder
  drawCylinder(50, 50, 130, 64);
  popMatrix();  
  
  pushMatrix();
  translate(500,900,0);
  rotateX(PI/2); //Determine the position of cylinder
  drawCylinder(50, 50, 130, 64); 
  popMatrix();  
  
  pushMatrix();
  translate(600,300,0);
  rotateX(PI/2); //Determine the position of cylinder
  drawCylinder(50, 50, 130, 64); 
  popMatrix();  
  
  pushMatrix();
  translate(600,700,0);
  rotateX(PI/2);//Determine the position of cylinder
  drawCylinder(50, 50, 130, 64); 
  popMatrix();  
  
  pushMatrix();
  translate(700,500,0);
  rotateX(PI/2); //Determine the position of cylinder
  drawCylinder(50, 50, 130, 64); 
  popMatrix();  
  
 pushMatrix();
  translate(800,400,0);
 rotateX(PI/2); //Determines the position of the cylinder.
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
  for (int i = 0; i < 10; i++) 
  { 
     float dist_object=PVector.dist(p,obstacles[i]);
     
     if(dist_object<=R && dist_object>0)
     {
        count=log(R/dist_object); 
       
     }
     else if (dist_object>R) 
     {
        count=0;
     }  
     
     c2=c2+count;
  }
  
 return (float)(c1+c2);
}




