//
// a pulley for Zero Turn Mower servo
// based on a windshield motor
//

thickness    = 30.0;	  	//
diameter     = 40.0;

mountDiameter = 10.0;

screwDiameter = 2.5;
screwLength = 200.0;
screwAngle = 50.0;
screwHeight1 = 15.0;
screwHeight2 = 24.0;

leverX = 10.5 * 2;
leverY = 20.8;
leverZ = 35.0;

leverSize = [leverX,leverY,leverZ]; 

leverOffsetZ = 9.5;

cheekLowThickness    = leverOffsetZ-0.01;	  	//
cheekHighThickness    = 8.0;	  	//
cheekDiameter1    = diameter;
cheekDiameter2    = 48.0;
cheekOffset       = thickness - cheekHighThickness;


module CheekLow()
{
   union() { 
    translate([0, 0, cheekLowThickness/2]) {
        cylinder(r1=cheekDiameter2/2, r2=cheekDiameter1/2, h=cheekLowThickness/2, $fn=180);
    }
    cylinder(r=cheekDiameter2/2, h=cheekLowThickness/2, $fn=180);
   }
}

module CheekHigh()
{
	translate([0, 0, cheekOffset]) {
       union() { 
        translate([0, 0, cheekHighThickness/2]) {
            cylinder(r=cheekDiameter2/2, h=cheekHighThickness/2, $fn=180);
        }
        cylinder(r1=cheekDiameter1/2, r2=cheekDiameter2/2, h=cheekHighThickness/2, $fn=180);
       }
    }
}

module ShaftCut()
{
    translate([0, 0, -0.01]) {	
        cylinder(r=mountDiameter/2, h=thickness + 0.02, $fn=60);
    }
}

module ScrewHole(height, angle)
{
    rotate([90, 0, angle + 90]) {	
        translate([0, height, 0]) {	
            cylinder(r=screwDiameter/2, h=screwLength, $fn=12);
        }
    }
}


module PulleyBody()
{
    difference() {
        
        union() {
            cylinder(r=diameter/2,h=thickness, $fn=180);
            CheekLow();
            //CheekHigh();
        }

        ShaftCut();
    }
    
}

module LeverCut()
{
    translate([0,0,leverZ/2 + leverOffsetZ]) {
        
        cube(leverSize,center=true);
        
        translate([leverX-0.05,0,0]) {
          cube([leverX,leverY,leverZ],center=true);
        }
    }
}

module Pulley()
{
    difference() {
        PulleyBody();
        LeverCut();
        //ScrewHole(screwHeight1,screwAngle);
        ScrewHole(screwHeight2,screwAngle);
        //ScrewHole(screwHeight1,-screwAngle);
        ScrewHole(screwHeight2,-screwAngle);
    }
}

Pulley();
//LeverCut();
