//
// a throttle large pulley for Zero Turn Mower servo
// based on a windshield motor
//

cheekLowThickness    = 7.0;	  	//
cheekHighThickness   = 7.0;	  	//
betweenCheeks     = 6.0;

cheeksTotalThickness = cheekLowThickness + cheekHighThickness + betweenCheeks;

offset = 23.0;
thickness    = offset + cheeksTotalThickness;	  	//
diameter     = 85.0;

echo("thickness=", thickness);

shaftDiameter = 6.2;

screwDiameter = 3.0;
screwLength = 20.0;

diameter2     = 50.0;

leverX = diameter * 2;
leverY = 35.5;
leverZ = 2.5;

leverSize = [leverX,leverY,leverZ]; 

leverOffsetZ = thickness - leverZ + 0.01;

cheekDiameter1    = diameter;
cheekDiameter2    = diameter+10;
cheekHighOffset       = cheekLowThickness + betweenCheeks;

sideCutX = diameter * 2;
sideCutY = 35.5;
sideCutZ = thickness - cheekHighOffset - cheekHighThickness;
sideCutOffset = 7.2;

sideCutSize = [sideCutX,sideCutY,sideCutZ]; 

sideCutOffsetZ = cheekHighOffset + cheekHighThickness + 0.01;

lighterHolePos = 27.0;
lighterHoleDiam = 12.0;
lighterHoleDepth = cheeksTotalThickness + 10; // - 4.0;

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
	translate([0, 0, cheekHighOffset]) {
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
        cylinder(r=shaftDiameter/2, h=thickness + 0.02, $fn=60);

        cylinder(r1=12, r2=7.5, h=6, $fn=60);
   		//sphere(r=10);
    }
}

module PulleyBody()
{
    difference() {
        
        union() {
            cylinder(r=diameter/2,h=thickness, $fn=180);
            CheekLow();
            CheekHigh();
        }

        ShaftCut();
    }
}

module LeverHole(x, y)
{
    translate([x, y, -screwLength]) {	
        cylinder(r=screwDiameter/2, h=screwLength, $fn=12);
    }
}

module LeverCut()
{
    translate([0,0,leverZ/2 + leverOffsetZ]) {
        
        union() {
            cube(leverSize,center=true);
            
            LeverHole(11.0, 9.0);
            LeverHole(-8.0, -6.5);
            LeverHole(30.0, -9.5);
        }
    }
}

module SideCut(off)
{
    translate([0,off,sideCutZ/2 + sideCutOffsetZ]) {
        
        cube(sideCutSize,center=true);
    }        
}

module RoundCut()
{
    translate([0,0,sideCutOffsetZ]) {
        difference() {
            cylinder(r=diameter/2+0.01,h=sideCutZ+0.01, $fn=180);
            translate([0,0,-0.01])
            cylinder(r1=diameter2/2+15, r2=diameter2/2, h=sideCutZ+0.1, $fn=180);
            translate([diameter2,0,0]) {
                cube([diameter2*2,50,50],center=true);
            }
        }
    }
}

module Pulley()
{
    difference() {
        
        PulleyBody();

        LeverCut();

        SideCut(leverY/2 + sideCutY/2 + sideCutOffset);
        SideCut(-leverY/2 - sideCutY/2 - sideCutOffset);
        
        RoundCut();

		for (a = [0:60:359]) rotate([0, 0, a]) {
    		translate([0, lighterHolePos, lighterHoleDepth/2 - 0.01])
		   	cylinder(r1=lighterHoleDiam, r2=2, h=lighterHoleDepth, center=true, $fn=40);
		}
    }
}

Pulley();
