//
// a slider for Zero Turn Mower servo
// based on a windshield motor
//

thickness   = 30.0;	  	//
width       = 45.0;
depth       = 32.0;

screwDiameter = 5.0;
screwLength = 200.0;
screwX = 10.0;
screwY = 13.0;

rodDiameter = 12.0;
rodHeight = 22.5;



module SliderBody()
{
    translate([0, 0, thickness/2]) {
      cube([width, depth, thickness],center=true);
    }
}

module ScrewHole(dx, dy)
{
    rotate([0, 0, 90]) {	
        translate([dx, dy, -1]) {	
            cylinder(r=screwDiameter/2, h=screwLength, $fn=12);
        }
    }
}

module RodHole(elev)
{
    rotate([90, 0, 0]) {	
        translate([0, elev, -(depth/2 + 5)]) {	
            cylinder(r=rodDiameter/2, h=depth+10, $fn=120);
        }
    }
}

module Rod()
{
    rodRadius = rodDiameter/2;
    midThickness = thickness/2;

    RodHole(midThickness - rodHeight/2 + rodRadius);
    
    RodHole(midThickness + rodHeight/2 - rodRadius);
    
    cubeHeight = rodHeight-rodDiameter;
    
    translate([0,0,midThickness]) {
      cube([rodDiameter,depth+10,cubeHeight],center=true);
    }
}

module Slider()
{
    difference() {

        SliderBody();
        
        ScrewHole(screwX, screwY);
        ScrewHole(-screwX, screwY);
        ScrewHole(screwX, -screwY);
        ScrewHole(-screwX, -screwY);
        
        Rod();
    }
}

Slider();
