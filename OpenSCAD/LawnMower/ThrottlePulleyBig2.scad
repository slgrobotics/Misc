//
// a throttle large pulley for Zero Turn Mower servo
// based on a windshield motor
//

cheekLowThickness    = 7.0;	  	//
cheekHighThickness   = 7.0;	  	//
betweenCheeks     = 6.0;

cheeksTotalThickness = cheekLowThickness + cheekHighThickness + betweenCheeks;

coneThickness = 32.0;          // cone thickness
thickness    = coneThickness + cheeksTotalThickness;	  	//
diameter     = 90.0;    // this is diameter of the working face of the pulley. Cheeks extra.
diameter2    = 35.0;    // cone smaller side
roundPrecision = 720;

echo("thickness=", thickness);

shaftDiameter = 27.0; // actual shaft is 26.6 mm diameter

pinDiameter = 6.8;
pinOffset = cheeksTotalThickness + 4.0;

lighterHolePos = 26.0;
lighterHoleDiam = 20.0;
lighterHoleDepth = thickness; // cheeksTotalThickness + 15; // - 4.0;

screwDiameter = 3.5;
screwPos = diameter/2 - 3;
screwLength = cheeksTotalThickness + 0.2;

screwDiameter2 = 3.5;
screwOffset2 = cheeksTotalThickness / 2.0;
screwLength2 = diameter / 2.0 - lighterHolePos - 2.0;

cheekDiameter1    = diameter;
cheekDiameter2    = diameter+10;
cheekHighOffset       = cheekLowThickness + betweenCheeks;

sideCutX = diameter * 2;
sideCutY = 68.0;
sideCutZ = thickness;

sideCutSize = [sideCutX,sideCutY,sideCutZ]; 

sideCutOffsetZ = cheeksTotalThickness + 0.001;

module CheekLow()
{
   union() { 
    translate([0, 0, cheekLowThickness/2]) {
        cylinder(r1=cheekDiameter2/2, r2=cheekDiameter1/2, h=cheekLowThickness/2, $fn=roundPrecision);
    }
    cylinder(r=cheekDiameter2/2, h=cheekLowThickness/2, $fn=roundPrecision);
   }
}

module CheekHigh()
{
	translate([0, 0, cheekHighOffset]) {
       union() { 
        translate([0, 0, cheekHighThickness/2]) {
            cylinder(r=cheekDiameter2/2, h=cheekHighThickness/2, $fn=roundPrecision);
        }
        cylinder(r1=cheekDiameter1/2, r2=cheekDiameter2/2, h=cheekHighThickness/2, $fn=roundPrecision);
       }
    }
}

module ShaftCut()
{
    translate([0, 0, -0.01]) {	
        cylinder(r=shaftDiameter/2, h=thickness + 0.02, $fn=180);
    }
}

module PinCut()
{
    translate([0, 0, pinOffset]) {	
        rotate([90, 90, 0]) {	
            cylinder(r=pinDiameter/2, h=diameter*2, $fn=120, center=true);
        }
    }
}

module PulleyBody()
{
    difference() {
        
        union() {
            translate([0,0,cheeksTotalThickness]) {
                cylinder(r1=diameter/2, r2=diameter2/2, h=coneThickness, $fn=roundPrecision);
            }
            CheekLow();
            cylinder(r=diameter/2, h=cheeksTotalThickness, $fn=roundPrecision);
            CheekHigh();
        }

        ShaftCut();
    }
}

module SideCut(off)
{
    translate([0,off,sideCutZ/2 + sideCutOffsetZ]) {
        
        cube(sideCutSize,center=true);
    }        
}

module Pulley()
{
    difference() {
        
        PulleyBody();
        
        PinCut();

        SideCut(sideCutY);
        SideCut(-sideCutY);
        
        // holes to lighten the part, remove unnecessary material:
		for (a = [30:60:389]) rotate([0, 0, a]) {
    		translate([0, lighterHolePos, lighterHoleDepth/2 - 0.01])
		   	cylinder(r1=lighterHoleDiam/2, r2=2, h=lighterHoleDepth, center=true, $fn=6);
		}
        
        // set of 3 screws for the rotation sensor arm:
		for (a = [0:60:359]) rotate([0, 0, a]) {
    		translate([0, lighterHolePos+3, screwLength/2 - 0.01])
		   	cylinder(r=screwDiameter/2, h=screwLength, center=true, $fn=18);
		}
        
		for (a = [15:60:359]) rotate([0, 0, a]) {
    		translate([0, screwPos, screwLength/2 - 0.01])
		   	cylinder(r=screwDiameter/2, h=screwLength, center=true, $fn=18);
		}

		for (a = [-15:60:359]) rotate([0, 0, a]) {
    		translate([0, screwPos, screwLength/2 - 0.01])
		   	cylinder(r=screwDiameter/2, h=screwLength, center=true, $fn=18);
		}
        
        // screws between cheeks for cable stopper:
        translate([0, 0, screwOffset2]) {	
            for (a = [0:30:359]) rotate([0, 0, a]) {
                translate([0, diameter/2, 0])
                rotate([90, 90, 0]) {	
                    cylinder(r=screwDiameter2/2, h=screwLength2 * 2, $fn=12, center=true);
                }
            }
        }
    }
}

Pulley();
