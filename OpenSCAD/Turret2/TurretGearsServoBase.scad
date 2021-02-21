
thickness    = 6;			// rectangular base
pthickness   = 3;			// rectangular platform
ithickness   = 9;			// insert platform

servowidth  = 29.6;		// HobbyKing 5030DX
servolength = 60;
servoseatlength = 84;
servooffset = 16;			// horizontal, from center, to adjust contact between gears
servoheight = 12;			// room for upper portion of the servo
dgearcut    = 48;
gearoffset  = 20;

dgearplatformcut0 = 86;			// a round cut for the gear platform
dgearplatformcut1 = 120;
hgearplatformcut = 5;

potdiameter = 7.5;		// for 5K servo feedback potentiometer
potbodydiameter = 18;
potbodyoffset = 3;

dp1 = 150;					// round platform diameter
dpi = 75;					// round insert diameter
sp1 = 150;					// platform side
mount_radius2 = 85;  	// mounting screws location - corners.
m3_wide_radius = 1.7;
m4_wide_radius = 2.6;

relief_radius0 = 68;  	// round relief cuts.
relief_radius1 = (dgearplatformcut0 + dgearplatformcut1)/4;

module Platform()
{
	difference() {
		union() {

	  		translate([0, 0, thickness/2]) cube([sp1,sp1,thickness], center = true);	// rectangular base
			translate([0, 0, thickness + pthickness/2]) cube([sp1,sp1,pthickness], center = true);
	  		translate([0, 0, thickness + pthickness]) cylinder(r=dpi/2,h=ithickness, $fn=120);
			translate([servooffset, 0, thickness/2]) cube([servoseatlength,servowidth,thickness], center = true);
		}

		// make it round and cut around the corners to save material and lessen size:
	  	translate([0, 0, 0]) {
			difference() {
				cylinder(r=dp1, h=100, $fn=120);
				cylinder(r=mount_radius2 + 5, h=100, $fn=120);
			}
		}
		for (a = [0:30:359]) rotate([0, 0, a]) {
    		translate([0, mount_radius2+4, thickness/2])
		   	cylinder(r=17, h=2*thickness, center=true, $fn=12);
		}

		// a round cut to accommodate gear platform's bottom:
	  	translate([0, 0, thickness + pthickness - hgearplatformcut]) {
			difference() {
				cylinder(r=dgearplatformcut1/2, h=hgearplatformcut, $fn=120);
				cylinder(r=dgearplatformcut0/2, h=hgearplatformcut, $fn=120);
			}
		}

		// mounting holes:
		translate([0, 0, thickness - pthickness]) {
			for (b = [45:90:359]) rotate([0, 0, b]) {
				translate([0, mount_radius2, pthickness/2])
			      cylinder(r=m4_wide_radius, h=thickness+pthickness, center=true, $fn=12);
			}
		}

		//cube([sp1,sp1,8], center = true);

	}
}

module Turret1()
{
	color([1.00,0.75,0.75]) 
	difference() {

		Platform();

		// round relief cuts:
		for (a = [0:30:359]) rotate([0, 0, a+15]) {
    		translate([0, relief_radius0, thickness/2])
		   	cylinder(r=5, h=2*thickness, center=true, $fn=12);
		}
		for (a = [0:30:359]) rotate([0, 0, a]) {
			if(a!=270)
			{
    			translate([0, relief_radius1, thickness/2])
		   		cylinder(r=5, h=2*thickness, center=true, $fn=12);
			}
		}

		// cut for the servo:
		translate([servooffset, 0, 0]) cube([servolength,servowidth,servoheight*2], center = true);

		// cut for the POT cable:
		translate([-dgearplatformcut0/2+7, 0, 0]) cube([7,17.5,100], center = true);

		// pot holes:
		cylinder(r=potdiameter/2, h=(thickness+pthickness+ithickness)*2, center=true);
		cylinder(r=potbodydiameter/2+0.5, h=(thickness+pthickness+ithickness-potbodyoffset)*2, center=true);
		translate([-potbodydiameter*1.25, 0, thickness+ithickness/2]) cube([potbodydiameter*1.9,potbodydiameter-0.5,ithickness], center = true);
		translate([0, 8, thickness+ithickness/2]) cylinder(r=2, h=(thickness+pthickness+ithickness)*2, center=true, $fn=12);

		// servo shaft and gear cut:
		translate([servooffset+gearoffset, 0, thickness + pthickness - hgearplatformcut])
			cylinder(r=dgearcut/2, h=ithickness+hgearplatformcut, center=false, $fn=120);


		// cuts to release tension and save material:
		translate([0, servowidth/2+12, 0]) cube([30,12,(thickness+ithickness+10)*2], center = true);
		translate([0, servowidth/2+17, thickness+pthickness+ithickness/2]) cube([100,22,ithickness], center = true);
		translate([0, -(servowidth/2+12), 0]) cube([30,12,(thickness+ithickness+10)*2], center = true);
		translate([0, -(servowidth/2+17), thickness+pthickness+ithickness/2]) cube([100,22,ithickness], center = true);	}
}

//translate([0, 0, -4])
 Turret1();

