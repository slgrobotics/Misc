
thickness    = 6;		// round/rectangular base
pthickness   = 3;		// rectangular platform
ithickness   = 13;	// insert platform

servowidth  = 30;		// TowePro 9805BB
servolength = 66;
servoseatlength = 84;
servooffset = 3.5;	// horizontal, from center, to adjust contact between gears
servoheight = 16;		// room for upper portion of the servo
dgearcut    = 37;
gearoffset  = 20;

potdiameter = 7.5;		// for 5K servo feedback potentiometer
potbodydiameter = 18;
potbodyoffset = 3;

dp1 = 100;					// round platform diameter
dpi = 55;					// round insert diameter
sp1 = 100;					// platform side
mount_radius0 = 43;  	// mounting screws location - round platform.
mount_radius1 = 49.3;  	// mounting screws location - inner.
mount_radius2 = 57.5;  	// mounting screws location - corners.
m3_wide_radius = 1.7;
m4_wide_radius = 2.1;

module Platform()
{
	difference() {
		union() {
	  		//cylinder(r=dp1/2,h=thickness, $fn=120);		// for round base

	  		translate([0, 0, thickness/2]) cube([sp1,sp1,thickness], center = true);	// rectangular base
			translate([0, 0, thickness + pthickness/2]) cube([sp1,sp1,pthickness], center = true);
	  		translate([0, 0, thickness + pthickness]) cylinder(r=dpi/2,h=ithickness, $fn=120);
			translate([servooffset, 0, thickness/2]) cube([servoseatlength,servowidth,thickness], center = true);
		}

			for (a = [45:90:359]) rotate([0, 0, a]) {
	    		translate([0, mount_radius1-5, 0])
			   	cylinder(r=8, h=thickness);
			}

		// mounting holes:
		translate([0, 0, thickness - pthickness]) {
			for (a = [45:90:359]) rotate([0, 0, a]) {
	    		translate([0, mount_radius1, pthickness/2])
			   	cylinder(r=m3_wide_radius, h=thickness+pthickness, center=true, $fn=12);
			}
	
			for (b = [45:90:359]) rotate([0, 0, b]) {
				translate([0, mount_radius2, pthickness/2])
			      cylinder(r=m4_wide_radius, h=thickness+pthickness, center=true, $fn=12);
			}
		}
	}
}

module Turret1()
{
	color([1.00,0.75,0.75]) 
	difference() {

		Platform();

		for (a = [0:30:359]) rotate([0, 0, a]) {
    		translate([0, mount_radius0, thickness/2])
		   	cylinder(r=m3_wide_radius, h=2*thickness, center=true, $fn=12);
		}

		// cut for the servo:
		translate([servooffset, 0, 0]) cube([servolength,servowidth,servoheight*2], center = true);

		// cut for the POT cable:
		translate([-sp1/2+8, 0, 0]) cube([7,15,20*2], center = true);

		// pot holes:
		cylinder(r=potdiameter/2, h=(thickness+pthickness+ithickness)*2, center=true);
		cylinder(r=potbodydiameter/2+0.5, h=(thickness+pthickness+ithickness-potbodyoffset)*2, center=true);
		translate([-potbodydiameter*1.25, 0, thickness+ithickness/2]) cube([potbodydiameter*2.5,potbodydiameter-0.5,ithickness], center = true);
		translate([0, 8, thickness+ithickness/2]) cylinder(r=2, h=(thickness+pthickness+ithickness)*2, center=true, $fn=12);

		// servo shaft and gear cut:
		translate([servooffset+gearoffset, 0, thickness+pthickness])
			cylinder(r=dgearcut/2, h=ithickness, center=false, $fn=120);


		// cuts to release tension and save material:
		translate([0, servowidth/2+12, 0]) cube([30,14,thickness*2], center = true);
		translate([0, -(servowidth/2+12), 0]) cube([30,14,thickness*2], center = true);
	}
}

Turret1();

