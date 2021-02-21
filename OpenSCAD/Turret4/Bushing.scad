
thicknessB   = 8;		// center pole at bushing
thicknessP   = 2.4;		// rest of the center pole
thicknesspl  = 6.5;  	// platform

dp1 = 7.1;				// center hole diameter
dpB = 10.0;				// center pole diameter at bushing
dpP = 9.5;				// center pole diameter
dp2 = 36;				// platform diameter

mount_radius0 = 10;	// mounting holes

m3_wide_radius = 1.6;

module Bushing()
{
	color([0.75,1.00,0.75])

	difference() {

		union()
		{
  			cylinder(r=dpP/2,h=thicknessB + thicknessP + thicknesspl, $fn=120);
  			cylinder(r=dpB/2,h=thicknessB + thicknesspl, $fn=120);
			cylinder(r=dp2/2, h=thicknesspl, $fn=120);
		}

		// center hole:
		difference() {
  			cylinder(r=dp1/2,h=thicknessB + thicknessP + thicknesspl, center=false, $fn=40);
			translate([7.2,0,0]) scale([1,1,1.4]) cube(10, center=true);
		}

		// screw hole:
		translate([0, 0, thicknesspl/2]) rotate(a=[0,90,0]) cylinder(r=m3_wide_radius, h=dp2/2+1, center=false, $fn=40);

		// mounting holes:
		for (a = [0:60:359]) rotate([0, 0, a]) {
    		translate([0, mount_radius0, thickness/2])
		   	cylinder(r=m3_wide_radius, h=thicknesspl, center=false, $fn=40);
		}
	}
}

Bushing();
