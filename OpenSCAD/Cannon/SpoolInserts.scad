
diameterOut = 52;
diameterIn = 15;
diameterLip = diameterOut + 10;
height = 28;
lipHeight = 8;
holeRradius = 5;
mount_radius0 = 15;	// mounting holes

smoothness = 180;


difference() {
    union() {
        cylinder(height-0.002, diameterOut/2, diameterOut/2, $fn = smoothness);
        cylinder(lipHeight, diameterLip/2, diameterLip/2, $fn = smoothness);
    }
    translate([0, 0, -0.001])
        cylinder(height, diameterIn/2, diameterIn/2, $fn = smoothness);
    // mounting holes:
    translate([0, 0, -0.001])
        for (a = [0:60:359]) rotate([0, 0, a]) {
            translate([0, mount_radius0, height/2])
            cylinder(r=holeRradius, h=height, center=true, $fn=40);
        }
}
