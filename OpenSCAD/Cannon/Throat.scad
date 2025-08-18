
// millimeters

smoothnessL = 40;
smoothnessH = 360;
smoothnessM = 180;

diameterInL = 161;
diameterInH = 165;
diameterOut = 205;
height = 82;

smallR = 5;
smoothingR = 185;
cubeSide = 1000;



// Using a shape that touches the X axis is allowed and produces
// 3D objects that don't have a hole in the center.
color("green")
    translate([0, 0, height])
    rotate([180, 0, 0])
        main_cone();
        difference() {
            translate([0, 0, height-smallR+5])
                ending_ring();
            translate([0, -cubeSide/2, 0])
                cube(cubeSide,center=false);
            translate([0, 0, -10])
                cylinder(height+100, diameterInH/2, diameterInH/2, $fn = smoothnessM);
        }

module main_cone() {
        difference() {
            cylinder(height-0.002, diameterOut/2, diameterOut/2, $fn = smoothnessM);
            doughnut_cutout();
            translate([0, -cubeSide/2, 0])
                cube(cubeSide,center=false);
            translate([0, 0, -0.001])
            cylinder(height+0.002, diameterInH/2, diameterInL/2, $fn = smoothnessM);
        }
}

module doughnut_cutout() {
  rotate_extrude($fn = smoothnessM) {
    translate([smoothingR+diameterInL/2 + 5, 0, 0]) {
        circle(r=smoothingR, $fn = smoothnessH); // Define the 2D circle
    }
  }
}

module ending_ring() {
  rotate_extrude($fn = smoothnessM) {
    translate([diameterInL/2+smallR-2, 0, 0]) {
        circle(r=smallR, $fn = smoothnessL);
    }
  }
}

