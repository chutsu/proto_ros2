$fn=32;

mav_width = 100.0;
mav_height = 50.0;
mav_arm_thickness = 8.0;
mav_arm_length= 450.0 / 2.0;
mav_arm_height= 10;
mav_prop_length = 100;

mav_body_color = [0.2, 0.2, 0.2];
mav_front_propeller_color = [0, 0, 1];
mav_back_propeller_color = [1, 0, 0];
// mav_front_propeller_color = [0.5, 0.5, 0.5];
// mav_back_propeller_color = [0.5, 0.5, 0.5];

// MAV
translate([0.0, 0.0, mav_height / 2]) {
  // MAV Body
  color(mav_body_color)
    cube([mav_width, mav_width, mav_height], center=true);

  // MAV Arms
  for (i = [0:3]) {
    color(mav_body_color)
      rotate([0, 0, 45 + i * 90])
        translate([mav_arm_length / 2, 0, mav_arm_height])
          cube([mav_arm_length, mav_arm_thickness, mav_arm_thickness], center=true);
  }

  // MAV Propeller
  pos = mav_arm_length * 0.7;
  h = mav_arm_height + mav_arm_thickness;

  color(mav_front_propeller_color)
    translate([pos, pos, h]) cylinder(r=mav_prop_length, center=true);
  color(mav_front_propeller_color)
    translate([pos, -pos, h]) cylinder(r=mav_prop_length, center=true);

  color(mav_back_propeller_color)
    translate([-pos, pos, h]) cylinder(r=mav_prop_length, center=true);
  color(mav_back_propeller_color)
    translate([-pos, -pos, h]) cylinder(r=mav_prop_length, center=true);
}
