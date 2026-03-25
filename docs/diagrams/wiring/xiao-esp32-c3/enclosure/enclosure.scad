// Enclosure for the XIAO ESP32-C3 IMU prototype.
// Coordinate system follows the prototype definition:
// - Board origin at [0,0,0]
// - +X along board long side, +Y along short side, +Z on component side.

// -------------------------------
// Rendering options
// -------------------------------
render_mode = "assembly";  // "assembly", "exploded", "base", "lid"
exploded_lid_offset_z = 22;

// -------------------------------
// Primary board dimensions
// -------------------------------
board_size_x = 50;
board_size_y = 70;
board_thickness = 1;
board_corner_hole_d = 2;
board_corner_hole_offset = 2.5;

// -------------------------------
// Mechanical clearances and walls
// -------------------------------
wall_thickness = 3;
base_floor_thickness = 3;
lid_roof_thickness = 3;
board_edge_clearance_xy = 1.5;

trace_side_clearance = 7;   // required below board copper side
component_side_clearance = 14;  // required above component side

// Derived cavity heights around the board plane z=0.
base_inner_depth = board_thickness + trace_side_clearance;
lid_inner_height = component_side_clearance;

base_total_height = base_inner_depth + base_floor_thickness;
lid_total_height = lid_inner_height + lid_roof_thickness;

// -------------------------------
// Fastener and insert dimensions
// -------------------------------
screw_nominal_d = 2;
screw_clearance_d = 2.4;
insert_pilot_d = 3.2;
insert_depth = 4;

// Corner post geometry (square posts integrated with corner walls).
corner_post_size = 8;

// -------------------------------
// External rounding
// -------------------------------
corner_radius = 2;

// -------------------------------
// Component coordinates (board frame)
// -------------------------------
button_center = [13, 12];
button_body_size = [12, 12, 12];
button_plunger_d = 1;

led_center = [25, 12];
led_body_d = 5;
led_window_clearance = 0.5;

switch_center = [39, 14];
switch_cutout_size = [21.5, 15.5];

// USB-C side cutout (assumed on +X wall).
usb_center_xy = [34, 50];
usb_cutout_span_z = [4, 7];
usb_cutout_size_y = 8.5;

// -------------------------------
// Button flexure slot in lid
// -------------------------------
button_slot_gap = 1;  // requested 1 mm spacing
button_slot_leg_length = 14;

// Keep 1-2 mm gap above button top.
button_press_clearance = 1.2;
button_pusher_d = button_plunger_d;

// -------------------------------
// Strap loops
// -------------------------------
strap_clearance = 5;
strap_frame_thickness = 5;
strap_loop_depth = 5;
strap_loop_corner_radius = 2;

// -------------------------------
// Axis engraving on lid top
// -------------------------------
engrave_depth = 1;
engrave_arrow_len = 10;
engrave_line_width = 1;
engrave_head_len = 2.5;
engrave_head_width = 2.2;

// -------------------------------
// Small tolerance for boolean operations
// -------------------------------
eps = 0.02;

// -------------------------------
// Derived footprint dimensions
// -------------------------------
case_inner_size = [
	board_size_x + 2 * board_edge_clearance_xy,
	board_size_y + 2 * board_edge_clearance_xy
];

case_outer_size = [
	case_inner_size[0] + 2 * wall_thickness,
	case_inner_size[1] + 2 * wall_thickness
];

case_center = [board_size_x / 2, board_size_y / 2];

case_min_x = case_center[0] - case_outer_size[0] / 2;
case_max_x = case_center[0] + case_outer_size[0] / 2;
case_min_y = case_center[1] - case_outer_size[1] / 2;
case_max_y = case_center[1] + case_outer_size[1] / 2;

strap_loop_outer_size_xz = [
	case_outer_size[0],
	strap_clearance + 2 * strap_frame_thickness
];

strap_loop_inner_size_xz = [
	strap_loop_outer_size_xz[0] - 2 * strap_frame_thickness,
	strap_loop_outer_size_xz[1] - 2 * strap_frame_thickness
];

board_mount_holes = [
	[board_corner_hole_offset, board_corner_hole_offset],
	[board_size_x - board_corner_hole_offset, board_corner_hole_offset],
	[board_corner_hole_offset, board_size_y - board_corner_hole_offset],
	[board_size_x - board_corner_hole_offset, board_size_y - board_corner_hole_offset]
];

// -------------------------------
// Utility geometry
// -------------------------------
module rounded_rect_2d(size_xy, radius) {
	// Clamp radius to avoid invalid rounded rectangles.
	safe_radius = min(radius, min(size_xy[0], size_xy[1]) / 2 - 0.01);
	offset(r = safe_radius)
		offset(delta = -safe_radius)
			square(size_xy, center = true);
}

module capsule_line_2d(start_pt, end_pt, width) {
	hull() {
		translate(start_pt) circle(d = width, $fn = 24);
		translate(end_pt) circle(d = width, $fn = 24);
	}
}

module arrow_2d(length, line_width, head_len, head_width) {
	union() {
		capsule_line_2d([0, 0], [length, 0], line_width);
		translate([length, 0])
			polygon(points = [
				[0, 0],
				[-head_len, head_width / 2],
				[-head_len, -head_width / 2]
			]);
	}
}

module strap_loop_profile_2d() {
	difference() {
		rounded_rect_2d(strap_loop_outer_size_xz, strap_loop_corner_radius);
		rounded_rect_2d(strap_loop_inner_size_xz, max(0.5, strap_loop_corner_radius - 0.5));
	}
}

module strap_loops_full() {
	// Loop on low-Y short side: extruded toward negative Y.
	translate([case_center[0], case_min_y, 0])
		rotate([90, 0, 0])
			linear_extrude(height = strap_loop_depth)
				strap_loop_profile_2d();

	// Loop on high-Y short side: extruded toward positive Y.
	translate([case_center[0], case_max_y, 0])
		rotate([-90, 0, 0])
			linear_extrude(height = strap_loop_depth)
				strap_loop_profile_2d();
}

module base_half_loops() {
	intersection() {
		strap_loops_full();
		translate([case_min_x - 100, case_min_y - 100, -200])
			cube([case_outer_size[0] + 200, case_outer_size[1] + 200, 200]);
	}
}

module lid_half_loops() {
	intersection() {
		strap_loops_full();
		translate([case_min_x - 100, case_min_y - 100, 0])
			cube([case_outer_size[0] + 200, case_outer_size[1] + 200, 200]);
	}
}

module board_corner_posts() {
	for (hole = board_mount_holes) {
		translate([
			hole[0] - corner_post_size / 2,
			hole[1] - corner_post_size / 2,
			-base_total_height
		])
			cube([corner_post_size, corner_post_size, base_total_height]);
	}
}

module board_corner_post_holes() {
	for (hole = board_mount_holes) {
		// Full through hole makes the opening visible from enclosure exterior.
		translate([hole[0], hole[1], -base_total_height - eps])
			cylinder(h = base_total_height + 2 * eps, d = screw_nominal_d, $fn = 40);

		// Insert seat at base exterior side so screws can span the full case height.
		translate([hole[0], hole[1], -base_total_height - eps])
			cylinder(h = insert_depth + eps, d = insert_pilot_d, $fn = 40);
	}
}

module button_slot_2d() {
	leg_offset_x = button_plunger_d / 2 + button_slot_gap / 2;

	union() {
		// U-slot side legs (parallel to Y axis).
		translate([button_center[0] - leg_offset_x, button_center[1] + button_slot_leg_length / 2])
			square([button_slot_gap, button_slot_leg_length], center = true);

		translate([button_center[0] + leg_offset_x, button_center[1] + button_slot_leg_length / 2])
			square([button_slot_gap, button_slot_leg_length], center = true);

		// Rounded U bottom matching button position and diameter.
		translate(button_center)
			circle(d = button_plunger_d + 2 * button_slot_gap, $fn = 48);
	}
}

module button_pusher_feature() {
	pusher_drop = max(0, lid_inner_height - (button_body_size[2] + button_press_clearance));

	if (pusher_drop > 0) {
		translate([button_center[0], button_center[1], lid_inner_height - pusher_drop])
			cylinder(h = pusher_drop + eps, d = button_pusher_d, $fn = 32);
	}
}

module usb_c_side_cutout() {
	cutout_height_z = usb_cutout_span_z[1] - usb_cutout_span_z[0];

	// Assumption: USB-C is accessed on the +X side wall.
	translate([
		case_max_x - wall_thickness - eps,
		usb_center_xy[1] - usb_cutout_size_y / 2,
		usb_cutout_span_z[0]
	])
		cube([
			wall_thickness + 2 * eps,
			usb_cutout_size_y,
			cutout_height_z
		]);
}

module imu_axis_engraving_2d() {
	// Place engraving near upper-left area of the lid top.
	origin = [case_min_x + wall_thickness + 7, case_max_y - wall_thickness - 13];

	translate(origin) {
		// X axis arrow
		arrow_2d(engrave_arrow_len, engrave_line_width, engrave_head_len, engrave_head_width);

		// Y axis arrow
		rotate(90)
			arrow_2d(engrave_arrow_len, engrave_line_width, engrave_head_len, engrave_head_width);

		// Z axis symbol (out-of-plane): circle with center dot.
		difference() {
			circle(d = 4, $fn = 40);
			circle(d = 2.2, $fn = 40);
		}
		circle(d = 0.9, $fn = 20);
	}
}

// -------------------------------
// Base geometry
// -------------------------------
module enclosure_base() {
	union() {
		difference() {
			union() {
				// Base shell below split plane z=0.
				translate([0, 0, -base_total_height])
					linear_extrude(height = base_total_height)
						translate(case_center)
							rounded_rect_2d(case_outer_size, corner_radius);

				base_half_loops();
			}

			// Internal base cavity under the board.
			translate([0, 0, -base_inner_depth])
				linear_extrude(height = base_inner_depth + eps)
					translate(case_center)
						rounded_rect_2d(case_inner_size, max(0.5, corner_radius - 0.7));
		}

		// Add board support posts as separate solids.
		difference() {
			board_corner_posts();
			board_corner_post_holes();
		}
	}
}

// -------------------------------
// Lid geometry
// -------------------------------
module enclosure_lid() {
	difference() {
		union() {
			// Lid shell above split plane z=0.
			linear_extrude(height = lid_total_height)
				translate(case_center)
					rounded_rect_2d(case_outer_size, corner_radius);

			lid_half_loops();

			// Internal pusher to improve button actuation through flexure tab.
			button_pusher_feature();
		}

		// Main internal cavity for components.
		linear_extrude(height = lid_inner_height + eps)
			translate(case_center)
				rounded_rect_2d(case_inner_size, max(0.5, corner_radius - 0.7));

		// Through holes in lid aligned to board mounting holes.
		for (hole = board_mount_holes) {
			translate([hole[0], hole[1], -eps])
				cylinder(h = lid_total_height + 2 * eps, d = screw_clearance_d, $fn = 40);
		}

		// Button flexure U-slot cut through roof.
		translate([0, 0, lid_inner_height - eps])
			linear_extrude(height = lid_roof_thickness + 2 * eps)
				button_slot_2d();

		// LED window on lid.
		translate([led_center[0], led_center[1], lid_inner_height - eps])
			cylinder(h = lid_roof_thickness + 2 * eps, d = led_body_d + led_window_clearance, $fn = 48);

		// Switch rectangular cutout on lid.
		translate([
			switch_center[0] - switch_cutout_size[0] / 2,
			switch_center[1] - switch_cutout_size[1] / 2,
			lid_inner_height - eps
		])
			cube([switch_cutout_size[0], switch_cutout_size[1], lid_roof_thickness + 2 * eps]);

		// USB-C side cutout.
		usb_c_side_cutout();

		// Axis engraving on top surface.
		translate([0, 0, lid_total_height - engrave_depth])
			linear_extrude(height = engrave_depth + eps)
				imu_axis_engraving_2d();
	}
}

// -------------------------------
// Render selection
// -------------------------------
if (render_mode == "base") {
	enclosure_base();
} else if (render_mode == "lid") {
	enclosure_lid();
} else if (render_mode == "exploded") {
	color("LightGray") enclosure_base();
	color("Gainsboro") translate([0, 0, exploded_lid_offset_z]) enclosure_lid();
} else {
	color("LightGray") enclosure_base();
	color("Gainsboro") enclosure_lid();
}
