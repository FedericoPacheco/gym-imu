$fn = 72;

// -----------------------------
// Display controls
// -----------------------------
VIEW_MODE = "ASSEMBLED"; // ASSEMBLED, EXPLODED
EXPLODED_GAP = 25;

// -----------------------------
// Board and coordinate system
// Origin is board bottom-left at component side surface (z = 0)
// -----------------------------
BOARD_SIZE_X = 70;
BOARD_SIZE_Y = 50;
BOARD_THICKNESS = 1;
BOARD_HOLE_DIAMETER = 2;
BOARD_HOLE_INSET = 2.5;
BOARD_FIT_CLEARANCE = 0.4;

// -----------------------------
// Enclosure envelope
// -----------------------------
TRACE_SIDE_CLEARANCE = 7;
COMPONENT_SIDE_CLEARANCE = 14;
WALL_THICKNESS = 3;
BASE_FLOOR_THICKNESS = 3;
LID_ROOF_THICKNESS = 3;

EXTERNAL_CORNER_RADIUS = 2;
INNER_CORNER_RADIUS = 1;

// -----------------------------
// Tongue-and-groove joint
// -----------------------------
TONGUE_THICKNESS = 1;
TONGUE_HEIGHT = 2;
JOINT_CLEARANCE = 0.2;

// -----------------------------
// Fasteners and posts
// -----------------------------
M2_CLEARANCE_DIAMETER = 2.4;
HEAT_SET_INSERT_HOLE_DIAMETER = 3.2;
POST_SIZE = 7;

// -----------------------------
// Lid feature positions (board coordinates)
// -----------------------------
BUTTON_CENTER_X = 13;
BUTTON_CENTER_Y = 12;
BUTTON_BODY_SIZE = 12;
BUTTON_PRESS_DIAMETER = 12;
BUTTON_SLOT_GAP = 1;
BUTTON_SLOT_ARM_LENGTH = 12.5;
BUTTON_SLOT_BOTTOM_DIAMETER = BUTTON_PRESS_DIAMETER;

LED_CENTER_X = 25;
LED_CENTER_Y = 12;
LED_DIAMETER = 5;
LED_HOLE_DIAMETER = 5.6;

SWITCH_CENTER_X = 39;
SWITCH_CENTER_Y = 14;
// Long side must be parallel to the y axis.
SWITCH_SIZE_X = 15;
SWITCH_SIZE_Y = 21;
SWITCH_HOLE_CLEARANCE = 0.6;

USB_CENTER_X = 34;
USB_CENTER_Y = 50;
USB_WIDTH_X = 8.5;
USB_Z_MIN = 4;
USB_Z_MAX = 7;
USB_CUTOUT_DEPTH_Y = 12;
USB_HOLE_CLEARANCE_X = 0.6;
USB_CORNER_RADIUS = 1;

// -----------------------------
// Strap loop handles
// -----------------------------
HANDLE_THICKNESS_Z = 9;
HANDLE_FRAME_THICKNESS = 5;
HANDLE_STRAP_CLEARANCE = 5;
HANDLE_END_MARGIN = 10;
HANDLE_CORNER_RADIUS = 2;
HANDLE_PREVIEW_GAP = 8;
HANDLE_SHORT_SIDE_EXTRA_THICKNESS = 1.5;

SIDE_SUPPORT_X_SIZE = POST_SIZE - 1;
HANDLE_INSERT_Z_OFFSET_FROM_BASE = 5.5;
// Keep pass-through holes centered in the short-side wall thickness (derived below).
HANDLE_INSERT_TO_BOARD_INSERT_MARGIN = 4;

HANDLE_INSERT_DIAMETER = HEAT_SET_INSERT_HOLE_DIAMETER;
HANDLE_SCREW_PASS_DIAMETER = M2_CLEARANCE_DIAMETER;
HANDLE_SCREW_TO_SLOT_MARGIN_X = 0.8;
HANDLE_SCREW_TO_OUTER_EDGE_MARGIN_X = 1.2;
HANDLE_SCREW_TO_SLOT_MARGIN_Y = 0.6;
HANDLE_SCREW_TO_END_MARGIN_Y = 1.0;

// -----------------------------
// Lid engraving
// -----------------------------
AXIS_ENGRAVE_DEPTH = 1.5;
AXIS_ARROW_LENGTH = 10;
AXIS_LINE_WIDTH = 1;
AXIS_LABEL_SIZE = 5;
AXIS_FONT = "Liberation Sans:style=Bold";
AXIS_ORIGIN_X = 25;
AXIS_ORIGIN_Y = 45;
AXIS_Z_HOLE_DIAMETER = 3.6;

// -----------------------------
// Generic modeling tolerances
// -----------------------------
EPS = 0.1;

// -----------------------------
// Derived geometry
// -----------------------------
INNER_MIN_X = -BOARD_FIT_CLEARANCE;
INNER_MAX_X = BOARD_SIZE_X + BOARD_FIT_CLEARANCE;
INNER_MIN_Y = -BOARD_FIT_CLEARANCE;
INNER_MAX_Y = BOARD_SIZE_Y + BOARD_FIT_CLEARANCE;

INNER_SIZE_X = INNER_MAX_X - INNER_MIN_X;
INNER_SIZE_Y = INNER_MAX_Y - INNER_MIN_Y;

OUTER_MIN_X = INNER_MIN_X - WALL_THICKNESS;
OUTER_MAX_X = INNER_MAX_X + WALL_THICKNESS;
OUTER_MIN_Y = INNER_MIN_Y - WALL_THICKNESS;
OUTER_MAX_Y = INNER_MAX_Y + WALL_THICKNESS;

OUTER_SIZE_X = OUTER_MAX_X - OUTER_MIN_X;
OUTER_SIZE_Y = OUTER_MAX_Y - OUTER_MIN_Y;

CASE_CENTER_X = (OUTER_MIN_X + OUTER_MAX_X) / 2;
CASE_CENTER_Y = (OUTER_MIN_Y + OUTER_MAX_Y) / 2;

BASE_CAVITY_DEPTH = TRACE_SIDE_CLEARANCE + BOARD_THICKNESS;
BASE_BOTTOM_Z = -BASE_CAVITY_DEPTH - BASE_FLOOR_THICKNESS;
POST_TOP_Z = -BOARD_THICKNESS;

LID_TOP_Z = COMPONENT_SIDE_CLEARANCE + LID_ROOF_THICKNESS;

TONGUE_OUTER_SIZE_X = INNER_SIZE_X + (2 * TONGUE_THICKNESS);
TONGUE_OUTER_SIZE_Y = INNER_SIZE_Y + (2 * TONGUE_THICKNESS);
TONGUE_OUTER_RADIUS = INNER_CORNER_RADIUS + TONGUE_THICKNESS;

GROOVE_OUTER_SIZE_X = INNER_SIZE_X + (2 * (TONGUE_THICKNESS + JOINT_CLEARANCE));
GROOVE_OUTER_SIZE_Y = INNER_SIZE_Y + (2 * (TONGUE_THICKNESS + JOINT_CLEARANCE));
GROOVE_INNER_SIZE_X = INNER_SIZE_X - (2 * JOINT_CLEARANCE);
GROOVE_INNER_SIZE_Y = INNER_SIZE_Y - (2 * JOINT_CLEARANCE);
GROOVE_OUTER_RADIUS = TONGUE_OUTER_RADIUS + JOINT_CLEARANCE;
GROOVE_INNER_RADIUS = max(0.5, INNER_CORNER_RADIUS - JOINT_CLEARANCE);

HANDLE_SPAN_Y = OUTER_SIZE_Y - (2 * HANDLE_END_MARGIN);
HANDLE_OUTER_DEPTH_X = HANDLE_STRAP_CLEARANCE + (2 * HANDLE_FRAME_THICKNESS);
HANDLE_INNER_SPAN_Y = HANDLE_SPAN_Y - (2 * HANDLE_FRAME_THICKNESS);
HANDLE_INNER_SPAN_Y_EFFECTIVE = HANDLE_INNER_SPAN_Y - (2 * HANDLE_SHORT_SIDE_EXTRA_THICKNESS);
HANDLE_SHORT_SIDE_WALL_THICKNESS = (HANDLE_SPAN_Y - HANDLE_INNER_SPAN_Y_EFFECTIVE) / 2;
HANDLE_SCREW_RADIUS = HANDLE_SCREW_PASS_DIAMETER / 2;

assert(HANDLE_INNER_SPAN_Y_EFFECTIVE > 0, "Handle short-side extra thickness is too large");

HANDLE_INSERT_Z = BASE_BOTTOM_Z + HANDLE_INSERT_Z_OFFSET_FROM_BASE;
HANDLE_ATTACHED_Z = HANDLE_INSERT_Z - (HANDLE_THICKNESS_Z / 2);
HANDLE_INSERT_Y_OFFSET = (HANDLE_SPAN_Y / 2) - (HANDLE_SHORT_SIDE_WALL_THICKNESS / 2);
SIDE_SUPPORT_Y_START = BOARD_HOLE_INSET - (POST_SIZE / 2);
SIDE_SUPPORT_Y_SIZE = BOARD_SIZE_Y - (2 * BOARD_HOLE_INSET) + POST_SIZE;
SIDE_SUPPORT_Z_START = BASE_BOTTOM_Z;
SIDE_SUPPORT_Z_SIZE = POST_TOP_Z - BASE_BOTTOM_Z;
HANDLE_SCREW_X_OFFSET =
	(HANDLE_STRAP_CLEARANCE / 2) + (HANDLE_SCREW_PASS_DIAMETER / 2) + HANDLE_SCREW_TO_SLOT_MARGIN_X;

assert(
	((HANDLE_OUTER_DEPTH_X / 2) - HANDLE_SCREW_X_OFFSET - (HANDLE_SCREW_PASS_DIAMETER / 2)) >= HANDLE_SCREW_TO_OUTER_EDGE_MARGIN_X,
	"Handle screw holes are too close to outer edge"
);
assert(
	(HANDLE_INSERT_Y_OFFSET - HANDLE_SCREW_RADIUS) >= ((HANDLE_INNER_SPAN_Y_EFFECTIVE / 2) + HANDLE_SCREW_TO_SLOT_MARGIN_Y),
	"Handle screw holes overlap handle opening"
);
assert(
	(HANDLE_INSERT_Y_OFFSET + HANDLE_SCREW_RADIUS) <= ((HANDLE_SPAN_Y / 2) - HANDLE_SCREW_TO_END_MARGIN_Y),
	"Handle screw holes are too close to handle ends"
);

BOARD_INSERT_Y_MIN = BOARD_HOLE_INSET;
BOARD_INSERT_Y_MAX = BOARD_SIZE_Y - BOARD_HOLE_INSET;
MIN_CENTER_SPACING_HANDLE_TO_BOARD_INSERT =
	(HANDLE_INSERT_DIAMETER / 2) + (HEAT_SET_INSERT_HOLE_DIAMETER / 2) + HANDLE_INSERT_TO_BOARD_INSERT_MARGIN;
HANDLE_LOWER_INSERT_Y = CASE_CENTER_Y - HANDLE_INSERT_Y_OFFSET;
HANDLE_UPPER_INSERT_Y = CASE_CENTER_Y + HANDLE_INSERT_Y_OFFSET;

assert(
	(HANDLE_LOWER_INSERT_Y - BOARD_INSERT_Y_MIN) >= MIN_CENTER_SPACING_HANDLE_TO_BOARD_INSERT,
	"Handle insert holes are too close to lower board/lid insert holes"
);
assert(
	(BOARD_INSERT_Y_MAX - HANDLE_UPPER_INSERT_Y) >= MIN_CENTER_SPACING_HANDLE_TO_BOARD_INSERT,
	"Handle insert holes are too close to upper board/lid insert holes"
);

SWITCH_HOLE_SIZE_X = SWITCH_SIZE_X + SWITCH_HOLE_CLEARANCE;
SWITCH_HOLE_SIZE_Y = SWITCH_SIZE_Y + SWITCH_HOLE_CLEARANCE;
USB_HOLE_SIZE_X = USB_WIDTH_X + USB_HOLE_CLEARANCE_X;
USB_HOLE_SIZE_Z = USB_Z_MAX - USB_Z_MIN;

MOUNT_POINTS = [
	[BOARD_HOLE_INSET, BOARD_HOLE_INSET],
	[BOARD_SIZE_X - BOARD_HOLE_INSET, BOARD_HOLE_INSET],
	[BOARD_HOLE_INSET, BOARD_SIZE_Y - BOARD_HOLE_INSET],
	[BOARD_SIZE_X - BOARD_HOLE_INSET, BOARD_SIZE_Y - BOARD_HOLE_INSET]
];

// -----------------------------
// Utility modules
// -----------------------------
module rounded_rectangle_2d(size_xy, radius) {
	safe_radius = max(0, min(radius, (min(size_xy) / 2) - 0.01));
	if (safe_radius > 0) {
		offset(r = safe_radius)
			offset(delta = -safe_radius)
				square(size_xy, center = true);
	} else {
		square(size_xy, center = true);
	}
}

module rounded_prism(size_xy, z_min, z_max, radius) {
	translate([CASE_CENTER_X, CASE_CENTER_Y, z_min])
		linear_extrude(height = z_max - z_min)
			rounded_rectangle_2d(size_xy, radius);
}

module cut_box(min_x, min_y, min_z, size_x, size_y, size_z) {
	translate([min_x, min_y, min_z])
		cube([size_x, size_y, size_z], center = false);
}

module line_slot_2d(p_start, p_end, width) {
	hull() {
		translate(p_start) circle(d = width);
		translate(p_end) circle(d = width);
	}
}

module mount_holes(diameter, z_min, z_max) {
	for (point = MOUNT_POINTS) {
		translate([point[0], point[1], z_min])
			cylinder(d = diameter, h = z_max - z_min, center = false);
	}
}

module mounting_posts() {
	for (point = MOUNT_POINTS) {
		translate([point[0] - (POST_SIZE / 2), point[1] - (POST_SIZE / 2), BASE_BOTTOM_Z])
			cube([POST_SIZE, POST_SIZE, POST_TOP_Z - BASE_BOTTOM_Z], center = false);
	}
}

module tongue_ring() {
	translate([CASE_CENTER_X, CASE_CENTER_Y, 0])
		linear_extrude(height = TONGUE_HEIGHT)
			difference() {
				rounded_rectangle_2d([TONGUE_OUTER_SIZE_X, TONGUE_OUTER_SIZE_Y], TONGUE_OUTER_RADIUS);
				rounded_rectangle_2d([INNER_SIZE_X, INNER_SIZE_Y], INNER_CORNER_RADIUS);
			}
}

module groove_volume() {
	translate([CASE_CENTER_X, CASE_CENTER_Y, -EPS])
		linear_extrude(height = TONGUE_HEIGHT + (2 * EPS))
			difference() {
				rounded_rectangle_2d([GROOVE_OUTER_SIZE_X, GROOVE_OUTER_SIZE_Y], GROOVE_OUTER_RADIUS);
				rounded_rectangle_2d([GROOVE_INNER_SIZE_X, GROOVE_INNER_SIZE_Y], GROOVE_INNER_RADIUS);
			}
}

// Keep the case-contact side sharp while preserving rounded free-side corners.
// sharp_side accepts "POS_X" or "NEG_X" in the local handle profile.
module handle_profile_2d(size_xy, radius, sharp_side) {
	safe_radius = max(0, min(radius, (min(size_xy) / 2) - 0.01));

	union() {
		rounded_rectangle_2d(size_xy, safe_radius);

		if (safe_radius > 0 && sharp_side == "POS_X") {
			translate([size_xy[0] / 2 - safe_radius, -size_xy[1] / 2])
				square([safe_radius, size_xy[1]], center = false);
		} else if (safe_radius > 0 && sharp_side == "NEG_X") {
			translate([-size_xy[0] / 2, -size_xy[1] / 2])
				square([safe_radius, size_xy[1]], center = false);
		}
	}
}

module detached_handle_profile_2d(case_contact_side) {
	difference() {
		handle_profile_2d([HANDLE_OUTER_DEPTH_X, HANDLE_SPAN_Y], HANDLE_CORNER_RADIUS, case_contact_side);
		rounded_rectangle_2d([HANDLE_STRAP_CLEARANCE, HANDLE_INNER_SPAN_Y_EFFECTIVE], HANDLE_CORNER_RADIUS / 2);
	}
}

module detached_single_handle(case_contact_side) {
	screw_x = (case_contact_side == "POS_X") ? HANDLE_SCREW_X_OFFSET : -HANDLE_SCREW_X_OFFSET;
	screw_cut_length = HANDLE_OUTER_DEPTH_X + (2 * abs(screw_x)) + (2 * EPS);

	difference() {
		linear_extrude(height = HANDLE_THICKNESS_Z)
			detached_handle_profile_2d(case_contact_side);

		for (y_offset = [-HANDLE_INSERT_Y_OFFSET, HANDLE_INSERT_Y_OFFSET]) {
			translate([screw_x, y_offset, HANDLE_THICKNESS_Z / 2])
				rotate([0, 90, 0])
					cylinder(d = HANDLE_SCREW_PASS_DIAMETER, h = screw_cut_length, center = true);
		}
	}
}

module base_handle_support_extensions() {
	translate([INNER_MIN_X, SIDE_SUPPORT_Y_START, SIDE_SUPPORT_Z_START])
		cube([SIDE_SUPPORT_X_SIZE, SIDE_SUPPORT_Y_SIZE, SIDE_SUPPORT_Z_SIZE], center = false);
	translate([INNER_MAX_X - SIDE_SUPPORT_X_SIZE, SIDE_SUPPORT_Y_START, SIDE_SUPPORT_Z_START])
		cube([SIDE_SUPPORT_X_SIZE, SIDE_SUPPORT_Y_SIZE, SIDE_SUPPORT_Z_SIZE], center = false);
}

module base_handle_insert_holes() {
	hole_length = SIDE_SUPPORT_X_SIZE + WALL_THICKNESS + (2 * EPS);

	for (y_offset = [-HANDLE_INSERT_Y_OFFSET, HANDLE_INSERT_Y_OFFSET]) {
		y_pos = CASE_CENTER_Y + y_offset;

		translate([OUTER_MIN_X - EPS, y_pos, HANDLE_INSERT_Z])
			rotate([0, 90, 0])
				cylinder(d = HANDLE_INSERT_DIAMETER, h = hole_length, center = false);

		translate([OUTER_MAX_X + EPS, y_pos, HANDLE_INSERT_Z])
			rotate([0, -90, 0])
				cylinder(d = HANDLE_INSERT_DIAMETER, h = hole_length, center = false);
	}
}

// -----------------------------
// Lid cutouts and engravings
// -----------------------------
module button_u_slot_2d() {
	bend_radius = BUTTON_SLOT_BOTTOM_DIAMETER / 2;
	bend_outer_radius = bend_radius + (BUTTON_SLOT_GAP / 2);
	bend_inner_radius = max(0.01, bend_radius - (BUTTON_SLOT_GAP / 2));
	bend_center = [BUTTON_CENTER_X, BUTTON_CENTER_Y];
	left_bottom = [bend_center[0] - bend_radius, bend_center[1]];
	right_bottom = [bend_center[0] + bend_radius, bend_center[1]];
	left_top = [left_bottom[0], bend_center[1] + BUTTON_SLOT_ARM_LENGTH];
	right_top = [right_bottom[0], bend_center[1] + BUTTON_SLOT_ARM_LENGTH];

	union() {
		line_slot_2d(left_bottom, left_top, BUTTON_SLOT_GAP);
		line_slot_2d(right_bottom, right_top, BUTTON_SLOT_GAP);

		translate(bend_center)
			intersection() {
				difference() {
					circle(r = bend_outer_radius);
					circle(r = bend_inner_radius);
				}
				translate([-bend_outer_radius, -bend_outer_radius])
					square([2 * bend_outer_radius, bend_outer_radius], center = false);
			}
	}
}

module button_slot_cutout() {
	translate([0, 0, COMPONENT_SIDE_CLEARANCE - EPS])
		linear_extrude(height = LID_ROOF_THICKNESS + (2 * EPS))
			button_u_slot_2d();
}

module led_cutout() {
	translate([LED_CENTER_X, LED_CENTER_Y, COMPONENT_SIDE_CLEARANCE - EPS])
		cylinder(d = LED_HOLE_DIAMETER, h = LID_ROOF_THICKNESS + (2 * EPS), center = false);
}

module switch_cutout() {
	cut_box(
		SWITCH_CENTER_X - (SWITCH_HOLE_SIZE_X / 2),
		SWITCH_CENTER_Y - (SWITCH_HOLE_SIZE_Y / 2),
		COMPONENT_SIDE_CLEARANCE - EPS,
		SWITCH_HOLE_SIZE_X,
		SWITCH_HOLE_SIZE_Y,
		LID_ROOF_THICKNESS + (2 * EPS)
	);
}

module usb_c_cutout() {
	translate([
		USB_CENTER_X,
		USB_CENTER_Y - (USB_CUTOUT_DEPTH_Y / 2) - EPS,
		USB_Z_MIN + (USB_HOLE_SIZE_Z / 2)
	])
		rotate([-90, 0, 0])
			linear_extrude(height = USB_CUTOUT_DEPTH_Y + (2 * EPS))
				rounded_rectangle_2d([USB_HOLE_SIZE_X, USB_HOLE_SIZE_Z], USB_CORNER_RADIUS);
}

module axis_engraving_2d() {
	x_end = [AXIS_ORIGIN_X - AXIS_ARROW_LENGTH, AXIS_ORIGIN_Y];
	y_end = [AXIS_ORIGIN_X, AXIS_ORIGIN_Y - AXIS_ARROW_LENGTH];

	union() {
		line_slot_2d([AXIS_ORIGIN_X, AXIS_ORIGIN_Y], x_end, AXIS_LINE_WIDTH);

		polygon(points = [
			[x_end[0] - 1, x_end[1]],
			[x_end[0] + 1.8, x_end[1] + 1.8],
			[x_end[0] + 1.8, x_end[1] - 1.8]
		]);

		line_slot_2d([AXIS_ORIGIN_X, AXIS_ORIGIN_Y], y_end, AXIS_LINE_WIDTH);

		polygon(points = [
			[y_end[0], y_end[1] - 1],
			[y_end[0] - 1.8, y_end[1] + 1.8],
			[y_end[0] + 1.8, y_end[1] + 1.8]
		]);

		translate([AXIS_ORIGIN_X, AXIS_ORIGIN_Y])
			circle(d = AXIS_Z_HOLE_DIAMETER);

		translate([x_end[0] - 1.2, x_end[1] - 0.3])
			text("x", size = AXIS_LABEL_SIZE, font = AXIS_FONT, halign = "right", valign = "center");
		translate([y_end[0] - 0.2, y_end[1] - 1.4])
			text("y", size = AXIS_LABEL_SIZE, font = AXIS_FONT, halign = "center", valign = "top");
		translate([AXIS_ORIGIN_X + 2.8, AXIS_ORIGIN_Y + 2.8])
			text("z", size = AXIS_LABEL_SIZE, font = AXIS_FONT, halign = "left", valign = "center");
	}
}

module axis_engraving_cutout() {
	translate([0, 0, LID_TOP_Z - AXIS_ENGRAVE_DEPTH])
		linear_extrude(height = AXIS_ENGRAVE_DEPTH + EPS)
			axis_engraving_2d();
}

// -----------------------------
// Main parts
// -----------------------------
module base_part() {
	difference() {
		union() {
			difference() {
				rounded_prism([OUTER_SIZE_X, OUTER_SIZE_Y], BASE_BOTTOM_Z, 0, EXTERNAL_CORNER_RADIUS);
				cut_box(
					INNER_MIN_X,
					INNER_MIN_Y,
					-BASE_CAVITY_DEPTH,
					INNER_SIZE_X,
					INNER_SIZE_Y,
					BASE_CAVITY_DEPTH + EPS
				);
			}
			tongue_ring();
			base_handle_support_extensions();
		}

		mount_holes(HEAT_SET_INSERT_HOLE_DIAMETER, BASE_BOTTOM_Z - EPS, TONGUE_HEIGHT + EPS);
		base_handle_insert_holes();
	}
}

module lid_part() {
	difference() {
		difference() {
			rounded_prism([OUTER_SIZE_X, OUTER_SIZE_Y], 0, LID_TOP_Z, EXTERNAL_CORNER_RADIUS);
			cut_box(
				INNER_MIN_X,
				INNER_MIN_Y,
				-EPS,
				INNER_SIZE_X,
				INNER_SIZE_Y,
				COMPONENT_SIDE_CLEARANCE + EPS
			);
			groove_volume();
		}

		mount_holes(M2_CLEARANCE_DIAMETER, -EPS, LID_TOP_Z + EPS);
		button_slot_cutout();
		led_cutout();
		switch_cutout();
		usb_c_cutout();
		axis_engraving_cutout();
	}
}

module lid_for_preview() {
	translate([0, 2 * CASE_CENTER_Y, LID_TOP_Z])
		rotate([180, 0, 0])
			lid_part();
}

module board_reference() {
	color([0.1, 0.5, 0.1, 0.25])
		translate([0, 0, -BOARD_THICKNESS])
			cube([BOARD_SIZE_X, BOARD_SIZE_Y, BOARD_THICKNESS], center = false);

	color([0.1, 0.5, 0.1, 0.35])
		for (point = MOUNT_POINTS) {
			translate([point[0], point[1], -BOARD_THICKNESS - EPS])
				cylinder(d = BOARD_HOLE_DIAMETER, h = BOARD_THICKNESS + (2 * EPS), center = false);
		}
}

module assembled_handles() {
	translate([OUTER_MIN_X - (HANDLE_OUTER_DEPTH_X / 2), CASE_CENTER_Y, HANDLE_ATTACHED_Z])
		color("silver") detached_single_handle("POS_X");
	translate([OUTER_MAX_X + (HANDLE_OUTER_DEPTH_X / 2), CASE_CENTER_Y, HANDLE_ATTACHED_Z])
		color("silver") detached_single_handle("NEG_X");
}

module scene() {
	if (VIEW_MODE == "ASSEMBLED") {
		color("gainsboro") lid_part();
		color("lightgray") base_part();
		assembled_handles();
		board_reference();
	} else {
		base_preview_x = -(OUTER_SIZE_X + EXPLODED_GAP) / 2;
		lid_preview_x = (OUTER_SIZE_X + EXPLODED_GAP) / 2;
		handles_left_x_1 = base_preview_x - (OUTER_SIZE_X / 2) - HANDLE_PREVIEW_GAP - (HANDLE_OUTER_DEPTH_X / 2);
		handles_left_x_2 = handles_left_x_1 - HANDLE_OUTER_DEPTH_X - HANDLE_PREVIEW_GAP;

		translate([base_preview_x, 0, 0])
			color("lightgray") base_part();
		translate([lid_preview_x, 0, 0])
			color("gainsboro") lid_for_preview();

		translate([handles_left_x_1, CASE_CENTER_Y, 0])
			color("silver") detached_single_handle("POS_X");
		translate([handles_left_x_2, CASE_CENTER_Y, 0])
			color("silver") detached_single_handle("NEG_X");
	}
}

scene();
