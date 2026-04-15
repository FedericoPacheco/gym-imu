# Instructions

Generate an enclosure for a wearable IMU device that has already been assembled.

Refer to the schematics at: docs/diagrams/wiring/xiao-esp32-c3/schematic.puml

Considering the coordinate system:

* X axis: along the protoboard's long side, in mm
* Y axis: along the protoboard's short side, in mm
* Z axis: height, positive on the side with the components, negative on the side with the copper traces, in mm.
* Origin: board's bottom left corner, z = 0 on the component's side surface.

The prototype has the following features:

* Protoboard size: 50 mm × 70 mm × 1 mm, with 2 mm diameter mounting holes at each corner. Each hole's center is located at 2.5 mm from each adjacent board edge.
* The side with the copper traces requires a clearance of at least 7 mm between the board and the case for wiring.
* All electronic components sit on the opposite side of the board (positive z) and require a clearance of at least 13 mm between the board and the case.
* Components center with respect to the origin:
  * Button: x = 13 mm, y = 12 mm
  * LED: x = 25 mm, y = 12 mm
  * Switch: x = 39 mm, y = 14 mm
  * IMU, microcontroller, and battery: not relevant
* Component dimensions (width × length × height):
  * Button: base 12 mm × 12 mm x 12 mm (unpressed). The pressed top is circular with 12 mm diameter.
  * LED: 5 mm diameter
  * Switch: 15 mm (x axis, short side) × 21 mm (y axis, long side)
  * IMU, microcontroller, and battery: not relevant

Requirements for the enclosure design:

* The enclosure should be a two-part design, base and lid, held together with machine screws that span the full height of the case and allow disassembly for maintenance. The split base/split plane will be at z = 0 mm. Draw them separately for easier review. Flip the lid upside down (outer top face down, inner cavity up) to achieve better printability.
* Add an interlocking alignment border between base and lid at the split plane (z = 0) using a tongue-and-groove joint around the full perimeter (except where interrupted by required external cutouts). Put the tongue on the base and the groove on the lid. Use a tongue height of 2 mm and tongue thickness of 1 mm. The intention is for the joint to self-align during assembly and prevent lateral shifting in xy plane before screws are tightened.
* The board will be secured to the base using two large and internal rectangular prism supports that run along the y axis, one per side, rather than isolated corner posts. These will have holes that will pass through the whole base, being visible from the outside. The screws that hold the lid will also go through the board and into the same holes, so the mounting holes must be precisely aligned. The lid will not have prisms nor posts.
* The button will remain inside. The lid should include a U-shaped slot that is parallel to the y axis and opens to the top (increasing y values). The bending side (bottom of the U) must be clearly rounded and follow the button's circular pressed surface, centered at the button center. The intent is for the material to be bent to actuate the button indirectly. Leave a space of 1 mm between the slot and the rest of the lid. Leave a minimal clearance of at most 1-2 mm to ensure the button can be pressed without excessive force.
* Provide a circular hole on the lid for the LED so it is visible from the outside.
* Provide a rectangular cutout on the lid for the switch so it can be toggled from the outside. The cutout must have its long side parallel to the y axis and short side parallel to the x axis.
* Include a cutout for the USB-C port centered at x = 34 mm, y = 50 mm, spanning z from 4 mm to 7 mm. Use a rounded-rectangle cutout (not a sharp-corner rectangle). USB Type-C receptacle nominal opening is 8.34 mm × 2.56 mm (USB-IF); for enclosure tolerance, use an approximate cutout around 8.5 mm × 3 mm with about 1 mm corner radius.
* Wall thickness should be of at least 3 mm to provide basic rigidity and resist small impacts.
* Round external corners and edges with a radius of around 2 mm.
* Add two detachable handles/loops that are printed as separate parts. Group both handles to the left side of the base along the x axis for easy review. Each handle should keep 5 mm strap clearance, be at least 5 mm thick, keep rounded corners on edges that do not touch the case, and keep sharp corners on the side that touches the case. Place the handle screw-pass holes through solid handle material from the sides, not through the central strap-clearance opening; achieve this by sizing/positioning the handle geometry, not by adding local bridge/cube blocks.
* The prisms that host the mounting holes for the board/lid inserts will also serve as the mounting points for the detachable handles, by adding another 4 pass-through holes (2 per side) running along the x axis for additional screws. Keeping a margin a safety, ensure the new screws do not clash with the existing base/board/lid fasteners. The holes of the handles will align with those of the prisms, so that the screws can go through them and into the base of the case.
* Engrave a 3-axis drawing near the upper left region of the lid (low x values, high y values) aligned with the IMU orientation: xy plane parallel to the board, z axis pointing up, x arrow pointing left (decreasing x), and y arrow pointing down (decreasing y). Use a 1 mm engraving depth and 10 mm arrow length, and make sure the drawing is thick enough so that it can be printed clearly with the lid is facing down. Include the "x", "y", and "z" labels near the corresponding axis. Represent the z axis with a simple circular hole (single round cutout), not a ring-with-dot symbol. Keep the full engraving (lines, arrowheads, and labels) inset from case edges so it does not touch or visually crowd any outer edge.
* The base/board/lid and handles will use M2 machine screws with hexagonal nuts on the opposite side, as well as washers on both sides, and epoxi glue to secure the nuts in case it's needed.
* Add a separate and toggable "ASSEMBLED" view mode where the lid is on top of the base and the handles are attached to the base to make sure everything fits together as expected.
* When the pieces are not assembled, make sure to leave a sufficient gap between them to prevent collisions and for easier review.

Tools and libraries available:

* OpenSCAD. Docs:
  * Overview: <https://openscad.org/documentation.html>

* BOSL2 library. Docs:
  * Overview: <https://github.com/BelfrySCAD/BOSL2/wiki>
  * Table of contents: <https://github.com/BelfrySCAD/BOSL2/wiki/TOC>

* NopSCADlib library. Docs:
  * Overview: <https://github.com/nophead/NopSCADlib/blob/master/readme.md>
  * Usage: <https://github.com/nophead/NopSCADlib/blob/master/docs/usage.md>

Coding standards for the OpenSCAD code:

* Use descriptive names.
* Avoid magic numbers. Use constants for all dimensions and provide default values. Write them in all caps and with underscores.
* Add comments to explain the code.
* Use OpenSCAD libraries (BOSL2, NopSCADlib) where appropriate.

If you need additional information, feel free to ask or explore the rest of the codebase.
If you identify potential issues, stop early and point them out before proceeding with the design.

Output:

Save the generated OpenSCAD code to: docs/diagrams/wiring/xiao-esp32-c3/enclosure/enclosure.scad and iterate by rendering and adjusting the design until it meets the requirements.
