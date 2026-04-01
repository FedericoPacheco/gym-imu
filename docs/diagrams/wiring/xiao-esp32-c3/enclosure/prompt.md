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
* All electronic components sit on the opposite side of the board (positive z) and require a clearance of at least 14 mm between the board and the case.
* Components center with respect to the origin:
  * Button: x = 13 mm, y = 12 mm
  * LED: x = 25 mm, y = 12 mm
  * Switch: x = 39 mm, y = 14 mm
  * IMU, microcontroller, and battery: not relevant
* Component dimensions (width × length × height):
  * Button: 12 mm × 12 mm x 12 mm (unpressed), 1mm diameter
  * LED: 5 mm diameter
  * Switch: 15 mm (x axis, short side) × 21 mm (y axis, long side)
  * IMU, microcontroller, and battery: not relevant

Requirements for the enclosure design:

* The enclosure should be a two-part design, base and lid, held together with M2 machine screws that span the full height of the case and allow disassembly for maintenance. The split base/split plane will be at z = 0 mm. Draw them separately for easier review.
* Add an interlocking alignment border between base and lid at the split plane (z = 0) using a tongue-and-groove joint around the full perimeter (except where interrupted by required external cutouts). Put the tongue on the base and the groove on the lid. Use a tongue height of 2 mm and tongue thickness of 1 mm. The intention is for the joint to self-align during assembly and prevent lateral shifting in x/y before screws are tightened.
* The board will be secured to the base with supporting square posts with holes for heat-set inserts. The holes will pass through the whole base, being visible from the outside. The screws that hold the lid will also go through the board and into the same inserts, so the mounting holes must be precisely aligned. The lid will not have posts. The posts may be integrated with the corner walls of the base.
* The button will remain inside. The lid should include a U-shaped slot with rounded corners that is parallel to the y axis and opens to the top (increasing y values), and the bottom of the U matches the button's position and diameter. The intent is for the material to be bent to actuate the button indirectly. Leave a space of 1 mm between the slot and the rest of the lid. Leave a minimal clearance of at most 1-2 mm to ensure the button can be pressed without excessive force.
* Provide a circular hole on the lid for the LED so it is visible from the outside.
* Provide a rectangular cutout on the lid for the switch so it can be toggled from the outside. The cutout must have its long side parallel to the y axis and short side parallel to the x axis.
* Include a cutout for the USB-C port centered at x = 34 mm, y = 50 mm, spanning z from 4 mm to 7 mm. USB-C approximate dimensions: 8.5 mm x 3 mm.
* Wall thickness should be of at least 3 mm to provide basic rigidity and resist small impacts.
* Round external corners and edges with a radius of around 2 mm.
* Add integrated handles/loops that span most of the length of both the short sides, leave 5 mm of clearance for a strap and be at least 5 mm thick. They will have a rectangular shape perpendicular to the walls and one of their sides will sit on the base plane. Keep rounded corners on the handle edges that do not touch the case, but make the two corners on the side that touches the case sharp (no rounding). The intent is for them to be 3d-printed without supports.
* Engrave a 3-axis drawing near the upper left region of the lid (low x values, high y values) aligned with the IMU orientation: xy plane parallel to the board, z axis pointing up, x arrow pointing left (decreasing x), and y arrow pointing down (decreasing y). Use a 1 mm engraving depth and 10 mm arrow length. Include the "x", "y", and "z" labels near the corresponding axis. Keep the full engraving (lines, arrowheads, and labels) inset from case edges so it does not touch or visually crowd any outer edge.

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
