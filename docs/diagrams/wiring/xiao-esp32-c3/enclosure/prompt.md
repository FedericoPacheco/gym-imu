# Instructions

Generate an enclosure for a portable IMU device.

Refer to the schematics at: docs/diagrams/wiring/xiao-esp32-c3/schematics.puml

A prototype has already been assembled and photographed. See: docs/diagrams/wiring/xiao-esp32-c3/prototype/protoboard.

Considering the coordinate system:

* Long side: x axis,
* Short side: y axis,
* Height: z axis
* Origin: board's bottom left edge

The prototype has the following features:

* Protoboard size: 5 cm × 7 cm × 1 mm, with 2 mm diameter mounting holes at each corner; each hole is located approximately 1.5 mm from the board edges.
* The side with the copper traces is the bottom of the enclosure (negative z) and requires a clearance of at least 0.8 cm between the board and the case for wiring.
* All electronic components sit on the opposite side of the board (positive z) and require a clearance of 1.4 cm between the board and the case.
* Components center with respect to the origin:
  * Button: x = 1.3 cm, y = 1.2 cm
  * LED: x = 2.5 cm, y = 1.2 cm
  * Switch: x = 3.9 cm, y = 1.4 cm
  * IMU, microcontroller, and battery: not relevant
* Component dimensions:
  * Button: 1.2 cm × 1.2 cm
  * LED: 0.5 cm diameter
  * Switch: 2.1 cm × 1.5 cm
  * IMU, microcontroller, and battery: not relevant

Requirements for the enclosure design:

* The board will be secured with screws.
* The button will remain inside; the case should include a U-shaped cut so the material can be bent to actuate the button indirectly.
* Provide a circular hole for the LED so it is visible from the outside.
* Provide a rectangular cutout for the switch so it can be toggled from the outside.
* Between the LED and switch (at x = 3.0 cm, y = 0.0 cm, z = 0.75 cm) include a 3.5 mm hole in the side of the case for the RF antenna wire; the antenna will be adhered to the outside of the enclosure.
* Include a cutout for the USB-C port centered at x = 3.2 cm, y = 5.0 cm, spanning z from 0.4 cm to 0.7 cm.
* Add strap handles on the short sides of the enclosure.
* Wall thickness should be at least 0.3 cm to resist small impacts.
* Design with as few parts as possible while allowing disassembly for maintenance.
* Round edges to avoid sharp corners.
* Add an axis drawing on the top of the enclosure aligned with the IMU orientation.

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
* Avoid magic numbers; expose dimensions as parameters.
* Add comments to explain the code.
* Use OpenSCAD libraries (BOSL2, NopSCADlib) where appropriate.

If you need additional information, feel free to ask or explore the rest of the codebase.

Output:

Save the generated OpenSCAD code to: docs/diagrams/wiring/xiao-esp32-c3/enclosure/enclosure.scad and iterate by rendering and adjusting the design until it meets the requirements.
