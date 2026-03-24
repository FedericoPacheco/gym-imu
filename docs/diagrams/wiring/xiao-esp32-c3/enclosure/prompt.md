# Instructions

Generate an enclosure for my portable IMU device.

Consider the schematics available at: docs/diagrams/wiring/xiao-esp32-c3/schematics.puml

A prototype has already been assembled. There're images of it available at: docs/diagrams/wiring/xiao-esp32-c3/prototype/protoboard. It has the following features:

* Protoboard size of 5cm x 7cm x 1mm, with 2mm diameter holes at each corner, sitting each one at approximately 1.5mm from the sides.
* The side with copper traces it's the bottom of the enclosure, and it needs a clearance of at least 0.8cm between the board and the case for wiring.
* All the electronic components sit on the other side of the board, and it needs a clearance of 1.4cm between the board and the case.
* Considering that the long side is the x axis, the short side the y axis, and the height the z axis, the displacement with respect to one edge of the board for the following components is:
  * Button (center): x: 1.3cm, y: 1.2cm
  * LED (center): x: 2.5cm, y: 1.2cm
  * Switch (center): x: 3.9cm, y: 1.4cm
  * IMU, microcontroller and battery: not relevant
* The dimensions of the components is:
  * Button: 1.2cm x 1.2cm
  * LED: 0.5cm diameter
  * Switch: 2.1cm x 1.5cm
  * IMU, microcontroller and battery: not relevant

Make sure to meet the following requirements for the enclosure design:

* The board will be secured with screws.
* The button will remain inside, but the case will be cut forming a "U" shape, so that the material can be bent and indirectly press the button.
* There will be a circular hole for the LED, so that it's visible from the outside.
* There will be a rectangular hole for the switch, so that it can be toggled from the outside.
* Between the LED and switch (x: 3cm, y: 0cm, and z: 0.75cm) there will be a 3.5mm hole on the side of case for the RF antenna wire, which will be adhered to the external side of the enclosure.
* There will be a hole for the USB-C port on x: 3.2cm, y: 5cm, and from z: 0.4cm to z: 0.7cm.
* On the short sides of the enclosure there will handles for a strap.  
* The enclosure walls shouldn't be thinner than 0.3cm, so that it can resist small impacts without breaking.
* Build the enclosure with as few parts as possible, leaving the possibility to disassemble it if needed.
* Round the edges of the enclosure to avoid sharp corners.
* Add a axis drawing on the top of the enclosure, following the orientation of the IMU.

Use the following tools, which are already installed on this computer:

* OpenSCAD. Docs:
  * Overview: <https://openscad.org/documentation.html>

* BOSL2 library. Docs:
  * Overview: <https://github.com/BelfrySCAD/BOSL2/wiki>
  * Table of contents: <https://github.com/BelfrySCAD/BOSL2/wiki/TOC>

* NopSCADlib library. Docs:
  * Overview: <https://github.com/nophead/NopSCADlib/blob/master/readme.md>
  * Usage: <https://github.com/nophead/NopSCADlib/blob/master/docs/usage.md>

Make sure to follow the following coding standards so that I can perform a code review later:

* Use descriptive names.
* Avoid magic numbers. Allow the dimensions to be changed with parameters.
* Add comments to explain the code.
* Use OpenSCAD libraries to simplify the code when possible.

If you need additional information, be free to ask or explore the rest of the codebase.

Output the created code to: docs/diagrams/wiring/xiao-esp32-c3/enclosure/enclosure.scad and iterate on it by rendering the design and making adjustments until it meets the requirements.
