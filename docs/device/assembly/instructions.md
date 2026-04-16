# Assembly instructions

1. Prepare tools as listed in [tools](tools.md).
2. Prepare materials as listed in the [bill of materials](bom.md).
3. If necessary, machine the protoboard to insert the switch, enlarge or make holes on the corners for screws, or reduce the board size.
4. Solder the components into the perfboard as shown in the [schematic](../diagrams/schematics/it2(xiao-esp32-c3)/schematic.svg). Also follow insights from [lessons learned](lessons.md) to avoid common pitfalls. The general order of assembly is as follows:

    4.1. Cut and bend two pieces of copper wire to solder to the back of the microcontroller that will pass through the protoboard holes. These will act as pins for the battery connection later.
    4.2. Place the microcontroller at the center and only solder the pins that are unused on this project.
    4.3. Place the IMU sensor to one side and only solder the pins that are unused on this project.
    4.4. Place short wires touching the necessary pins of the microcontroller and the sensor and solder them in one stroke. Leave the power and ground pins untouched.
    4.5. Make common wire lines for power and ground, and connect the respective pins of the microcontroller and the sensor to them.
    4.6. Solder the LED and button in any particular order.
    4.7. Solder the switch and battery wires.
    4.8. Secure wires with hot glue.
    4.9. Measure continuity with a multimeter.
    4.10. Flash the device and while plugged test it briefly. Review logs.
    4.11. Turn it on unplugged and test it briefly.

5. Measure the button, switch and LED placement with a caliper and change the 3D model of the case if necessary. Print the case.

6. Review the printed case and address if it needs any adjustments. If necessary:

    - Enlarge holes with a small or manual drill.
    - Roughen surfaces with sandpaper to make glue stick better.

7. Begin case + board assembly:

    7.1. If available, insert heat-set inserts inside the vertical holes of the base with a soldering iron.
    7.2. Pass screws with washers through the handles.
    7.3. Place epoxy glue on the handles.
    7.4. Attach the handles to the base through the horizontal holes, securing them with another washer and nut on the inside of the case. Tighten them and clean glue residue if it leaks out.
    7.5. Place the board inside the case and secure it on the sides and to the bosses of the base with hot glue. Make sure the holes align properly.
    7.6. Pass screws with washers through the base, board and lid. Secure them on the other side with another washer and nut. Tighten them.
    7.7. Sew the strap to the handles by hand following a box X pattern and either using a running stitch (faster to do, less strong) or back stitch (slower, stronger).

8. Perform a final test of the device as done in step 4.11.
