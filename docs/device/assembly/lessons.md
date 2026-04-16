# 1st iteration

## Electronics

- Solder pins to components before attempting to use them on the breadboard. Otherwise you may run into hardware bugs or damage the components.
- Measure continuity with a multimeter before powering up the circuit. Validate the circuit on a breadboard before soldering on a protoboard.
- Think your soldering strategy through before you start soldering.
- Don't try to grab the wires with your fingers when soldering, they heat up quickly and will burn you. Use tweezers instead.
- Use hot glue to fix wires in place and for strain relief only when you are done. If you keep soldering, the wires will heat up and the glue will melt and lose its hold.
- Place your wrists on the table when soldering and only move the fingers. It will give you more stability and precision.
- If you feel that you need more than two hands, remember that you can solder complex joins in multiple steps if needed. Solder two components, let it cool down, then grab the third, make the solder liquid again and join it to the other two, and so on.
- Every time you buy a component, buy at least one extra as a backup in case you burn it or it turns out to be defective.
- Design the schematic with common components that are easily available. This is especially true in developing countries where the supply chain is not as good as in the US or Europe.
- Try to avoid counterfeit components if possible. They may be cheaper but may be unreliable or make things fail in unexpected ways.

## Mechanical design and enclosure

- If you need to rework too much an already printed case you're probably in trouble. 3D printed plastic can't be machined easily, at least with a big/powerful drill: the material may tear internally or heat up and melt, chips sometimes clog the holes, the bit can wobble even if you grab the drill firmly, the holes may end up bigger or oval. Try out: manual drill, step drilling, low RPMs, cleaning and cooling the bit periodically, sharp bits, clean chips periodically.
- Don't design or print anything on CAD UNTIL you have all the components in hand and can measure them. Don't assume you will be able to find them on the local hardware store or online, sometimes they are just not available. Don't assume something will fit just because the space available looks big enough on the screen.
- Always have some margin for error: screws slightly longer than necessary, holes with small extra diameter, extra space in the case for the board, having different types of glue available for rework, allow reversibility or disassembly, etc.
- Hot glue and epoxy are fine initially but they are not a good solution for the final product. They don't look good and may fail unexpectedly.
- OpenSCAD + vibe coding with a comprehensive prompt works, but it may not actually be a lot faster (and is certainly less ergonomic) than learning other CAD software such as Fusion360 or FreeCAD and design the case by hand with the mouse.

## Tooling

- Avoid cheap or very old tools if possible. They may be inaccurate, unreliable, or just plain frustrating to use.
- Choose the right tools for the job to prevent frustration and wasting time. Use precision tools for precision work.
- Use a soldering iron with temperature control, otherwise you may end up with cold joints or overheated components. Different solders require different temperatures.
- Measuring with a ruler is just a bad idea, it leads to design errors. Use calipers instead.

## Errors found

- The recording LED turns on briefly when I turn on the device, even if I don't press the record button. Find out why.
- Address if can solder everything more tightly and cut the protoboard cleanly to reduce the case size. It's a little bit big to bear on the chest.
- Try to place wires in 45° or 90° angles to make it look neater.
- Switch's rectangle: about 1mm more to the left.
- Usb-c hole: about 2mm more to the right, and bigger overall (can't plug cable).
- Base's interior bosses should be wider, at least 2-3mm more.
- All holes should be M3.25.
- Handles' short sides should be wider, at least 2-3mm more, but leaving the holes on roughly the same place. Alternatively, they should have material extensions to the sides to pass the screws through there. Maybe I can change from machine screws to self-tapping.
- If I use epoxy to compliment screws when securing the handles to the base, make a better surface for it to stick to, maybe some roughness or a saw pattern. Alternatively, consider snap fits or other mechanical solutions that don't require glue.
- The axis engraving didn't actually remove material from the surface.  
- Reduce the overall height of the case if possible.
