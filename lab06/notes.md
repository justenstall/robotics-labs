# Lab 6 Notes

## Sensor Info

### Roomba 600 Drive-on Charger

160  Reserved
161  Force Field
164  Green Buoy
165  Green Buoy and Force Field
168  Red Buoy
169  Red Buoy and Force Field
172  Red Buoy and Green Buoy
173  Red Buoy, Green Buoy and Force Field

Reserved = 160
Force Field = 1
Green Buoy = 4
Red Buoy = 8

### Charging Sources Available

0  None
1  Internal Charger
2  Home Base (dock)
3  Both

## Ideas

Treat the dock force field as a wall in the wall-follow PID loop, so we drive around it until we can see a buoy.

chase's runs at 30 degree angle until the omni sensor detects the far buoy (quasi-centered)

rotate until the left IR detects the right sensor

inverse relationship of the IR sensor sides