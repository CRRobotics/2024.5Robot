# 2024Robot
 
Its OK I guess, take a look around if you dare (to get confused by our wacky implementations)

## Remaining tasks
Tune PIDs
- Tune arm elevation PID

Shooter auto
- Test the pose-estimation
- Finally, fill the tables with data
- Test for repeatability

Shooter
- Yeah, shots are not consistent at all right now, must fix it somehow
- Test asymmetric shooter speeds to see if that improves stability

Auto modes
- Test Paths
- Test Autos (linking paths)
- Test Auto+vision
- Test Game piece recognition & pathing

User interaction system
- Design the competition user input scheme
- Design competition user feedback (LEDs, whats on smart dashboard, etc) system
- Implement the user input scheme
- Implement competition user feedback system

Final polish
- Scan for any remaining "// TODO:" comments
- Sort objects and methods into groups
- Refactor and clean implementations as much as possible
  - Fix bad namings
  - Add important comments
  - Condense, centralise, simplify, improve etc whatever you can
- Remove reduntant or depreciated object, imports, methods classes, etc