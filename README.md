# Eurobot Project by Florian Reimat
## About 
This repo is fork from @iutgeiitoulon project, it's a minimal project for Robot Club Toulon's Robots, unless there are many editions architecture
of this project is  pretty similar to the source.

## Useful Links  
- [Valentin Gies](https://www.vgies.com/)
- [RCT Website](https://rct.univ-tln.fr/)


## Advancement
- [x] Edit all Class for adding Console Colors
	- [x] MsgDecoder
	- [x] MsgEncoder
	- [x] MsgGenerator
	- [x] MsgProcessor
	- [x] UsbVendor
- [ ] Make the Trajectory Planner
	- [x] Ghost Rotation
	- [x] Ghost Shifting
	- [ ] PID Controller
	- [ ] Robot Enslavement
- [ ] Improve WPF interface 
	- [x] Handle Multiple Waypoints 
	- [ ] Preview Trajectory Planner before applying
	- [x] Improve Ghost Preview
	- [ ] Add Rack State Wpf Controller
	- [ ] Add Points Estimation Wpf Controller
	- [ ] Make an only Match Wpf Controller
- [ ] Make an Strategy Manager
- [ ] Make Lidar Processing
	- [x] Make Clusters Detection
	- [x] Make Cup Detection
	- [x] Make Line Detection
	- [ ] Make Rectangle Fitting
- [x] Others
	- [x] Implementing Protocol Security

## Explanation of Project
- RobotEurobot2Roues:
	It's the main project, it connect all Event and Create the architecture of the program, by the way it's an Console program.

- WpfRobotEurobot2Roues:
	It's the Global Wpf project, it connect all Wpf asset and show them on a single window.

- WpfAsservissementDisplay + WpfOscilloscope + WpfWorldMapDisplay:
	These project are Wpf assets, some of them require Scichart Library 

- USBVendor + MsgDecoder + MsgEncoder + Protocol_Security:
	These project are for communication with the low-level chip. They add an communication protocol + security

- MsgProcessor:
	This project convert all received communication from low-level and launch the related event

- MsgGenerator:
	This project convert all data before sending to low-level

- EventArgsLibrary + Constants + Utilities:
	Theses project are just mess filled with everything uncategorised

- ConsoleFormat:
	It's just an project for formatting data and print on Console

- XBoxController:
	This is just an XBox Driver for managing robot
