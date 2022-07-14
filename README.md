# Eurobot Project by Florian Reimat
## About 
This repo is the main code for Robot Club Toulon 2nd team robot during Eurobot 2021.

## Demo
I'm sorry i don't have a full view demo, check [RCT Instagram](https://www.instagram.com/robotclubtoulon/) for other demo
![Demo](misc/demo.gif)

## Useful Links
- [RCT Official Repo](https://github.com/iutgeiitoulon/Eurobot2021TwoWheels)
- [Valentin Gies](https://www.vgies.com/)
- [RCT Website](https://rct.univ-tln.fr/)


## Advancement
- [x] Edit all Class for adding Console Colors
	- [x] MsgDecoder
	- [x] MsgEncoder
	- [x] MsgGenerator
	- [x] MsgProcessor
	- [x] UsbVendor
- [x] Make the Trajectory Planner
	- [x] Ghost Rotation
	- [x] Ghost Shifting
	- [x] PID Controller
	- [x] Robot Enslavement
- [x] Improve WPF interface 
	- [x] Handle Multiple Waypoints 
	- [ ] Preview Trajectory Planner before applying
	- [x] Improve Ghost Preview
	- [x] Add Rack State Wpf Controller
	- [x] Add Points Estimation Wpf Controller
	- [x] Make an only Match Wpf Controller
- [x] Make an Strategy Manager
- [x] Make Lidar Processing
	- [x] Make Clusters Detection
	- [x] Make Cup Detection
	- [x] Make Line Detection
	- [x] Make Rectangle Fitting
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
