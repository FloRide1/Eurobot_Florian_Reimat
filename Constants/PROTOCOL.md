# Protocol Table
## Description 
This is an explanation of all implemented protocol in Commands.cs 
- Name : The Command Name
- Code : The Function of the Message
- PC → 2R : Function who can be send from PC to low-level
- 2R → PC : Function who can be send from low-level to PC
- Payload : The size of the payload (...) 
- Timestamp : If the timestamp is present in the payload
- Need to Edit : This is just if needed to convert function from Holonomic Robot to 2 Wheels Robot


| Name                                    | Code   | PC → 2R | 2R → PC | Payload Size | Timestamp | Need to Edit |
|-----------------------------------------|--------|---------|---------|--------------|-----------|--------------| 
| Welcome Message                         | 0x0100 |    🗙    |    ✔    |       0      |     🗙     |              |
| Error Message                           | 0x0101 |    🗙    |    ✔    |   Variable   |     🗙     |              |
| IMU Data                                | 0x0110 |    🗙    |    ✔    |      28      |     ✔     |              | 
| IO Monitoring                           | 0x0120 |    🗙    |    ✔    |       5      |     ✔     |              |
| Power Monitoring                        | 0x0130 |    🗙    |    ✔    |      20      |     ✔     |              |
| Encoder Raw Data                        | 0x0140 |    🗙    |    ✔    |      36      |     ✔     |              |
| Speed Polar And Independant Odometry    | 0x0150 |    🗙    |    ✔    |      32      |     ✔     | Yes          |
| Speed Auxiliary Odometry                | 0x0151 |    🗙    |    ✔    |      16      |     ✔     |              |
| Speed Polar PID Debug Error Corr        | 0x0152 |    🗙    |    ✔    |      40      |     ✔     | Yes          |
| Speed Indepedant PID Debug Error Corr   | 0x0153 |    🗙    |    ✔    |      52      |     ✔     | Yes          |
| Speed Polar PID Debug Internal          | 0x0154 |    🗙    |    ✔    |      40      |     ✔     | Yes          |
| Speed Independant PID Debug Internal    | 0x0155 |    🗙    |    ✔    |      52      |     ✔     | Yes          |
| Speed Auxiliary Motor Consignes         | 0x0156 |    🗙    |    ✔    |      36      |     ✔     |              |
| Motor Currents Monitoring               | 0x0160 |    🗙    |    ✔    |      36      |     ✔     |              |
| IO Polling Enable Status                | 0x0180 |    🗙    |    ✔    |       1      |     🗙     |              |
| Power Monitoring Enable Status          | 0x0181 |    🗙    |    ✔    |       1      |     🗙     |              |
| Encoder Raw Monitoring Enable Status    | 0x0182 |    🗙    |    ✔    |       1      |     🗙     |              |
| Asservissement Mode Status              | 0x0183 |    🗙    |    ✔    |       1      |     🗙     |              |
| Speed PID Enable Debug Error Status     | 0x0184 |    🗙    |    ✔    |       1      |     🗙     |              |
| Speed PID Enable Debug Internal Status  | 0x0185 |    🗙    |    ✔    |       1      |     🗙     |              |
| Speed Consigne Monitoring Enable Status | 0x0186 |    🗙    |    ✔    |       1      |     🗙     |              |
| Motors Enable Disable Status            | 0x0187 |    🗙    |    ✔    |       1      |     🗙     |              |
| Motor Current Monitoring Enable Status  | 0x0188 |    🗙    |    ✔    |       1      |     🗙     |              |
| Tir  Enable Disable Status              | 0x0189 |    🗙    |    ✔    |       1      |     🗙     | Yes          |
| Emergency Stop                          | 0x0200 |    ✔    |    🗙    |    Unknown   |     🗙     |              |
| IO Polling Enable                       | 0x0220 |    ✔    |    🗙    |       1      |     🗙     |              |
| IO Polling Set Frequency                | 0x0221 |    ✔    |    🗙    |       1      |     🗙     |              |
| Power Monitoring Enable                 | 0x0230 |    ✔    |    🗙    |       1      |     🗙     |              |
| Encoder Raw Monitoring Enable           | 0x0240 |    ✔    |    🗙    |       1      |     🗙     |              |
| Odometry Point To Meter                 | 0x0241 |    ✔    |    🗙    |       4      |     🗙     |              |
| 4 Wheels Angle Set                      | 0x0242 |    ✔    |    🗙    |      16      |     🗙     | Useless      |
| 4 Wheels  To Polar Matrix Set           | 0x0243 |    ✔    |    🗙    |      48      |     🗙     | Useless      |
| 2 Wheels Angle Set                      | 0x0244 |    ✔    |    🗙    |       2      |     🗙     |              |
| 2 Wheels To Polar Matrix Set            | 0x0245 |    ✔    |    🗙    |      16      |     🗙     |              |
| Set Asservissement Mode                 | 0x0250 |    ✔    |    🗙    |       1      |     🗙     |              |
| Speed PID Enable Debug Error Corr       | 0x0251 |    ✔    |    🗙    |       1      |     🗙     |              |
| Speed PID Enable Debug Internal         | 0x0252 |    ✔    |    🗙    |       1      |     🗙     |              |
| Speed Consigne Monitoring Enable        | 0x0253 |    ✔    |    🗙    |       1      |     🗙     |              |
| Speed Polar PID Set Gains               | 0x0254 |    ✔    |    🗙    |      72      |     🗙     | Yes          |
| Speed Independant PID Set Gains         | 0x0255 |    ✔    |    🗙    |      96      |     🗙     | Yes          |
| Speed Polar Set Consigne                | 0x0256 |    ✔    |    🗙    |      12      |     🗙     | Yes          |
| Speed Individual Motor Set Consigne     | 0x0257 |    ✔    |    🗙    |       5      |     🗙     |              |
| Speed PID Reset                         | 0x0258 |    ✔    |    🗙    |       0      |     🗙     |              |
| Motors Enable Disable                   | 0x0260 |    ✔    |    🗙    |       1      |     🗙     |              |
| Motor Current Monitoring Enable         | 0x0261 |    ✔    |    🗙    |       1      |     🗙     |              |
| Tir Enable Disable                      | 0x0270 |    ✔    |    🗙    |       1      |     🗙     | Useless      |
| Tir Command                             | 0x0271 |    ✔    |    🗙    |      14      |     🗙     | Useless      |
| Tir Move Up                             | 0x0272 |    ✔    |    🗙    |       0      |     🗙     | Useless      |
| Tir Move Down                           | 0x0273 |    ✔    |    🗙    |       0      |     🗙     | Useless      |
| Herkulex Forward                        | 0x0280 |    ✔    |    🗙    |   Variable   |     🗙     | Maybe        |
| Pololu Servo Set Position               | 0x0290 |    ✔    |    🗙    |   Undefined  |     🗙     | Unknown      |

## More precision
Timestamp are always first bytes of the frame and there are counted in the Payload size 
