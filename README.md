#  <h1 align="center"> MicroMouse :checkered_flag:</h1> 
### An intelligent maze solving car 🧠🏎️

MicroMouse can solve a complex maze in the shortest amount of time while maintaining a decent velocity profile. It can accomplish this by utilising a combination of closed-loop controllers such as PID and a variety of conditions. It is able to tackle the line following problem using an array of infrared sensors and achieve the same results.

## :computer: Code
The code was written on the Atmega328P board. All low-level drivers were implemented by us, to achieve the best performance and the least code size.

## :money_with_wings: Components & cost
| Name           | Use & description          | Number | Market Cost per Part                                                                                                         |
| -------------- | -------------------------- | ------ | ---------------------------------------------------------------------------------------------------------------------------- |
| Arduino Uno    | The main development board | 1      | [160 EGP](https://free-electronic.com/product/arduino-uno-r3-ch340-usb-cable/)                                               |
| TCRT 5000      | The IR sensor              | 5      | [10 EGP](https://store.fut-electronics.com/products/tcrt5000-reflective-ir-sensor?_pos=1&_sid=65bee30f3&_ss=r)               |
| Gearbox Motors | The used motors            | 2      | [25 EGP](https://store.fut-electronics.com/products/dc-geared-motors-for-robots-straight-shaft?_pos=33&_sid=eb26e25ca&_ss=r) |
| Car chassis    | Car’s Main body            | 1      | 190 EGP                                                                                                                      |
| Motor Driver   | The L298N H-bridge         | 1      | [70 EGP](https://store.fut-electronics.com/products/l298-dual-motor-driver-module-2a?_pos=16&_sid=eb26e25ca&_ss=r)           |
| Car batteries  | Lithium Rechargeable ones  | 3      | 35 EGP                                                                                                                       |
| Battery Holder | To hold the batteries      | 1      | 15 EGP                                                                                                                       |

## :pushpin: Schematic diagram
Our Car consists of 5 IR sensors and 2 DC motors. In this schematic, we show the connections of the sensors, motors, and power connections.

![Copy of Line Follower](https://user-images.githubusercontent.com/56788883/171509716-32decac7-9016-49d1-b802-5e7c3174b95f.png)
>> Note that:
We have used the L293N motor driver, but since it is not supported in the simulation, we have shown the connections of the L293D H-bridge with a voltage regulator. 
We have used the TCRT 5000 IR sensors, but since it is not supported in the simulation, we have used another module.

## Body
#### :triangular_ruler: Design
![image](https://user-images.githubusercontent.com/56788883/171510096-97784792-5c41-4698-8ae8-d633fbc45c80.png)
![image](https://user-images.githubusercontent.com/56788883/171510265-c1c543d9-fc6b-44f2-9289-b67fdd7f553c.png)
#### After Printing
![image](https://user-images.githubusercontent.com/56788883/171510400-8cd16ee7-a4d2-4c00-b876-99614bdb7078.png)


## Challenges :construction:
- **The Tight Range of the IR Sensor**: One of the most crucial issues we have faced was the tight range of the IR sensor, we weren’t able to agree on a cutting threshold between the white and black colors that was robust to other color and heat changes. We have solved this problem by using some resistors connected with the sensor and tuning its value until we reached the best range and threshold. That came with the cost of slower readings, but a few milliseconds longer wasn’t very critical to our application.
- **Varying Voltage Level**: Simply, we were somehow tuning the optimum velocity profile for our car. But, the velocity was affected by how much the batteries are charged. We managed to overcome this issue by using an AC/DC power adapter during the testing and adding fully charged batteries during the competition.
- **Number of IR Sensors**: We managed to tackle the line following problem using only 3 sensors but, that wasn't enough to do the same in solving the maze. We had to use extra two sensors; three for following the line and the other two to detect the turns.


## Contributors

<table>
  <tr>
    <td align="center">
    <a href="https://github.com/GhiathAjam" target="_black">
    <img src="https://avatars.githubusercontent.com/u/43111805?v=4" width="150px;" alt="Gheiath Ajam"/>
    <br />
    <sub><b>Gheiath Ajam</b></sub></a>
    </td>
    <td align="center">
    <a href="https://github.com/Ziyadhassan" target="_black">
    <img src="https://avatars.githubusercontent.com/u/56728268?v=4" width="150px;" alt="Ziyad Hassan"/>
    <br />
    <sub><b>Ziyad Hassan</b></sub></a>
    </td>
    <td align="center">
    <a href="https://github.com/AhmedKhaled590" target="_black">
    <img src="https://avatars.githubusercontent.com/u/62337087?v=4" width="150px;" alt="Ahmed Khaled"/>
    <br />
    <sub><b>Ahmed Khaled</b></sub></a>
    </td>
    <td align="center">
    <a href="https://github.com/ahmed-8areeb" target="_black">
    <img src="https://avatars.githubusercontent.com/u/62256670?v=4" width="150px;" alt="Ahmed Ghareeb"/>
    <br />
    <sub><b>Ahmed Ghareeb</b></sub></a>
    </td>
    <td align="center">
    <a href="https://github.com/HazemAbdo" target="_black">
    <img src="https://avatars.githubusercontent.com/u/59124058?v=4" width="150px;" alt="Hazem Abdo"/>
    <br />
    <sub><b>Hazem Abdo</b></sub></a>
    </td>
    <td align="center">
    <a href="https://github.com/mhmdahmedfathi" target="_black">
    <img src="https://avatars.githubusercontent.com/u/52926511?v=4" width="150px;" alt="Mohamed Ahmed"/>
    <br />
    <sub><b>Mohamed Ahmed</b></sub></a>
    </td>
  </tr>
  <tr>
    <td align="center">
    <a href="https://github.com/maryemsalah22" target="_black">
    <img src="https://avatars.githubusercontent.com/u/56718680?v=4" width="150px;" alt="Maryam Salah"/>
    <br />
    <sub><b>Maryam Salah</b></sub></a>
    </td>
    <td align="center">
    <a href="https://github.com/hoskillua" target="_black">
    <img src="https://avatars.githubusercontent.com/u/47090776?v=4" width="150px;" alt="Hossam Saeed"/>
    <br />
    <sub><b>Hossam Saeed</b></sub></a>
    </td>
    <td align="center">
    <a href="https://github.com/nadeenay" target="_black">
    <img src="https://avatars.githubusercontent.com/u/70846138?v=4" width="150px;" alt="Nadeen Ayman"/>
    <br />
    <sub><b>Nadeen Ayman</b></sub></a>
    </td>
    <td align="center">
    <a href="https://github.com/meryacine" target="_black">
    <img src="https://avatars.githubusercontent.com/u/56920956?v=4" width="150px;" alt="Omar Yacine"/>
    <br />
    <sub><b>Omar Yacine</b></sub></a>
    </td>
    <td align="center">
    <a href="https://github.com/AhmedMahmoudHafez" target="_black">
    <img src="https://avatars.githubusercontent.com/u/93212160?v=4" width="150px;" alt="Ahmed Hafez"/>
    <br />
    <sub><b>Ahmed Hafez</b></sub></a>
    </td>
    <td align="center">
    <a href="https://github.com/BigFish2086" target="_black">
    <img src="https://avatars.githubusercontent.com/u/63132227?v=4" width="150px;" alt="Ahmed Mohamed Ibrahim"/>
    <br />
    <sub><b>Ahmed Mohamed Ibrahim/b></sub></a>
    </td>
  </tr>
 </table>
