# TCP/IP Communication between Universal Robots UR3 (Server) and simple Client (C#)

## Requirements:

**Software:**
```bash
Universal Robots Polyscope, Visual Studio (or something similar)
```

| Software/Package           | Link                                                                                  |
| -------------------------- | ------------------------------------------------------------------------------------- |
| Universal Robots Polyscope | https://www.universal-robots.com/download/                                                     |
| Visual Studio              | https://visualstudio.microsoft.com/downloads/                                         |

**Programming Language:**
```bash
C#, Another Language (Python, C/C++ -> similar implementation as C#)
```

**Packages:**
```bash
C# (.NET Framework 4.6.1)
```

## Project Description:

The project is focused on a simple demonstration of client / server communication via TCP / IP. In this case, it is a collaborative robot Universal Robots UR3 / UR3e (server), which communicates with the client via the C# application. An example of an application is reading (Joint / Cartesian position) and writing data (movement -> linear, joint interpolation, etc.). The application was tested on each of the robot types (UR3 -> real hardware + simulation, UR5, UR10, etc.) E and CB series (simulation using VMware <-> UR Polyscope in Windows).

Writing data is really simple, just create a variable of type STRING in accordance with certain principles, transform the data into a BYTE and send this command to the Server. Data is read using packets on the server (see the file for a description of the packet -> /Client_Data_Interfaces/).

The application uses performance optimization using multi-threaded programming. Communication (C# application) can be used in Unity3D for digital twins / augmented reality or in other relevant applications. 

Sample application in the Unity3D program (Digital-Twin):

[Universal Robots UR3 - Unity3D Robotics](https://github.com/rparak/Unity3D_Robotics_UR)

The project was realized at Institute of Automation and Computer Science, Brno University of Technology, Faculty of Mechanical Engineering (NETME Centre - Cybernetics and Robotics Division).

<p align="center">
<img src=https://github.com/rparak/UR_Robot_data_processing/blob/main/images/communication_scheme.png width="650" height="350">
</p>

## Project Hierarchy:

**Client (C#) - Repositary [/UR_Robot_data_processing/UR_TCPip_RW_Console_app/]:**

```bash
[ Main Program ] /Program.cs/
```

**Data Interfaces - Repositary [UR_Robot_data_processing/Client_Data_Interfaces/]:**

```bash
[ Datasheet ] /Client_InterfacesV3.14andV5.9.xlsx/
```

## Example of reading the position of a joint from different series of Universal Robots UR3:

<p align="center">
<img src=https://github.com/rparak/UR_Robot_data_processing/blob/main/images/cb_1.PNG width="650" height="350">
<img src=https://github.com/rparak/UR_Robot_data_processing/blob/main/images/e_1.PNG width="650" height="350">
</p>

## Contact Info:
Roman.Parak@outlook.com

## License
[MIT](https://choosealicense.com/licenses/mit/)
