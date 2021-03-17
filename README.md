# TCP/IP Communication between Universal Robots UR3 (Server) and simple Client (C#)

## Requirements:

**Software:**
```bash
Universal Robots Polyscope, Visual Studio (or something similar)
```
Universal Robots: https://www.universal-robots.com/download/

Visual Studio: https://visualstudio.microsoft.com/downloads/

**Programming Language:**
```bash
C#, Another Language (Python, C/C++ -> similar implementation as C#)
```

**Packages:**
```bash
C# (.NET Framework 4.6.1)
```

## Project Description:

The project is focused on a simple demonstration of client / server communication via TCP / IP. In this case, it is a collaborative robot Universal Robots UR3 (server) and the client is a C# application. An example of an application is reading (joint/cartesian position) and writing data (movement -> linear, joint interpolation, etc.).

The application was tested on each of the robot types (UR3 -> real hardware + simulation, UR5, UR10, etc.) on the E and CB series (simulation using VMware). Communication (C# application) can be used in Unity3D for digital twins / augmented reality or in other relevant applications.

<p align="center">
<img src=https://github.com/rparak/UR_Robot_data_processing/blob/main/images/communication_scheme.png width="650" height="350">
</p>

## Project Hierarchy:

**Client (C#) - Repositary [/UR_Robot_data_processing/UR_TCPip_RW_Console_app/UR_TCPip_RW_Console_app/]:**

```bash
[ Main Program ] /Program.cs/
```

**Data Interfaces - Repositary [UR_Robot_data_processing/Clinet_Data_Interfaces/]:**

```bash
[ Datasheet ] /Client_InterfacesV3.14andV5.9.xlsx/
```

## Example of reading the position of a joint from different series of Universal Robots UR3:

## Contact Info:
Roman.Parak@outlook.com

## License
[MIT](https://choosealicense.com/licenses/mit/)
