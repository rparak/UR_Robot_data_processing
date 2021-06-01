/****************************************************************************
MIT License
Copyright(c) 2020 Roman Parak
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*****************************************************************************
Author   : Roman Parak
Email    : Roman.Parak @outlook.com
Github   : https://github.com/rparak
File Name: Program.cs
****************************************************************************/

// ------------------------------------------------------------------------------------------------------------------------//
// ----------------------------------------------------- LIBRARIES --------------------------------------------------------//
// ------------------------------------------------------------------------------------------------------------------------//

// -------------------- System -------------------- //
using System;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Threading;

namespace UR_TCPip_RW_Console_app
{
    class Program
    {
        // -------------------- Socket -------------------- //
        static Socket socket_read = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
        static Socket socket_write = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
        // -------------------- UTF8Encoding -------------------- //
        static UTF8Encoding utf8 = new UTF8Encoding();
        // -------------------- Bool -------------------- //
        static bool communication_read_isOk;
        static bool isMove;
        static bool start_move_cmd;
        static bool tcpip_r_while, tcpip_w_while;
        // -------------------- Byte -------------------- //
        static byte[] cmd;
        static byte[] packet_ur = new byte[746];
        static byte[] joint_1 = new byte[8]; static byte[] joint_2 = new byte[8]; static byte[] joint_3 = new byte[8];
        static byte[] joint_4 = new byte[8]; static byte[] joint_5 = new byte[8]; static byte[] joint_6 = new byte[8];
        static byte[] x_cartesian = new byte[8]; static byte[] y_cartesian = new byte[8];
        static byte[] z_cartesian = new byte[8]; static byte[] rx_cartesian = new byte[8];
        static byte[] ry_cartesian = new byte[8]; static byte[] rz_cartesian = new byte[8];
        // -------------------- String -------------------- //
        static string move;
        static string par_nJCacc, par_nJCvel;
        static string par_nJCt;
        static string par_nJacc, par_nJvel;
        static string par_nJt;
        // -------------------- Double -------------------- //
        static double par_nJ1, par_nJ2, par_nJ3, par_nJ4, par_nJ5, par_nJ6;
        static double par_nX, par_nY, par_nZ, par_nRX, par_nRY, par_nRZ;
        static double[] robotBaseRotLink_UR3_c = { 0f, 0f, 0f, 0f, 0f, 0f };
        static double[] robotBaseRotLink_UR3_j = { 0f, 0f, 0f, 0f, 0f, 0f };
        static double[] robotBaseRotLink_UR3_before = { 0f, 0f, 0f, 0f, 0f, 0f };
        static double[] robotBaseRotLink_UR3_aux = { 0f, 0f, 0f, 0f, 0f, 0f };
        // -------------------- Thread -------------------- //
        static Thread tcpip_read_Thread, tcpip_write_Thread;
        // -------------------- Int -------------------- //
        static int byteCount_s;

        // ------------------------------------------------------------------------------------------------------------------------//
        // ------------------------------------------------ MAIN FUNCTION {Cyclic} ------------------------------------------------//
        // ------------------------------------------------------------------------------------------------------------------------//
        static void Main(string[] args)
        {
            // ------------------------ Initialization { TCP/IP Read Data} ------------------------//
            // Robot IP Address
            string ip_adr_robot = "192.168.230.129";
            // Robot Port
            int port_adr_robot  = 30003;

            // ------------------------ Threading Block { TCP/IP Read Data } ------------------------//
            tcpip_r_while = true;

            tcpip_read_Thread = new Thread(() => TCPip_read_thread_function(ip_adr_robot, port_adr_robot));
            tcpip_read_Thread.IsBackground = true;
            tcpip_read_Thread.Start();

            // ------------------------ Threading Block { TCP/IP Write Data } ------------------------//
            tcpip_w_while = true;

            tcpip_write_Thread = new Thread(() => TCPip_write_thread_function(ip_adr_robot, port_adr_robot));
            tcpip_write_Thread.IsBackground = true;
            tcpip_write_Thread.Start();

            // ------------------------ Main Block { Control of the Robot (UR) } ------------------------//
            try
            {
                // Joint Position [° -> radian]
                par_nJ1 = Degree_to_radian(90);
                par_nJ2 = Degree_to_radian(-90);
                par_nJ3 = Degree_to_radian(-125);
                par_nJ4 = Degree_to_radian(-45);
                par_nJ5 = Degree_to_radian(90);
                par_nJ6 = Degree_to_radian(360);
                // Time [s]
                par_nJt = "2.0";
                // Acceleration [°/(s^2)] -> {Definition: 20 [°/(s^2)] -> Convert to radian = Math.PI * 20 / 180.0}
                par_nJacc = "0.5";
                // Velocity [°/s] ->  {Definition: 20 [°/s] = 0.2 -> Convert to radian = Math.PI * 20 / 180.0}
                par_nJvel = "0.3";
                // TCP Position [mm -> m]
                par_nX = MM_to_M(150);
                par_nY = MM_to_M(-270);
                par_nZ = MM_to_M(200);
                // Rotation Vector [° -> radian]
                par_nRX = Degree_to_radian(0);
                par_nRY = Degree_to_radian(180);
                par_nRZ = Degree_to_radian(0);
                // Time [s]
                par_nJCt = "2.0";
                // Acceleration [mm/(s^2)] -> {Definition: 200 [mm/(s^2)] = 0.2}
                par_nJCacc = "0.5";
                // Velocity [mm/s] -> {Definition: 200 [mm/s] = 0.2}
                par_nJCvel = "0.3";

                // start command {move instruction}
                start_move_cmd = true;

                // -------------------- Main Cycle {While} -------------------- //
                while (true)
                {
                    // -------------------- Communication {Check} -------------------- //
                    if (communication_read_isOk == true)
                    {
                        // Read data {Joint (1 - 6); Cartesian (X, Y, Z; RX, RY, RZ)} 
                        // Uncomment one of the following lines to read the data (Joint, Position)
                        //Console.WriteLine("Joint 1: {0}", robotBaseRotLink_UR3_j[0]);
                        //Console.WriteLine("Position X: {0}", robotBaseRotLink_UR3_c[0]);

                        // Thread Sleep {100 ms}
                        Thread.Sleep(100);
                    }

                    // -------------------- Move Instruction {Start} -------------------- //
                    if (start_move_cmd == true)
                    {
                        // Create command for motion -> Command_instruction_move(command state)
                        // 0 - Joint(a,v); 1 - Joint(t); 2 - CartesJ(a,v); 3 - CartesJ(t); 4 - CartesL(a,v); 5 - CartesL(t)
                        move = Command_instruction_move(1);
                        // command string -> Bytes 
                        cmd = utf8.GetBytes(move);
                        // change state {TCPip_write_thread_function} -> Start
                        isMove = true;
                    }
                    else
                    {
                        // change state {TCPip_write_thread_function} -> Stop
                        isMove = false;
                    }
                }
            }
            catch (Exception e) {
                Application_Quit();
                Console.WriteLine(e.Message);
            }
        }

        // ------------------------------------------------------------------------------------------------------------------------//
        // -------------------------------------------------------- FUNCTIONS -----------------------------------------------------//
        // ------------------------------------------------------------------------------------------------------------------------//

        // -------------------- Aux. Function {Degree to Radian} -------------------- //
        static double Degree_to_radian(double degree)
        {
            return (Math.PI * degree / 180.0);
        }

        // -------------------- Aux. Function {millimetres to metres} -------------------- //
        static double MM_to_M(double mm)
        {
            return mm/1000;
        }

        // -------------------- Command Instruction {UR robot} -------------------- //
        static string Command_instruction_move(int state)
        {
            // null -> result command value
            string result_cmd = "";

            switch (state)
            {
                case 0:
                    {
                        result_cmd = "movej([" + par_nJ1.ToString() + "," + par_nJ2.ToString() + "," + par_nJ3.ToString() + "," + par_nJ4.ToString() + "," + par_nJ5.ToString() + "," + par_nJ6.ToString() + "], a=" + par_nJacc + ", v=" + par_nJvel + ")" + "\n";
                    }
                    break;
                case 1:
                    {
                        result_cmd = "movej([" + par_nJ1.ToString() + "," + par_nJ2.ToString() + "," + par_nJ3.ToString() + "," + par_nJ4.ToString() + "," + par_nJ5.ToString() + "," + par_nJ6.ToString() + "], t=" + par_nJt + ")" + "\n";
                    }
                    break;
                case 2:
                    {
                        result_cmd = "movej(p[" + par_nX.ToString() + "," + par_nY.ToString() + "," + par_nZ.ToString() + "," + par_nRX.ToString() + "," + par_nRY.ToString() + "," + par_nRZ.ToString() + "], a=" + par_nJCacc + ", v=" + par_nJCvel + ")" + "\n";
                    }
                    break;
                case 3:
                    {
                        result_cmd = "movej(p[" + par_nX.ToString() + "," + par_nY.ToString() + "," + par_nZ.ToString() + "," + par_nRX.ToString() + "," + par_nRY.ToString() + "," + par_nRZ.ToString() + "], t=" + par_nJCt + ")" + "\n";
                    }
                    break;
                case 4:
                    {
                        result_cmd = "movel(p[" + par_nX.ToString() + "," + par_nY.ToString() + "," + par_nZ.ToString() + "," + par_nRX.ToString() + "," + par_nRY.ToString() + "," + par_nRZ.ToString() + "], a=" + par_nJCacc + ", v=" + par_nJCvel + ")" + "\n";
                    }
                    break;
                case 5:
                    {
                        result_cmd = "movel(p[" + par_nX.ToString() + "," + par_nY.ToString() + "," + par_nZ.ToString() + "," + par_nRX.ToString() + "," + par_nRY.ToString() + "," + par_nRZ.ToString() + "], t=" + par_nJCt + ")" + "\n";
                    }
                    break;
            }

            return result_cmd;
        }

        // -------------------- Abort Threading Blocks -------------------- //
        static void Application_Quit()
        {
            try
            {
                // Control -> Stop {Read TCP/IP data}
                tcpip_r_while = false;
                // Control -> Stop {Write TCP/IP data}
                tcpip_w_while = false;

                // Abort threading block {TCP/Ip -> read data}
                if (tcpip_read_Thread.IsAlive == true)
                {
                    tcpip_read_Thread.Abort();
                }

                // Abort threading block {TCP/Ip  -> write data}
                if (tcpip_write_Thread.IsAlive == true)
                {
                    tcpip_write_Thread.Abort();
                }

                // Socket Read {shutdown -> both (Receive, Send)}
                socket_read.Shutdown(SocketShutdown.Both);
                // Socket Write {shutdown -> both (Receive, Send)}
                socket_write.Shutdown(SocketShutdown.Both);
            }
            catch (Exception e)
            {
                Console.WriteLine(e.Message);
            }
            finally
            {
                // Socket Read {close}
                socket_read.Close();
                // Socket Write {close}
                socket_write.Close();
            }
        }

        // ------------------------ Threading Block { TCP/IP Read Data } ------------------------//
        static void TCPip_read_thread_function(string ip_adr, int port_adr)
        {
            // Initialization TCP/IP Communication
            IPAddress ip      = IPAddress.Parse(ip_adr);
            IPEndPoint ip_end = new IPEndPoint(ip, port_adr);

            if (socket_read.Connected == false)
            {
                // connect to controller -> if the controller is disconnected
                socket_read.Connect(ip_end);
            }

            // Threading while {read date}
            while (tcpip_r_while)
            {
                // Receive data from UR packet
                byteCount_s = socket_read.Receive(packet_ur);

                // Check Length
                if (byteCount_s == packet_ur.Length)
                {
                    // set variable {communation is ok}
                    communication_read_isOk = true;

                    // -------------------- JOINT {Read} -------------------- //
                    // Joint 1
                    joint_1[0] = packet_ur[259]; joint_1[1] = packet_ur[258]; joint_1[2] = packet_ur[257];
                    joint_1[3] = packet_ur[256]; joint_1[4] = packet_ur[255]; joint_1[5] = packet_ur[254];
                    joint_1[6] = packet_ur[253]; joint_1[7] = packet_ur[252];
                    // Joint 2
                    joint_2[0] = packet_ur[267]; joint_2[1] = packet_ur[266]; joint_2[2] = packet_ur[265];
                    joint_2[3] = packet_ur[264]; joint_2[4] = packet_ur[263]; joint_2[5] = packet_ur[262];
                    joint_2[6] = packet_ur[261]; joint_2[7] = packet_ur[260];
                    // Joint 3
                    joint_3[0] = packet_ur[275]; joint_3[1] = packet_ur[274]; joint_3[2] = packet_ur[273];
                    joint_3[3] = packet_ur[272]; joint_3[4] = packet_ur[271]; joint_3[5] = packet_ur[270];
                    joint_3[6] = packet_ur[269]; joint_3[7] = packet_ur[268];
                    // Joint 4
                    joint_4[0] = packet_ur[283]; joint_4[1] = packet_ur[282]; joint_4[2] = packet_ur[281];
                    joint_4[3] = packet_ur[280]; joint_4[4] = packet_ur[279]; joint_4[5] = packet_ur[278];
                    joint_4[6] = packet_ur[277]; joint_4[7] = packet_ur[276];
                    // Joint 5
                    joint_5[0] = packet_ur[291]; joint_5[1] = packet_ur[290]; joint_5[2] = packet_ur[289];
                    joint_5[3] = packet_ur[288]; joint_5[4] = packet_ur[287]; joint_5[5] = packet_ur[286];
                    joint_5[6] = packet_ur[285]; joint_5[7] = packet_ur[284];
                    // Joint 6
                    joint_6[0] = packet_ur[299]; joint_6[1] = packet_ur[298]; joint_6[2] = packet_ur[297];
                    joint_6[3] = packet_ur[296]; joint_6[4] = packet_ur[295]; joint_6[5] = packet_ur[294];
                    joint_6[6] = packet_ur[293]; joint_6[7] = packet_ur[292];

                    // data editing for reading
                    robotBaseRotLink_UR3_aux[0] = (float)Math.Round(BitConverter.ToDouble(joint_1, 0) * (180 / Math.PI), 2);
                    robotBaseRotLink_UR3_aux[1] = (float)Math.Round(BitConverter.ToDouble(joint_2, 0) * (180 / Math.PI), 2);
                    robotBaseRotLink_UR3_aux[2] = (float)Math.Round(BitConverter.ToDouble(joint_3, 0) * (180 / Math.PI), 2);
                    robotBaseRotLink_UR3_aux[3] = (float)Math.Round(BitConverter.ToDouble(joint_4, 0) * (180 / Math.PI), 2);
                    robotBaseRotLink_UR3_aux[4] = (float)Math.Round(BitConverter.ToDouble(joint_5, 0) * (180 / Math.PI), 2);
                    robotBaseRotLink_UR3_aux[5] = (float)Math.Round(BitConverter.ToDouble(joint_6, 0) * (180 / Math.PI), 2);

                    if ((robotBaseRotLink_UR3_aux[0] != 0) && (robotBaseRotLink_UR3_aux[1] != 0) &&
                        (robotBaseRotLink_UR3_aux[2] != 0) && (robotBaseRotLink_UR3_aux[3] != 0) &&
                        (robotBaseRotLink_UR3_aux[4] != 0) && (robotBaseRotLink_UR3_aux[5] != 0))
                    {
                        robotBaseRotLink_UR3_j[0] = (float)Math.Round(BitConverter.ToDouble(joint_1, 0) * (180 / Math.PI), 2);
                        robotBaseRotLink_UR3_j[1] = (float)Math.Round(BitConverter.ToDouble(joint_2, 0) * (180 / Math.PI), 2);
                        robotBaseRotLink_UR3_j[2] = (float)Math.Round(BitConverter.ToDouble(joint_3, 0) * (180 / Math.PI), 2);
                        robotBaseRotLink_UR3_j[3] = (float)Math.Round(BitConverter.ToDouble(joint_4, 0) * (180 / Math.PI), 2);
                        robotBaseRotLink_UR3_j[4] = (float)Math.Round(BitConverter.ToDouble(joint_5, 0) * (180 / Math.PI), 2);
                        robotBaseRotLink_UR3_j[5] = (float)Math.Round(BitConverter.ToDouble(joint_6, 0) * (180 / Math.PI), 2);

                        robotBaseRotLink_UR3_before[0] = robotBaseRotLink_UR3_j[0];
                        robotBaseRotLink_UR3_before[1] = robotBaseRotLink_UR3_j[1];
                        robotBaseRotLink_UR3_before[2] = robotBaseRotLink_UR3_j[2];
                        robotBaseRotLink_UR3_before[3] = robotBaseRotLink_UR3_j[3];
                        robotBaseRotLink_UR3_before[4] = robotBaseRotLink_UR3_j[4];
                        robotBaseRotLink_UR3_before[5] = robotBaseRotLink_UR3_j[5];
                    }
                    else
                    {
                        robotBaseRotLink_UR3_j[0] = robotBaseRotLink_UR3_before[0];
                        robotBaseRotLink_UR3_j[1] = robotBaseRotLink_UR3_before[1];
                        robotBaseRotLink_UR3_j[2] = robotBaseRotLink_UR3_before[2];
                        robotBaseRotLink_UR3_j[3] = robotBaseRotLink_UR3_before[3];
                        robotBaseRotLink_UR3_j[4] = robotBaseRotLink_UR3_before[4];
                        robotBaseRotLink_UR3_j[5] = robotBaseRotLink_UR3_before[5];
                    }

                    // -------------------- CARTESIAN {Read} -------------------- //
                    // X
                    x_cartesian[0] = packet_ur[451]; x_cartesian[1] = packet_ur[450]; x_cartesian[2] = packet_ur[449];
                    x_cartesian[3] = packet_ur[448]; x_cartesian[4] = packet_ur[447]; x_cartesian[5] = packet_ur[446];
                    x_cartesian[6] = packet_ur[445]; x_cartesian[7] = packet_ur[444];
                    // Y
                    y_cartesian[0] = packet_ur[459]; y_cartesian[1] = packet_ur[458]; y_cartesian[2] = packet_ur[457];
                    y_cartesian[3] = packet_ur[456]; y_cartesian[4] = packet_ur[455]; y_cartesian[5] = packet_ur[454];
                    y_cartesian[6] = packet_ur[453]; y_cartesian[7] = packet_ur[452];
                    // Z
                    z_cartesian[0] = packet_ur[467]; z_cartesian[1] = packet_ur[466]; z_cartesian[2] = packet_ur[465];
                    z_cartesian[3] = packet_ur[464]; z_cartesian[4] = packet_ur[463]; z_cartesian[5] = packet_ur[462];
                    z_cartesian[6] = packet_ur[461]; z_cartesian[7] = packet_ur[460];
                    // RX
                    rx_cartesian[0] = packet_ur[475]; rx_cartesian[1] = packet_ur[474]; rx_cartesian[2] = packet_ur[473];
                    rx_cartesian[3] = packet_ur[472]; rx_cartesian[4] = packet_ur[471]; rx_cartesian[5] = packet_ur[470];
                    rx_cartesian[6] = packet_ur[469]; rx_cartesian[7] = packet_ur[468];
                    // RY
                    ry_cartesian[0] = packet_ur[483]; ry_cartesian[1] = packet_ur[482]; ry_cartesian[2] = packet_ur[481];
                    ry_cartesian[3] = packet_ur[480]; ry_cartesian[4] = packet_ur[479]; ry_cartesian[5] = packet_ur[478];
                    ry_cartesian[6] = packet_ur[477]; ry_cartesian[7] = packet_ur[476];
                    // RZ
                    rz_cartesian[0] = packet_ur[491]; rz_cartesian[1] = packet_ur[490]; rz_cartesian[2] = packet_ur[489];
                    rz_cartesian[3] = packet_ur[488]; rz_cartesian[4] = packet_ur[487]; rz_cartesian[5] = packet_ur[486];
                    rz_cartesian[6] = packet_ur[485]; rz_cartesian[7] = packet_ur[484];

                    // data editing for reading
                    robotBaseRotLink_UR3_c[0] = Math.Round(BitConverter.ToDouble(x_cartesian, 0) * (1000), 2);
                    robotBaseRotLink_UR3_c[1] = Math.Round(BitConverter.ToDouble(y_cartesian, 0) * (1000), 2);
                    robotBaseRotLink_UR3_c[2] = Math.Round(BitConverter.ToDouble(z_cartesian, 0) * (1000), 2);
                    robotBaseRotLink_UR3_c[3] = Math.Round(BitConverter.ToDouble(rx_cartesian, 0) * (180 / Math.PI), 4);
                    robotBaseRotLink_UR3_c[4] = Math.Round(BitConverter.ToDouble(ry_cartesian, 0) * (180 / Math.PI), 4);
                    robotBaseRotLink_UR3_c[5] = Math.Round(BitConverter.ToDouble(rz_cartesian, 0) * (180 / Math.PI), 4);
                }
            }
            // disconnect tcp/ip communication (read data) after stopping the threading
            socket_read.Disconnect(true);
        }
        // ------------------------ Threading Block {TCP/IP Write Data} ------------------------//
        static void TCPip_write_thread_function(string ip_adr, int port_adr)
        {
            // Initialization TCP/IP Communication
            IPAddress ip = IPAddress.Parse(ip_adr);
            IPEndPoint ip_end = new IPEndPoint(ip, port_adr);

            if (socket_write.Connected == false)
            {
                // connect to controller -> if the controller is disconnected
                socket_write.Connect(ip_end);
            }

            // Initialization of the State-machine
            int state_ursend = 0;

            // Threading while {write command}
            while (tcpip_w_while)
            {
                switch (state_ursend)
                {
                    case 0:
                        {
                            // Command from main { true? Change state and send command to robot)
                            if (isMove == true)
                            {
                                // go to the motion state
                                state_ursend = 1;
                            }

                        }
                        break;
                    case 1:
                        {
                            // start move command -> turn off
                            start_move_cmd = false;

                            // send command
                            socket_write.Send(cmd);

                            // Thread Sleep {1000 ms}
                            Thread.Sleep(1000);

                            // go to the initialization state
                            state_ursend = 0;
                        }
                        break;
                }
            }
            // disconnect tcp/ip communication (read data) after stopping the threading
            socket_write.Disconnect(true);
        }
    }
}
