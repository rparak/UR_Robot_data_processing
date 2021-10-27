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

// System Lib.
using System;
using System.Text;
using System.Net;
using System.Net.Sockets;
using System.Threading;

namespace UR_TCPip_RW_Console_app
{
    public static class UR_Stream_Data
    {
        // IP Port Number and IP Address
        public static string ip_address;
        //  Real-time (Read Only)
        public const ushort port_number = 30013;
        // Comunication Speed (ms)
        public static int time_step;
        // Joint Space:
        //  Orientation {J1 .. J6} (rad)
        public static double[] J_Orientation = new double[6];
        // Cartesian Space:
        //  Position {X, Y, Z} (mm)
        public static double[] C_Position = new double[3];
        //  Orientation {Euler Angles} (rad):
        public static double[] C_Orientation = new double[3];
    }

    public static class UR_Control_Data
    {
        // IP Port Number and IP Address
        public static string ip_address;
        //  Real-time (Read/Write)
        public const ushort port_number = 30003;
        // Comunication Speed (ms)
        public static int time_step;
        // Home Parameters UR3/UR3e:
        //  Joint Space:
        //      Orientation {J1 .. J6} (rad)
        public static double[] J_Orientation = new double[6] { -1.6, -1.7, -2.2, -0.8, 1.59, -0.03 };
        //  Cartesian Space:
        //      Position {X, Y, Z} (mm)
        public static double[] C_Position = new double[3] { -0.11, -0.26, 0.15 };
        //      Orientation {Euler Angles} (rad):
        public static double[] C_Orientation = new double[3] { 0.0, 3.11, 0.0 };
        // Move Parameters: Velocity, Acceleration
        public static string velocity = "1.0";
        public static string acceleration = "1.0";
    }

    class Program
    {
        static void Main(string[] args)
        {
            // Initialization {TCP/IP Universal Robots}
            //  Read Data:
            UR_Stream_Data.ip_address = "192.168.230.133";
            //  Communication speed: CB-Series 125 Hz, E-Series 500 Hz
            UR_Stream_Data.time_step = ((1 / 125) * 1000);
            //  Write Data:
            UR_Control_Data.ip_address = "192.168.230.133";
            //  Communication speed: CB-Series 125 Hz, E-Series 500 Hz
            UR_Control_Data.time_step = ((1 / 125) * 1000);

            // Start Stream {Universal Robots TCP/IP}
            UR_Stream ur_stream_robot = new UR_Stream();
            ur_stream_robot.Start();

            // Start Control {Universal Robots TCP/IP}
            UR_Control ur_ctrl_robot = new UR_Control();
            ur_ctrl_robot.Start();

            Console.WriteLine("[INFO] Stop (y):");
            // Stop communication
            string stop_rs = Convert.ToString(Console.ReadLine());

            if (stop_rs == "y")
            {
                Console.WriteLine("Joint Space: Orientation (radian)");
                Console.WriteLine("J1: {0} | J2: {1} | J3: {2} | J4: {3} | J5: {4} | J6: {5}",
                                   UR_Stream_Data.J_Orientation[0], UR_Stream_Data.J_Orientation[1], UR_Stream_Data.J_Orientation[2],
                                   UR_Stream_Data.J_Orientation[3], UR_Stream_Data.J_Orientation[4], UR_Stream_Data.J_Orientation[5]);

                Console.WriteLine("Cartesian Space: Position (metres), Orientation (radian):");
                Console.WriteLine("X: {0} | Y: {1} | Z: {2} | RX: {3} | RY: {4} | RZ: {5}",
                                   UR_Stream_Data.C_Position[0], UR_Stream_Data.C_Position[1], UR_Stream_Data.C_Position[2],
                                   UR_Stream_Data.C_Orientation[0], UR_Stream_Data.C_Orientation[1], UR_Stream_Data.C_Orientation[2]);

                // Stop UR {Control / Stream}
                ur_stream_robot.Stop();
                ur_ctrl_robot.Stop();

                // Application quit
                Environment.Exit(0);
            }
        }
    }

    class UR_Stream
    {
        // Initialization of variables
        private Thread robot_thread      = null;
        private TcpClient tcp_client     = new TcpClient();
        private NetworkStream network_stream = null;
        private bool exit_thread = false;
        private byte[] packet = new byte[2048];

        // Offset:
        //  Size of first packet in bytes (Integer)
        private const byte first_packet_size = 4;
        //  Size of other packets in bytes (Double)
        private const byte offset = 8;

        public void UR_Stream_Thread()
        {
            try
            {
                if (tcp_client.Connected == false)
                {
                    // Connect to controller -> if the controller is disconnected
                    tcp_client.Connect(UR_Stream_Data.ip_address, UR_Stream_Data.port_number);
                }

                // Initialization TCP/IP Communication (Stream)
                network_stream = tcp_client.GetStream();

                while (exit_thread == false)
                {
                    // Get the data from the robot
                    if (network_stream.Read(packet, 0, packet.Length) > 0)
                    {
                        // Reverses the order of elements in a one-dimensional array or part of an array.
                        Array.Reverse(packet);

                        // Note:
                        //  For more information on values 32... 37, etc., see the UR Client Interface document.

                        // Read Joint Values in radians
                        UR_Stream_Data.J_Orientation[0] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (32 * offset));
                        UR_Stream_Data.J_Orientation[1] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (33 * offset));
                        UR_Stream_Data.J_Orientation[2] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (34 * offset));
                        UR_Stream_Data.J_Orientation[3] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (35 * offset));
                        UR_Stream_Data.J_Orientation[4] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (36 * offset));
                        UR_Stream_Data.J_Orientation[5] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (37 * offset));

                        // Read Cartesian (Positon) Values in metres
                        UR_Stream_Data.C_Position[0] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (56 * offset));
                        UR_Stream_Data.C_Position[1] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (57 * offset));
                        UR_Stream_Data.C_Position[2] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (58 * offset));
                        // Read Cartesian (Orientation) Values in metres 
                        UR_Stream_Data.C_Orientation[0] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (59 * offset));
                        UR_Stream_Data.C_Orientation[1] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (60 * offset));
                        UR_Stream_Data.C_Orientation[2] = BitConverter.ToDouble(packet, packet.Length - first_packet_size - (61 * offset));
                    }
                    Thread.Sleep(UR_Stream_Data.time_step);
                }
            }
            catch (SocketException e)
            {
                Console.WriteLine("SocketException: {0}", e);
            }
        }

        public void Start()
        {
            // Start a thread and listen to incoming messages
            robot_thread = new Thread(new ThreadStart(UR_Stream_Thread));
            robot_thread.Start();
        }
        public void Stop()
        {
            // Stop and exit thread
            exit_thread = true;
            if (robot_thread == null && robot_thread.IsAlive == true)
            {
                // Disconnect communication
                network_stream.Dispose();
                tcp_client.Close();
                Thread.Sleep(1000);
            }
        }
    }

    class UR_Control
    {
        // Initialization of variables
        private Thread robot_thread = null;
        private TcpClient tcp_client = new TcpClient();
        private NetworkStream network_stream = null;
        private bool exit_thread = false;
        private UTF8Encoding utf8 = new UTF8Encoding();
        byte[] buffer_cmd;

        public void UR_Control_Thread()
        {
            try
            {

                if (tcp_client.Connected == false)
                {
                    // Connect to controller -> if the controller is disconnected
                    tcp_client.Connect(UR_Control_Data.ip_address, UR_Control_Data.port_number);
                }

                // Initialization TCP/IP Communication (Stream)
                network_stream = tcp_client.GetStream();

                while (exit_thread == false)
                {
                    // Note:
                    //  For more information about commands, see the URScript Programming Language document 

                    // Instruction 1 (Home Position): Joint Input Command, Move Joint Interpolation
                    //  Get Bytes from String
                    buffer_cmd = utf8.GetBytes("movej([" + UR_Control_Data.J_Orientation[0].ToString() + "," + UR_Control_Data.J_Orientation[1].ToString() + "," + UR_Control_Data.J_Orientation[2].ToString() + ","
                                                         + UR_Control_Data.J_Orientation[3].ToString() + "," + UR_Control_Data.J_Orientation[4].ToString() + "," + UR_Control_Data.J_Orientation[5].ToString() + "],"
                                                         + "a=" + UR_Control_Data.acceleration + ", v=" + UR_Control_Data.velocity + ")" + "\n");
                    //  Send command to the robot
                    network_stream.Write(buffer_cmd);
                    //  Wait Time (5 seconds)
                    Thread.Sleep(5000);

                    // Instruction 2 (Multiple Positions): Cartesian Input Command, Move Linear Interpolation
                    //  Get Bytes from String
                    buffer_cmd = utf8.GetBytes("[movel(p[" + UR_Control_Data.C_Position[0].ToString() + "," + UR_Control_Data.C_Position[1].ToString() + "," + (UR_Control_Data.C_Position[2] - 0.1).ToString() + ","
                                                           + UR_Control_Data.C_Orientation[0].ToString() + "," + UR_Control_Data.C_Orientation[1].ToString() + "," + UR_Control_Data.C_Orientation[2].ToString() + "],"
                                                           + "a=" + UR_Control_Data.acceleration + ", v=" + UR_Control_Data.velocity + ")," +
                                               "movel(p[" + (UR_Control_Data.C_Position[0] - 0.1).ToString() + ", " + UR_Control_Data.C_Position[1].ToString() + ", " + (UR_Control_Data.C_Position[2] - 0.1).ToString() + ", "
                                                           + UR_Control_Data.C_Orientation[0].ToString() + "," + UR_Control_Data.C_Orientation[1].ToString() + "," + UR_Control_Data.C_Orientation[2].ToString() + "],"
                                                           + "a=" + UR_Control_Data.acceleration + ", v=" + UR_Control_Data.velocity + ")," +
                                               "movel(p[" + (UR_Control_Data.C_Position[0] - 0.1).ToString() + ", " + UR_Control_Data.C_Position[1].ToString() + ", " + UR_Control_Data.C_Position[2].ToString() + ", "
                                                           + UR_Control_Data.C_Orientation[0].ToString() + "," + UR_Control_Data.C_Orientation[1].ToString() + "," + UR_Control_Data.C_Orientation[2].ToString() + "],"
                                                           + "a=" + UR_Control_Data.acceleration + ", v=" + UR_Control_Data.velocity + ")," +
                                               "movel(p[" + UR_Control_Data.C_Position[0].ToString() + ", " + UR_Control_Data.C_Position[1].ToString() + ", " + (UR_Control_Data.C_Position[2]).ToString() + ", "
                                                           + UR_Control_Data.C_Orientation[0].ToString() + "," + UR_Control_Data.C_Orientation[1].ToString() + "," + UR_Control_Data.C_Orientation[2].ToString() + "],"
                                                           + "a=" + UR_Control_Data.acceleration + ", v=" + UR_Control_Data.velocity + ")]" + "\n");
                    //  Send command to the robot
                    network_stream.Write(buffer_cmd);
                    //  Wait Time (5 seconds)
                    Thread.Sleep(5000);

                    // Instruction 3 (Multiple Positions): Cartesian Input Command, Move Joint Interpolation
                    //  Get Bytes from String
                    buffer_cmd = utf8.GetBytes("[movej(p[" + UR_Control_Data.C_Position[0].ToString() + "," + UR_Control_Data.C_Position[1].ToString() + "," + (UR_Control_Data.C_Position[2] - 0.1).ToString() + ","
                                                           + UR_Control_Data.C_Orientation[0].ToString() + "," + UR_Control_Data.C_Orientation[1].ToString() + "," + UR_Control_Data.C_Orientation[2].ToString() + "],"
                                                           + "a=" + UR_Control_Data.acceleration + ", v=" + UR_Control_Data.velocity + ")," +
                                               "movej(p[" + (UR_Control_Data.C_Position[0] - 0.1).ToString() + ", " + UR_Control_Data.C_Position[1].ToString() + ", " + (UR_Control_Data.C_Position[2] - 0.1).ToString() + ", "
                                                           + UR_Control_Data.C_Orientation[0].ToString() + "," + UR_Control_Data.C_Orientation[1].ToString() + "," + UR_Control_Data.C_Orientation[2].ToString() + "],"
                                                           + "a=" + UR_Control_Data.acceleration + ", v=" + UR_Control_Data.velocity + ")," +
                                               "movej(p[" + (UR_Control_Data.C_Position[0] - 0.1).ToString() + ", " + UR_Control_Data.C_Position[1].ToString() + ", " + UR_Control_Data.C_Position[2].ToString() + ", "
                                                           + UR_Control_Data.C_Orientation[0].ToString() + "," + UR_Control_Data.C_Orientation[1].ToString() + "," + UR_Control_Data.C_Orientation[2].ToString() + "],"
                                                           + "a=" + UR_Control_Data.acceleration + ", v=" + UR_Control_Data.velocity + ")," +
                                               "movej(p[" + UR_Control_Data.C_Position[0].ToString() + ", " + UR_Control_Data.C_Position[1].ToString() + ", " + (UR_Control_Data.C_Position[2]).ToString() + ", "
                                                           + UR_Control_Data.C_Orientation[0].ToString() + "," + UR_Control_Data.C_Orientation[1].ToString() + "," + UR_Control_Data.C_Orientation[2].ToString() + "],"
                                                           + "a=" + UR_Control_Data.acceleration + ", v=" + UR_Control_Data.velocity + ")]" + "\n");
                    //  Send command to the robot
                    network_stream.Write(buffer_cmd);
                    //  Wait Time (5 seconds)
                    Thread.Sleep(5000);
                }
            }
            catch (SocketException e)
            {
                Console.WriteLine("SocketException: {0}", e);
            }
        }

        public void Start()
        {
            // Start a thread and listen to incoming messages
            robot_thread = new Thread(new ThreadStart(UR_Control_Thread));
            robot_thread.Start();
        }
        public void Stop()
        {
            // Stop and exit thread
            exit_thread = true;
            if (robot_thread == null && robot_thread.IsAlive == true)
            {
                // Disconnect communication
                network_stream.Dispose();
                tcp_client.Close();
                Thread.Sleep(1000);
            }
        }
    }
}
