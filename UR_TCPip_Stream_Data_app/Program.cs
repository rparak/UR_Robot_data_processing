 /****************************************************************************
MIT License
Copyright(c) 2021 Roman Parak
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
using System.Net.Sockets;
using System.Threading;
using System.Diagnostics;
using System.Collections.Generic;

namespace UR_TCPip_Stream_Data_app
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
        public static List<double> J_Orientation_1 = new List<double>();
        public static List<double> J_Orientation_2 = new List<double>();
        public static List<double> J_Orientation_3 = new List<double>();
        public static List<double> J_Orientation_4 = new List<double>();
        public static List<double> J_Orientation_5 = new List<double>();
        public static List<double> J_Orientation_6 = new List<double>();
    }

    class Program
    {
        static void Main(string[] args)
        {
            // Variable used to save data to a file from the UR robot.
            bool save_data = true;

            // Initialization {TCP/IP Universal Robots}
            //  Read Data:
            UR_Stream_Data.ip_address = "192.168.230.132";
            //  Communication speed: CB-Series 125 Hz (8 ms), E-Series 500 Hz (2 ms)
            UR_Stream_Data.time_step = 2;

            // Start Stream {Universal Robots TCP/IP}
            UR_Stream ur_stream_robot = new UR_Stream();
            ur_stream_robot.Start();

            Console.WriteLine("[INFO] Stop (y):");
            // Stop communication
            string stop_rs = Convert.ToString(Console.ReadLine());

            if (stop_rs == "y")
            {
                if (save_data == true)
                {
                    // Write Data to file (.txt)
                    Write_Data(UR_Stream_Data.J_Orientation_1, UR_Stream_Data.J_Orientation_2, UR_Stream_Data.J_Orientation_3,
                               UR_Stream_Data.J_Orientation_4, UR_Stream_Data.J_Orientation_5, UR_Stream_Data.J_Orientation_6,
                               "D:\\UR10e_Stream_Data_Example.txt");

                    Console.WriteLine("[INFO] File saved successfully!");
                }

                // Destroy UR {Control / Stream}
                ur_stream_robot.Destroy();

                // Application quit
                Environment.Exit(0);
            }
        }
        public static void Write_Data(List<double> Joint_1, List<double> Joint_2, List<double> Joint_3,
                                      List<double> Joint_4, List<double> Joint_5, List<double> Joint_6,
                                      string file_path)
        {
            try
            {
                using (System.IO.StreamWriter file = new System.IO.StreamWriter(@file_path, true))
                {
                    for (int i = 0; i < Joint_1.Count; ++i)
                    {
                        file.WriteLine(Joint_1[i].ToString() + "," + Joint_2[i].ToString() + "," + Joint_3[i].ToString() + "," +
                                       Joint_4[i].ToString() + "," + Joint_5[i].ToString() + "," + Joint_6[i].ToString());
                    }
                }
            }
            catch (Exception ex)
            {
                throw new ApplicationException("Error: ", ex);
            }
        }
    }

    class UR_Stream
    {
        // Initialization of Class variables
        //  Thread
        private Thread robot_thread = null;
        private bool exit_thread = false;
        //  TCP/IP Communication
        private TcpClient tcp_client = new TcpClient();
        private NetworkStream network_stream = null;
        //  Packet Buffer (Read)
        private byte[] packet = new byte[1116];

        // Offset:
        //  Size of first packet in bytes (Integer)
        private const byte first_packet_size = 4;
        //  Size of other packets in bytes (Double)
        private const byte offset = 8;

        // Total message length in bytes
        private const UInt32 total_msg_length = 3288596480;

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

                // Initialization timer
                var t = new Stopwatch();

                while (exit_thread == false)
                {
                    // Get the data from the robot
                    if (network_stream.Read(packet, 0, packet.Length) != 0)
                    {
                        if (BitConverter.ToUInt32(packet, first_packet_size - 4) == total_msg_length)
                        {
                            // t_{0}: Timer start.
                            t.Start();

                            // Reverses the order of elements in a one-dimensional array or part of an array.
                            Array.Reverse(packet);

                            // Note:
                            //  For more information on values 32... 37, etc., see the UR Client Interface document.
                            // Read Joint Values in radians
                            UR_Stream_Data.J_Orientation_1.Add(BitConverter.ToDouble(packet, packet.Length - first_packet_size - (32 * offset)));
                            UR_Stream_Data.J_Orientation_2.Add(BitConverter.ToDouble(packet, packet.Length - first_packet_size - (33 * offset)));
                            UR_Stream_Data.J_Orientation_3.Add(BitConverter.ToDouble(packet, packet.Length - first_packet_size - (34 * offset)));
                            UR_Stream_Data.J_Orientation_4.Add(BitConverter.ToDouble(packet, packet.Length - first_packet_size - (35 * offset)));
                            UR_Stream_Data.J_Orientation_5.Add(BitConverter.ToDouble(packet, packet.Length - first_packet_size - (36 * offset)));
                            UR_Stream_Data.J_Orientation_6.Add(BitConverter.ToDouble(packet, packet.Length - first_packet_size - (37 * offset)));

                            // t_{1}: Timer stop.
                            t.Stop();

                            // Recalculate the time: t = t_{1} - t_{0} -> Elapsed Time in milliseconds
                            if (t.ElapsedMilliseconds < UR_Stream_Data.time_step)
                            {
                                Thread.Sleep(UR_Stream_Data.time_step - (int)t.ElapsedMilliseconds);
                            }

                            // Reset (Restart) timer.
                            t.Restart();
                        }
                    }
                }
            }
            catch (SocketException e)
            {
                Console.WriteLine("SocketException: {0}", e);
            }
        }

        public void Start()
        {
            exit_thread = false;
            // Start a thread to control Universal Robots (UR)
            robot_thread = new Thread(new ThreadStart(UR_Stream_Thread));
            robot_thread.IsBackground = true;
            robot_thread.Start();
        }
        public void Stop()
        {
            exit_thread = true;
            // Start a thread
            if (robot_thread.IsAlive == true)
            {
                Thread.Sleep(100);
            }
        }
        public void Destroy()
        {
            // Start a thread and disconnect tcp/ip communication
            Stop();
            if (tcp_client.Connected == true)
            {
                network_stream.Dispose();
                tcp_client.Close();
            }
            Thread.Sleep(100);
        }
    }
}