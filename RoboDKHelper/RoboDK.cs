using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Net.Sockets;
using Microsoft.Win32;

namespace RoboDKHelper
{

    /***************************************************************************************
     * 
     *    说明：该类是用来负责和RoboDK软件通信的基础类，提供所有RoboDK软件中子物品的操作基础方法
     * 
     ********************************************************************************************/
    
    /// <summary>
    /// RoboDK的异常类
    /// </summary>
    public class RDKException : Exception
    {
        /// <summary>
        /// 实例化一个对象
        /// </summary>
        /// <param name="Message"></param>
        public RDKException(string Message)
            : base(Message)
        {

        }
    }
    
    
    /// <summary>
    /// 创建并自动化Robodk的链接。所有子项目的操作都是通过该连接来完成的
    /// robodk tree（它可以是机器人，对象，工具，框架，程序，...）。
    /// </summary>
    public class RoboDK
    {
        // Tree item types
        public const int ITEM_TYPE_ANY =                   -1;   // 任意
        public const int ITEM_TYPE_STATION =                1;   // 站点
        public const int ITEM_TYPE_ROBOT =                  2;   // 机器人
        public const int ITEM_TYPE_FRAME =                  3;   // 参考系
        public const int ITEM_TYPE_TOOL =                   4;   // 工具
        public const int ITEM_TYPE_OBJECT =                 5;   // 对象
        public const int ITEM_TYPE_TARGET =                 6;   // 目标点
        public const int ITEM_TYPE_PROGRAM =                8;   // 程序
        public const int ITEM_TYPE_INSTRUCTION =            9;   // 命令
        public const int ITEM_TYPE_PROGRAM_PYTHON =         10;  // Python程序
        public const int ITEM_TYPE_MACHINING =              11;  // 机器
        public const int ITEM_TYPE_BALLBARVALIDATION =      12;  // 球杆验证
        public const int ITEM_TYPE_CALIBPROJECT = 13;
        public const int ITEM_TYPE_VALID_ISO9283 = 14;

        // Instruction types
        public const int INS_TYPE_INVALID =                -1;  // 无效
        public const int INS_TYPE_MOVE = 0;
        public const int INS_TYPE_MOVEC = 1;
        public const int INS_TYPE_CHANGESPEED = 2;
        public const int INS_TYPE_CHANGEFRAME = 3;
        public const int INS_TYPE_CHANGETOOL = 4;
        public const int INS_TYPE_CHANGEROBOT = 5;
        public const int INS_TYPE_PAUSE = 6;
        public const int INS_TYPE_EVENT = 7;
        public const int INS_TYPE_CODE = 8;
        public const int INS_TYPE_PRINT = 9;

        // Move types
        public const int MOVE_TYPE_INVALID = -1;
        public const int MOVE_TYPE_JOINT = 1;
        public const int MOVE_TYPE_LINEAR = 2;
        public const int MOVE_TYPE_CIRCULAR = 3;

        // Station parameters request
        public const string PATH_OPENSTATION = "PATH_OPENSTATION";
        public const string FILE_OPENSTATION = "FILE_OPENSTATION";
        public const string PATH_DESKTOP = "PATH_DESKTOP";

        // Script execution types
        public const int RUNMODE_SIMULATE = 1;                      // performs the simulation moving the robot (default)
        public const int RUNMODE_QUICKVALIDATE = 2;                 // performs a quick check to validate the robot movements
        public const int RUNMODE_MAKE_ROBOTPROG = 3;                // makes the robot program
        public const int RUNMODE_MAKE_ROBOTPROG_AND_UPLOAD = 4;     // makes the robot program and updates it to the robot
        public const int RUNMODE_MAKE_ROBOTPROG_AND_START = 5;      // makes the robot program and starts it on the robot (independently from the PC)
        public const int RUNMODE_RUN_ROBOT = 6;                     // moves the real robot from the PC (PC is the client, the robot behaves like a server)

        // Program execution type
        public const int PROGRAM_RUN_ON_SIMULATOR = 1;        // Set the program to run on the simulator
        public const int PROGRAM_RUN_ON_ROBOT = 2;            // Set the program to run on the robot

        // TCP calibration types
        public const int CALIBRATE_TCP_BY_POINT = 0;
        public const int CALIBRATE_TCP_BY_PLANE = 1;

        // projection types (for AddCurve)
        public const int PROJECTION_NONE = 0; // No curve projection
        public const int PROJECTION_CLOSEST = 1; // The projection will the closest point on the surface
        public const int PROJECTION_ALONG_NORMAL = 2; // The projection will be done along the normal.
        public const int PROJECTION_ALONG_NORMAL_RECALC = 3; // The projection will be done along the normal. Furthermore, the normal will be recalculated according to the surface normal.

        // Euler type
        public const int EULER_RX_RYp_RZpp = 0; // generic
        public const int EULER_RZ_RYp_RXpp = 1; // ABB RobotStudio
        public const int EULER_RZ_RYp_RZpp = 2; // Kawasaki, Adept, Staubli
        public const int EULER_RZ_RXp_RZpp = 3; // CATIA, SolidWorks
        public const int EULER_RX_RY_RZ = 4; // Fanuc, Kuka, Motoman, Nachi
        public const int EULER_RZ_RY_RX = 5; // CRS
        public const int EULER_QUEATERNION = 6; // ABB Rapid

        // State of the RoboDK window
        public const int WINDOWSTATE_HIDDEN = -1;
        public const int WINDOWSTATE_SHOW = 0;
        public const int WINDOWSTATE_MINIMIZED = 1;
        public const int WINDOWSTATE_NORMAL = 2;
        public const int WINDOWSTATE_MAXIMIZED = 3;
        public const int WINDOWSTATE_FULLSCREEN = 4;
        public const int WINDOWSTATE_CINEMA = 5;
        public const int WINDOWSTATE_FULLSCREEN_CINEMA = 6;

        // Instruction program call type:
        public const int INSTRUCTION_CALL_PROGRAM = 0;
        public const int INSTRUCTION_INSERT_CODE = 1;
        public const int INSTRUCTION_START_THREAD = 2;
        public const int INSTRUCTION_COMMENT = 3;

        
        
        private int SAFE_MODE { get; set; } = 1;                      // checks that provided items exist in memory
        private int AUTO_UPDATE { get; set; } = 0;                    // if AUTO_UPDATE is zero, the scene is rendered after every function call  
        private int TIMEOUT { get; set; } = 10 * 1000;                    // timeout for communication, in seconds
        private Socket COM { get; set; }                             // tcpip com
        private string IP { get; set; } = "127.0.0.1";                // IP address of the simulator (localhost if it is the same computer), otherwise, use RL = Robolink('yourip') to set to a different IP
        private int PORT { get; set; } = -1;                          // port where connection succeeded



        #region 初始化及连接断开


        /// <summary>
        /// 实例化一个roboDK对象，并连接到roboDK软件
        /// </summary>
        /// <param name="robodk_ip"></param>
        /// <param name="robodk_port"></param>
        public RoboDK(string robodk_ip = "127.0.0.1",int robodk_port=20500)
        {
            //A connection is attempted upon creation of the object"""
            if (robodk_ip != "")
            {
                IP = robodk_ip;
            }
            PORT = robodk_port;
            Connect();
        }

        /// <summary>
        /// Disconnect from the RoboDK API. This flushes any pending program generation.
        /// </summary>
        /// <returns></returns>
        public bool Disconnect()
        {
            if (COM.Connected)
            {
                COM.Disconnect(false);
            }
            return true;
        }


        private bool Set_connection_params(int safe_mode = 1, int auto_update = 0, int timeout = -1)
        {
            //Sets some behavior parameters: SAFE_MODE, AUTO_UPDATE and TIMEOUT.
            SAFE_MODE = safe_mode;
            AUTO_UPDATE = auto_update;
            if (timeout >= 0)
            {
                TIMEOUT = timeout;
            }
            send_line("CMD_START");
            send_line(Convert.ToString(SAFE_MODE) + " " + Convert.ToString(AUTO_UPDATE));
            string response = rec_line();
            if (response == "READY")
            {
                return true;
            }
            return false;
        }

        /// <summary>
        /// 连接到roboDK软件
        /// </summary>
        /// <returns></returns>
        private bool Connect()
        {
            bool connected = false;
            COM?.Close();
            COM = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
            try
            {
                COM.Connect(IP, PORT);
                connected = true;
            }
            catch
            {

            }

            if (connected && !Set_connection_params())
            {
                connected = false;
            }
            return connected;
        }


        #endregion


        #region 状态检查器
        

        /// <summary>
        /// Checks if the object is currently linked to RoboDK
        /// 检查是否连接到了RoboDK软件
        /// </summary>
        /// <returns></returns>
        public bool Connected()
        {
            //return COM.Connected;//does not work well
            bool part1 = COM.Poll(1000, SelectMode.SelectRead);
            bool part2 = COM.Available == 0;
            if (part1 && part2)
            {
                return false;
            }
            else
            {
                return true;
            }
        }

        //If we are not connected it will attempt a connection, if it fails, it will throw an error
        private void check_connection()
        {
            //if (!is_connected() && !Connect())
            //{
            //    throw new RDKException("Can't connect to RoboDK library");
            //}
        }

        /// <summary>
        /// 接收服务器的roboDK的状态
        /// </summary>
        /// <returns></returns>
        private int check_status()
        {
            int status = rec_int();
            if (status > 0 && status < 10)
            {
                string strproblems;
                strproblems = "Unknown error";
                if (status == 1)
                {
                    strproblems = "Invalid item provided: The item identifier provided is not valid or it does not exist.";
                }
                else if (status == 2)
                {
                    strproblems = rec_line();
                    return 0;
                }
                else if (status == 3)
                { // output error
                    strproblems = rec_line();
                    throw new RDKException(strproblems);
                }
                else if (status == 9)
                {
                    strproblems = "Invalid license. Contact us at: info@robodk.com";
                }
                else
                {
                    throw new RDKException(strproblems); //raise Exception(strproblems)
                }
            }
            else if (status == 0)
            {
                // everything is OK
            }
            else
            {
                throw new RDKException("Problems running function");
            }
            return status;
        }

        //Formats the color in a vector of size 4x1 and ranges [0,1]
        private bool check_color(double[] color)
        {
            if (color.Length < 4)
            {
                throw new RDKException("Invalid color. A color must be a 4-size double array [r,g,b,a]"); //raise Exception('Problems running function');
                //return false;
            }
            return true;
        }
        
        #endregion


        #region 基础发送接收指令


        /// <summary>
        /// 发送一行数据
        /// </summary>
        /// <param name="line"></param>
        private void send_line(string line)
        {
            line = line.Replace('\n', ' ');// one new line at the end only! 看不懂这行代码什么意思，完全没有用啊
            byte[] data = Encoding.UTF8.GetBytes(line + "\n");
            COM.Send(data);
        }

        /// <summary>
        /// 从RoboDK软件接收一行数据
        /// </summary>
        /// <returns></returns>
        private string rec_line()
        {
            //Receives a string. It reads until if finds LF (\\n)
            byte[] buffer = new byte[1];
            int bytesread = COM.Receive(buffer, 1, SocketFlags.None);

            System.IO.MemoryStream ms = new System.IO.MemoryStream();

            while (bytesread > 0 && buffer[0] != (byte)'\n')
            {
                ms.WriteByte(buffer[0]);
                bytesread = COM.Receive(buffer, 1, SocketFlags.None);
            }
            string result= Encoding.UTF8.GetString(ms.ToArray());
            ms.Dispose();
            return result;
        }

        /// <summary>
        /// 从SOCKET接收一定数据长度的字节，直到接收到为止
        /// </summary>
        /// <param name="size"></param>
        /// <returns></returns>
        private byte[] ReadBytesFromSocket(int size)
        {
            byte[] data = new byte[size];
            int received = 0;
            while (received < size)
            {
                int count = COM.Receive(data, received, size - received, SocketFlags.None);
                received += count;
            }
            return data;
        }


        private void send_int(Int32 number)
        {
            byte[] bytes = BitConverter.GetBytes(number);
            Array.Reverse(bytes); 
            COM.Send(bytes);
        }

        private int rec_int()
        {
            byte[] bytes = ReadBytesFromSocket(4);
            Array.Reverse(bytes); 
            return BitConverter.ToInt32(bytes, 0);
        }


        private void send_ulong(ulong number)
        {
            byte[] bytes = BitConverter.GetBytes(number);
            Array.Reverse(bytes); 
            COM.Send(bytes);
        }

        private ulong rec_ulong()
        {
            byte[] bytes = ReadBytesFromSocket(8);
            Array.Reverse(bytes); 
            return BitConverter.ToUInt64(bytes, 0);
        }


        private void send_double(double number)
        {
            byte[] bytes = BitConverter.GetBytes(number);
            Array.Reverse(bytes); 
            COM.Send(bytes);
        }


        private double rec_double()
        {
            byte[] bytes = ReadBytesFromSocket(8);
            Array.Reverse(bytes); 
            return BitConverter.ToDouble(bytes, 0);
        }

        
        
        #endregion













        //Sends an item pointer
        private void SendItem(Item item)
        {
            ulong itemPtr = item == null ? 0L : item.GetItemPtr();
            send_ulong(itemPtr);
        }
        
        private Item ReceiveItem()
        {
            ulong ptr = rec_ulong();
            int type = rec_int();
            return new Item(this, ptr, type);
        }

        private void SendPose(Matrix pose)
        {
            if (!pose.IsHomogeneous())
            {
                // warning!!
                return;
            }
            
            // 此处必须将数据整理成一个byte[]一起发送，否则roboDK软件不会自动接收满数据
            byte[] bytesarray = new byte[8 * 16];
            int cnt = 0;
            for (int j = 0; j < pose.cols; j++)
            {
                for (int i = 0; i < pose.rows; i++)
                {
                    byte[] onedouble = BitConverter.GetBytes(pose[i, j]);
                    Array.Reverse(onedouble);
                    Array.Copy(onedouble, 0, bytesarray, cnt * 8, 8);
                    cnt = cnt + 1;
                }
            }
            COM.Send(bytesarray, 8 * 16, SocketFlags.None);
        }

        private Matrix ReceivePose()
        {
            Matrix pose = new Matrix(4, 4);
            byte[] bytes = ReadBytesFromSocket(16 * 8);
            int cnt = 0;
            for (int j = 0; j < pose.cols; j++)
            {
                for (int i = 0; i < pose.rows; i++)
                {
                    byte[] onedouble = new byte[8];
                    Array.Copy(bytes, cnt, onedouble, 0, 8);
                    Array.Reverse(onedouble);
                    pose[i, j] = BitConverter.ToDouble(onedouble, 0);
                    cnt = cnt + 8;
                }
            }
            return pose;
        }

        private void SendXyz(double[] xyzpos)
        {
            for (int i = 0; i < 3; i++)
            {
                byte[] bytes = BitConverter.GetBytes(xyzpos[i]);
                Array.Reverse(bytes);
                COM.Send(bytes, 8, SocketFlags.None);
            }
        }
        private void ReceiveXyz(double[] xyzpos)
        {
            byte[] bytes = ReadBytesFromSocket(3 * 8);
            for (int i = 0; i < 3; i++)
            {
                byte[] onedouble = new byte[8];
                Array.Copy(bytes, i * 8, onedouble, 0, 8);
                Array.Reverse(onedouble);
                xyzpos[i] = BitConverter.ToDouble(onedouble, 0);
            }
        }



        // Sends an array of doubles
        private void SendDoubleArray(double[] values)
        {
            if (values == null)
            {
                send_int(0);
                return;
            }
            int nvalues = values.Length;
            send_int(nvalues);
            byte[] bytesarray = new byte[8 * nvalues];
            for (int i = 0; i < nvalues; i++)
            {
                byte[] onedouble = BitConverter.GetBytes(values[i]);
                Array.Reverse(onedouble);
                onedouble.CopyTo(bytesarray, i * 8);
            }
            COM.Send(bytesarray, SocketFlags.None);
        }

        // Receives an array of doubles
        private double[] ReceiveArray()
        {
            int nvalues = rec_int();
            if (nvalues > 0)
            {
                double[] values = new double[nvalues];
                byte[] bytes = ReadBytesFromSocket(nvalues * 8);
                for (int i = 0; i < nvalues; i++)
                {
                    byte[] onedouble = new byte[8];
                    Array.Copy(bytes, i * 8, onedouble, 0, 8);
                    Array.Reverse(onedouble);
                    values[i] = BitConverter.ToDouble(onedouble, 0);
                }
                return values;
            }
            return null;
        }
        
        void SendMatrix(Matrix mat)
        {
            if (mat == null)
            {
                send_int(0);
                send_int(0);
                return;
            }
            send_int(mat.rows);
            send_int(mat.cols);
            for (int j = 0; j < mat.cols; j++)
            {
                for (int i = 0; i < mat.rows; i++)
                {
                    byte[] bytes = BitConverter.GetBytes(mat[i, j]);
                    Array.Reverse(bytes);
                    COM.Send(bytes, 8, SocketFlags.None);
                }
            }

        }

        
        Matrix ReceiveMatrix()
        {
            int size1 = rec_int();
            int size2 = rec_int();
            int recvsize = size1 * size2 * 8;
            byte[] bytes = new byte[recvsize];
            Matrix mat = new Matrix(size1, size2);
            if (recvsize > 0)
            {
                bytes = ReadBytesFromSocket(recvsize);
            }
            int cnt = 0;
            for (int j = 0; j < mat.cols; j++)
            {
                for (int i = 0; i < mat.rows; i++)
                {
                    byte[] onedouble = new byte[8];
                    Array.Copy(bytes, cnt, onedouble, 0, 8);
                    Array.Reverse(onedouble);
                    mat[i, j] = BitConverter.ToDouble(onedouble, 0);
                    cnt = cnt + 8;
                }
            }
            return mat;
        }

       
        private void MoveX(Item target, double[] joints, Matrix mat_target, Item itemrobot, int movetype, bool blocking = true)
        {
            itemrobot.WaitMove();
            string command = "MoveX";
            send_line(command);
            send_int(movetype);
            if (target != null)
            {
                send_int(3);
                SendDoubleArray(null);
                SendItem(target);
            }
            else if (joints != null)
            {
                send_int(1);
                SendDoubleArray(joints);
                SendItem(null);
            }
            else if (mat_target != null && mat_target.IsHomogeneous())
            {
                send_int(2);
                SendDoubleArray(mat_target.ToDoubles());
                SendItem(null);
            }
            else
            {
                throw new RDKException("Invalid target type"); //raise Exception('Problems running function');
            }
            SendItem(itemrobot);
            check_status();

            if (blocking)
            {
                itemrobot.WaitMove();
            }
        }

        // .Net Framework 4.5及以上才支持dynamic参数类型
        //void moveX(dynamic target, Item itemrobot, int movetype, bool blocking = true)
        //{
        //    itemrobot.WaitMove();
        //    string command = "MoveX";
        //    send_line(command);
        //    send_int(movetype);

        //    if(target is Item item)
        //    {
        //        send_int(3);
        //        send_array(null);
        //        send_item(item);
        //    }
        //    else if(target is Matrix mat)
        //    {
        //        send_int(2);
        //        send_array(mat.ToDoubles());
        //        send_item(null);
        //    }
        //    else if(target is double[] joints)
        //    {
        //        send_int(1);
        //        send_array(joints);
        //        send_item(null);
        //    }
        //    else
        //    {
        //        throw new RDKException("Invalid input values"); 
        //    }

        //    send_item(itemrobot);
        //    check_status();
        //    if (blocking)
        //    {
        //        itemrobot.WaitMove();
        //    }

        //}


            
        void MoveC_private(Item target1, double[] joints1, Matrix mat_target1, Item target2, double[] joints2, Matrix mat_target2, Item itemrobot, bool blocking = true)
        {
            itemrobot.WaitMove();
            string command = "MoveC";
            send_line(command);
            send_int(3);
            if (target1 != null)
            {
                send_int(3);
                SendDoubleArray(null);
                SendItem(target1);
            }
            else if (joints1 != null)
            {
                send_int(1);
                SendDoubleArray(joints1);
                SendItem(null);
            }
            else if (mat_target1 != null && mat_target1.IsHomogeneous())
            {
                send_int(2);
                SendDoubleArray(mat_target1.ToDoubles());
                SendItem(null);
            }
            else
            {
                throw new RDKException("Invalid type of target 1");
            }
           
            if (target2 != null)
            {
                send_int(3);
                SendDoubleArray(null);
                SendItem(target2);
            }
            else if (joints2 != null)
            {
                send_int(1);
                SendDoubleArray(joints2);
                SendItem(null);
            }
            else if (mat_target2 != null && mat_target2.IsHomogeneous())
            {
                send_int(2);
                SendDoubleArray(mat_target2.ToDoubles());
                SendItem(null);
            }
            else
            {
                throw new RDKException("Invalid type of target 2");
            }
            

            SendItem(itemrobot);
            check_status();
            if (blocking)
            {
                itemrobot.WaitMove();
            }
        }
        

        
        /// <summary>
        /// 通过名称返回对象，如果没有准确的匹配，就返回最后一次接近的匹配
        /// </summary>
        /// <param name="name">Item name</param>
        /// <param name="itemtype">Filter by item type RoboDK.ITEM_TYPE_...</param>
        /// <returns></returns>
        public Item GetItem(string name, int itemtype = -1)
        {
            check_connection();
            string command;
            if (itemtype < 0)
            {
                command = "G_Item";
                send_line(command);
                send_line(name);
            }
            else
            {
                command = "G_Item2";
                send_line(command);
                send_line(name);
                send_int(itemtype);
            }
            Item item = ReceiveItem();
            check_status();
            return item;
        }

        /// <summary>
        /// 获取在roboDK中所有可用项目的名称列表，可以提供一个指定类型的值，比如只获取机器人信息
        /// </summary>
        /// <param name="filter">项目的类型</param>
        /// <returns></returns>
        public string[] GetItemListNames(int filter = -1)
        {
            check_connection();
            string command;
            if (filter < 0)
            {
                command = "G_List_Items";
                send_line(command);
            }
            else
            {
                command = "G_List_Items_Type";
                send_line(command);
                send_int(filter);
            }
            int numitems = rec_int();
            string[] listnames = new string[numitems];
            for (int i = 0; i < numitems; i++)
            {
                listnames[i] = rec_line();
            }
            check_status();
            return listnames;
        }

        /// <summary>
        /// 返回robodk当前打开的工作站中所有可用项目的项目列表（名称或指针列表）
        /// </summary>
        /// <param name="filter">ITEM_TYPE</param>
        /// <returns></returns>
        public Item[] GetItemList(int filter = -1)
        {
            check_connection();
            string command;
            if (filter < 0)
            {
                command = "G_List_Items_ptr";
                send_line(command);
            }
            else
            {
                command = "G_List_Items_Type_ptr";
                send_line(command);
                send_int(filter);
            }
            int numitems = rec_int();
            Item[] listitems = new Item[numitems];
            for (int i = 0; i < numitems; i++)
            {
                listitems[i] = ReceiveItem();
            }
            check_status();
            return listitems;
        }
        /// <summary>
        /// 通知roboDK软件复制一个对象
        /// </summary>
        /// <param name="item"></param>
        public void Copy(Item item)
        {
            check_connection();
            string command = "Copy";
            send_line(command);
            SendItem(item);
            check_status();
        }

        /// <summary>
        /// 通知roboDK软件粘贴一个对象
        /// </summary>
        /// <param name="frame"></param>
        /// <returns></returns>
        public Item Paste(Item frame)
        {
            check_connection();
            string command = "Paste";
            send_line(command);
            SendItem(frame);
            Item newItem = ReceiveItem();
            check_status();
            return newItem;
        }

        

        /// <summary>
        /// 显示一个RoboDK弹出窗口，从打开的工作站中选择一个对象。
        /// 可以指定项目类型以过滤所需的项目。 如果没有指定类型，则可以选择所有项目。
        /// </summary>
        /// <param name="message">Message to pop up</param>
        /// <param name="itemtype">optionally filter by RoboDK.ITEM_TYPE_*</param>
        /// <returns></returns>
        public Item ItemUserPick(string message = "Pick one item", int itemtype = -1)
        {
            check_connection();
            string command = "PickItem";
            send_line(command);
            send_line(message);
            send_int(itemtype);
            COM.ReceiveTimeout = 3600 * 1000;
            Item item = ReceiveItem();
            COM.ReceiveTimeout = TIMEOUT;
            check_status();
            return item;
        }

        /// <summary>
        /// 显示RoboDK窗口
        /// </summary>
        public void ShowRoboDK()
        {
            check_connection();
            string command = "RAISE";
            send_line(command);
            check_status();
        }

        /// <summary>
        /// 隐藏RoboDK窗口
        /// </summary>
        public void HideRoboDK()
        {
            check_connection();
            string command = "HIDE";
            send_line(command);
            check_status();
        }

        /// <summary>
        /// 关闭RoboDK窗口并完成RoboDK执行
        /// </summary>
        public void CloseRoboDK()
        {
            check_connection();
            string command = "QUIT";
            send_line(command);
            check_status();
            COM.Disconnect(false);
        }



        /// <summary>
        /// 设置RoboDK窗口的窗台，最小化，最大化等
        /// </summary>
        /// <param name="windowstate"></param>
        public void SetWindowState(int windowstate = WINDOWSTATE_NORMAL)
        {
            check_connection();
            string command = "S_WindowState";
            send_line(command);
            send_int(windowstate);
            check_status();
        }


        /// <summary>
        /// 显示消息
        /// </summary>
        /// <param name="message"></param>
        public void ShowMessage(string message)
        {
            check_connection();
            string command = "ShowMessage";
            send_line(command);
            send_line(message);
            COM.ReceiveTimeout = 3600 * 1000;
            check_status();
            COM.ReceiveTimeout = TIMEOUT;
        }
        

        /// <summary>
        /// 加载一个绝对路径的文件并将其附加到父级。 它可以是robodk支持的任何文件
        /// </summary>
        /// <param name="filename">absolute path of the file</param>
        /// <param name="parent">parent to attach. Leave empty for new stations or to load an object at the station root</param>
        /// <returns>Newly added object. Check with item.Valid() for a successful load</returns>
        public Item AddFile(string filename, Item parent = null)
        {
            check_connection();
            string command = "Add";
            send_line(command);
            send_line(filename);
            SendItem(parent);
            Item newitem = ReceiveItem();
            check_status();
            return newitem;
        }
        

        /// <summary>
        /// 将项目保存到文件。 如果没有提供项目，则保存已经打开的工作站
        /// </summary>
        /// <param name="filename">absolute path to save the file</param>
        /// <param name="itemsave">object or station to save. Leave empty to automatically save the current station.</param>
        public void Save(string filename, Item itemsave = null)
        {
            check_connection();
            string command = "Save";
            send_line(command);
            send_line(filename);
            SendItem(itemsave);
            check_status();
        }

        /// <summary>
        /// 添加曲线提供的点坐标。 所提供的点必须是顶点列表。 可以可选地提供顶点法线
        /// </summary>
        /// <param name="curve_points">matrix 3xN or 6xN -> N must be multiple of 3</param>
        /// <param name="reference_object">object to add the curve and/or project the curve to the surface</param>
        /// <param name="add_to_ref">If True, the curve will be added as part of the object in the RoboDK item tree (a reference object must be provided)</param>
        /// <param name="projection_type">Type of projection. For example: PROJECTION_ALONG_NORMAL_RECALC will project along the point normal and recalculate the normal vector on the surface projected.</param>
        /// <returns>added object/curve (null if failed)</returns>
        public Item AddCurve(Matrix curve_points, Item reference_object = null, bool add_to_ref = false, int projection_type = PROJECTION_ALONG_NORMAL_RECALC)
        {
            check_connection();
            string command = "AddWire";
            send_line(command);
            SendMatrix(curve_points);
            SendItem(reference_object);
            send_int(add_to_ref ? 1 : 0);
            send_int(projection_type);
            Item newitem = ReceiveItem();
            check_status();
            return newitem;
        }

        /// <summary>
        /// 给出一个坐标点。 提供的点必须是[XYZ]坐标的列表。 可选地，可以提供顶点法线[XYZijk]
        /// </summary>
        /// <param name="points">matrix 3xN or 6xN -> list of points to project</param>
        /// <param name="object_project">object to project</param>
        /// <param name="projection_type">Type of projection. For example: PROJECTION_ALONG_NORMAL_RECALC will project along the point normal and recalculate the normal vector on the surface projected.</param>
        /// <returns></returns>
        public Matrix ProjectPoints(Matrix points, Item object_project, int projection_type = PROJECTION_ALONG_NORMAL_RECALC)
        {
            check_connection();
            string command = "ProjectPoints";
            send_line(command);
            SendMatrix(points);
            SendItem(object_project);
            send_int(projection_type);
            Matrix projected_points = ReceiveMatrix();
            check_status();
            return projected_points;
        }

        /// <summary>
        /// 关闭当前站，没有保存建议
        /// </summary>
        public void CloseStation()
        {
            check_connection();
            string command = "Remove";
            send_line(command);
            SendItem(new Item(this));
            check_status();
        }

        /// <summary>
        /// 添加一个可以让机器人达到的新目标。
        /// </summary>
        /// <param name="name">name of the target</param>
        /// <param name="itemparent">parent to attach to (such as a frame)</param>
        /// <param name="itemrobot">main robot that will be used to go to self target</param>
        /// <returns>the new target created</returns>
        public Item AddTarget(string name, Item itemparent = null, Item itemrobot = null)
        {
            check_connection();
            string command = "Add_TARGET";
            send_line(command);
            send_line(name);
            SendItem(itemparent);
            SendItem(itemrobot);
            Item newitem = ReceiveItem();
            check_status();
            return newitem;
        }

        /// <summary>
        /// 增加一个新的参考系，可以让个机器人引用
        /// </summary>
        /// <param name="name">name of the reference frame</param>
        /// <param name="itemparent">parent to attach to (such as the robot base frame)</param>
        /// <returns>the new reference frame created</returns>
        public Item AddFrame(string name, Item itemparent = null)
        {
            check_connection();
            string command = "Add_FRAME";
            send_line(command);
            send_line(name);
            SendItem(itemparent);
            Item newitem = ReceiveItem();
            check_status();
            return newitem;
        }

        /// <summary>
        /// 增加一个新的程序，可以被机器人引用
        /// </summary>
        /// <param name="name">name of the program</param>
        /// <param name="itemrobot"></param>
        /// <returns>the new program created</returns>
        public Item AddProgram(string name, Item itemrobot = null)
        {
            check_connection();
            string command = "Add_PROG";
            send_line(command);
            send_line(name);
            SendItem(itemrobot);
            Item newitem = ReceiveItem();
            check_status();
            return newitem;
        }

        
        /// <summary>
        /// Adds a function call in the program output. RoboDK will handle the syntax when the code is generated for a specific robot. If the program exists it will also run the program in simulate mode.
        /// 在程序输出中添加一个函数调用。 当为特定机器人生成代码时，RoboDK将处理语法。 如果程序存在，它也将以模拟模式运行程序
        /// </summary>
        /// <param name="function_w_params">Function name with parameters (if any)</param>
        /// <returns></returns>
        public int RunProgram(string function_w_params)
        {
            return RunCode(function_w_params, true);
        }

        /// <summary>
        /// Adds code to run in the program output. If the program exists it will also run the program in simulate mode.
        /// </summary>
        /// <param name="code"></param>
        /// <param name="code_is_fcn_call"></param>
        /// <returns></returns>
        public int RunCode(string code, bool code_is_fcn_call = false)
        {
            check_connection();
            string command = "RunCode";
            send_line(command);
            send_int(code_is_fcn_call ? 1 : 0);
            send_line(code);
            int prog_status = rec_int();
            check_status();
            return prog_status;
        }

        /// <summary>
        /// Shows a message or a comment in the output robot program.
        /// 在输出机器人程序中显示消息或注释
        /// </summary>
        /// <param name="message"></param>
        /// <param name="message_is_comment"></param>
        public void RunMessage(string message, bool message_is_comment = false)
        {
            check_connection();
            string command = "RunMessage";
            send_line(command);
            send_int(message_is_comment ? 1 : 0);
            send_line(message);
            check_status();
        }

        /// <summary>
        /// Renders the scene. This function turns off rendering unless always_render is set to true.
        /// 渲染现场。 除非将always_render设置为true，否则此功能将关闭渲染
        /// </summary>
        /// <param name="always_render"></param>
        public void Render(bool always_render = false)
        {
            bool auto_render = !always_render;
            check_connection();
            string command = "Render";
            send_line(command);
            send_int(auto_render ? 1 : 0);
            check_status();
        }

        /// <summary>
        /// Returns the number of pairs of objects that are currently in a collision state.
        /// 返回当前处于冲突状态的对象对数
        /// </summary>
        /// <returns></returns>
        public int Collisions()
        {
            check_connection();
            string command = "Collisions";
            send_line(command);
            int ncollisions = rec_int();
            check_status();
            return ncollisions;
        }

        /// <summary>
        /// Returns 1 if item1 and item2 collided. Otherwise returns 0.
        /// </summary>
        /// <param name="item1"></param>
        /// <param name="item2"></param>
        /// <returns></returns>
        public int Collision(Item item1, Item item2)
        {
            check_connection();
            string command = "Collided";
            send_line(command);
            SendItem(item1);
            SendItem(item2);
            int ncollisions = rec_int();
            check_status();
            return ncollisions;
        }

        /// <summary>
        /// Sets the current simulation speed. Set the speed to 1 for a real-time simulation. The slowest speed allowed is 0.001 times the real speed. Set to a high value (>100) for fast simulation results.
        /// </summary>
        /// <param name="speed"></param>
        public void SetSimulationSpeed(double speed)
        {
            check_connection();
            string command = "SimulateSpeed";
            send_line(command);
            send_int((int)(speed * 1000.0));
            check_status();
        }

        /// <summary>
        /// Gets the current simulation speed. Set the speed to 1 for a real-time simulation.
        /// </summary>
        /// <returns></returns>
        public double SimulationSpeed()
        {
            check_connection();
            string command = "GetSimulateSpeed";
            send_line(command);
            double speed = ((double)rec_int()) / 1000.0;
            check_status();
            return speed;
        }
        /// <summary>
        /// Sets the behavior of the RoboDK API. By default, robodk shows the path simulation for movement instructions (run_mode=1=RUNMODE_SIMULATE).
        /// Setting the run_mode to RUNMODE_QUICKVALIDATE allows performing a quick check to see if the path is feasible.
        /// if robot.Connect() is used, RUNMODE_RUN_FROM_PC is selected automatically.
        /// </summary>
        /// <param name="run_mode">int = RUNMODE
        /// RUNMODE_SIMULATE=1        performs the simulation moving the robot (default)
        /// RUNMODE_QUICKVALIDATE=2   performs a quick check to validate the robot movements
        /// RUNMODE_MAKE_ROBOTPROG=3  makes the robot program
        /// RUNMODE_RUN_REAL=4        moves the real robot is it is connected</param>
        public void SetRunMode(int run_mode = 1)
        {
            check_connection();
            string command = "S_RunMode";
            send_line(command);
            send_int(run_mode);
            check_status();
        }

        /// <summary>
        /// Returns the behavior of the RoboDK API. By default, robodk shows the path simulation for movement instructions (run_mode=1)
        /// </summary>
        /// <returns>int = RUNMODE
        /// RUNMODE_SIMULATE=1        performs the simulation moving the robot (default)
        /// RUNMODE_QUICKVALIDATE=2   performs a quick check to validate the robot movements
        /// RUNMODE_MAKE_ROBOTPROG=3  makes the robot program
        /// RUNMODE_RUN_REAL=4        moves the real robot is it is connected</returns>
        public int RunMode()
        {
            check_connection();
            string command = "G_RunMode";
            send_line(command);
            int runmode = rec_int();
            check_status();
            return runmode;
        }

        /// <summary>
        /// Gets all the user parameters from the open RoboDK station.
        /// The parameters can also be modified by right clicking the station and selecting "shared parameters"
        /// User parameters can be added or modified by the user
        /// 从打开的RoboDK站获取所有用户参数。
        /// 也可以通过右键单击站点并选择“共享参数”来修改参数
        /// 用户参数可以由用户添加或修改
        /// </summary>
        /// <returns>list of pairs of strings as parameter-value (list of a list)</returns>
        public Dictionary<string,string> GetParams()
        {
            check_connection();
            string command = "G_Params";
            send_line(command);
            Dictionary<string, string> dict = new Dictionary<string, string>();
            int nparam = rec_int();
            for (int i = 0; i < nparam; i++)
            {
                string param = rec_line();
                string value = rec_line();
                dict.Add(param, value);
            }
            check_status();
            return dict;
        }

        /// <summary>
        /// Gets a global or a user parameter from the open RoboDK station.
        /// The parameters can also be modified by right clicking the station and selecting "shared parameters"
        /// Some available parameters:
        /// PATH_OPENSTATION = folder path of the current .stn file
        /// FILE_OPENSTATION = file path of the current .stn file
        /// PATH_DESKTOP = folder path of the user's folder
        /// Other parameters can be added or modified by the user
        /// </summary>
        /// <param name="param">RoboDK parameter</param>
        /// <returns>value</returns>
        public string GetParam(string param)
        {
            check_connection();
            string command = "G_Param";
            send_line(command);
            send_line(param);
            string value = rec_line();
            if (value.StartsWith("UNKNOWN "))
            {
                value = null;
            }
            check_status();
            return value;
        }

        /// <summary>
        /// Sets a global parameter from the RoboDK station. If the parameters exists, it will be modified. If not, it will be added to the station.
        /// The parameters can also be modified by right clicking the station and selecting "shared parameters"
        /// </summary>
        /// <param name="param">RoboDK parameter</param>
        /// <param name="value">value</param>
        /// <returns></returns>
        public void SetParam(string param, string value)
        {
            check_connection();
            string command = "S_Param";
            send_line(command);
            send_line(param);
            send_line(value);
            check_status();
        }

        /// <summary>
        /// Returns the current joints of a list of robots.
        /// </summary>
        /// <param name="robot_item_list">list of robot items</param>
        /// <returns>list of robot joints (double x nDOF)</returns>
        public double[][] Joints(Item[] robot_item_list)
        {
            check_connection();
            string command = "G_ThetasList";
            send_line(command);
            int nrobs = robot_item_list.Length;
            send_int(nrobs);
            double[][] joints_list = new double[nrobs][];
            for (int i = 0; i < nrobs; i++)
            {
                SendItem(robot_item_list[i]);
                joints_list[i] = ReceiveArray();
            }
            check_status();
            return joints_list;
        }

        /// <summary>
        /// Sets the current robot joints for a list of robot items and a list of a set of joints.
        /// </summary>
        /// <param name="robot_item_list">list of robot items</param>
        /// <param name="joints_list">list of robot joints (double x nDOF)</param>
        public void SetJoints(Item[] robot_item_list, double[][] joints_list)
        {
            int nrobs = Math.Min(robot_item_list.Length, joints_list.Length);
            check_connection();
            string command = "S_ThetasList";
            send_line(command);
            send_int(nrobs);
            for (int i = 0; i < nrobs; i++)
            {
                SendItem(robot_item_list[i]);
                SendDoubleArray(joints_list[i]);
            }
            check_status();
        }

        /// <summary>
        /// Defines the name of the program when the program is generated. It is also possible to specify the name of the post processor as well as the folder to save the program. 
        /// This method must be called before any program output is generated (before any robot movement or other instruction).
        /// </summary>
        /// <param name="progname">name of the program</param>
        /// <param name="defaultfolder">folder to save the program, leave empty to use the default program folder</param>
        /// <param name="postprocessor">name of the post processor (for a post processor in C:/RoboDK/Posts/Fanuc_post.py it is possible to provide "Fanuc_post.py" or simply "Fanuc_post")</param>
        /// <param name="robot">Robot to link</param>
        /// <returns></returns>
        public int ProgramStart(string progname, string defaultfolder = "", string postprocessor = "", Item robot = null)
        {
            check_connection();
            string command = "ProgramStart";
            send_line(command);
            send_line(progname);
            send_line(defaultfolder);
            send_line(postprocessor);
            SendItem(robot);
            int errors = rec_int();
            check_status();
            return errors;
        }

        /// <summary>
        /// Set the pose of the wold reference frame with respect to the view (camera/screen)
        /// </summary>
        /// <param name="pose"></param>
        public void SetViewPose(Matrix pose)
        {
            check_connection();
            string command = "S_ViewPose";
            send_line(command);
            SendPose(pose);
            check_status();
        }

        /// <summary>
        /// Get the pose of the wold reference frame with respect to the view (camera/screen)
        /// </summary>
        public Matrix ViewPose()
        {
            check_connection();
            string command = "G_ViewPose";
            send_line(command);
            Matrix pose = ReceivePose();
            check_status();
            return pose;
        }


        /// <summary>
        /// Item类代表RoboDK站中的一个项目。 项目可以是机器人，参考系，工具，对象，目标，...在站点树中可见的任何项目。
        /// 一个项目也可以被看作是可以附加其他项目（子项目）的节点。
        /// 每个项目都有一个父项/节点，并且可以有一个或多个子项/节点RoboLinkItem是RoboLink的“朋友”类。
        /// </summary>
        public class Item
        {
            private UInt64 item = 0;
            private RoboDK link; // pointer to the RoboLink connection
            int type = -1;
            string name;

            /// <summary>
            /// 实例化一个RoboDK的项目对象
            /// </summary>
            /// <param name="connection_link"></param>
            /// <param name="item_ptr"></param>
            /// <param name="itemtype"></param>
            public Item(RoboDK connection_link, UInt64 item_ptr = 0, int itemtype = -1)
            {
                item = item_ptr;
                link = connection_link;
                type = itemtype;
            }

            /// <summary>
            /// 该对象的唯一值
            /// </summary>
            /// <returns></returns>
            public UInt64 GetItemPtr()
            {
                return item;
            }

            /// <summary>
            /// 返回该对象的字符串标识形式。
            /// </summary>
            /// <returns></returns>
            public override string ToString()
            {
                if (Valid())
                {
                    return String.Format("RoboDK item {0} of type {1}", item, type);
                }
                else
                {
                    return "RoboDK item (INVALID)";
                }
            }

            /// <summary>
            /// 返回两个对象是否为同一个对象，根据在roboDK中的序列号来判定
            /// </summary>
            /// <param name="item_other"></param>
            /// <returns></returns>
            public bool Equals(Item item_other)
            {
                return item == item_other.item;
            }

            /// <summary>
            /// Returns the RoboDK link Robolink().
            /// </summary>
            /// <returns></returns>
            public RoboDK RoboLink()
            {
                return link;
            }
            
            /// <summary>
            /// Returns the type of an item (robot, object, target, reference frame, ...)
            /// </summary>
            /// <returns></returns>
            public int Type()
            {
                link.check_connection();
                string command = "G_Item_Type";
                link.send_line(command);
                link.SendItem(this);
                int itemtype = link.rec_int();
                link.check_status();
                return itemtype;
            }
            

            /// <summary>
            /// 将工作站或是对象保存到文件
            /// </summary>
            /// <param name="filename"></param>
            public void Save(string filename)
            {
                link.Save(filename, this);
            }

            /// <summary>
            /// 从工作站中删除项目和它的所有的子节点
            /// </summary>
            public void Delete()
            {
                link.check_connection();
                string command = "Remove";
                link.send_line(command);
                link.SendItem(this);
                link.check_status();
                item = 0;
            }

            /// <summary>
            /// Checks if the item is valid. An invalid item will be returned by an unsuccessful function call.
            /// 检查本项目是否是有效的，如果项目无效，将会调用失败
            /// </summary>
            /// <returns>true if valid, false if invalid</returns>
            public bool Valid()
            {
                if (item == 0)
                {
                    return false;
                }
                return true;
            }

            ////// add more methods
            /// <summary>
            /// 返回附加到提供的项目的项目子项的列表
            /// </summary>
            /// <returns>item x n -> list of child items</returns>
            public Item[] Childs()
            {
                link.check_connection();
                string command = "G_Childs";
                link.send_line(command);
                link.SendItem(this);
                int nitems = link.rec_int();
                Item[] itemlist = new Item[nitems];
                for (int i = 0; i < nitems; i++)
                {
                    itemlist[i] = link.ReceiveItem();
                }
                link.check_status();
                return itemlist;
            }

            /// <summary>
            /// 获取项目是否可见，可见返回1，否则返回0
            /// </summary>
            /// <returns>true if visible, false if not visible</returns>
            public bool Visible()
            {
                link.check_connection();
                string command = "G_Visible";
                link.send_line(command);
                link.SendItem(this);
                int visible = link.rec_int();
                link.check_status();
                return (visible != 0);
            }
            /// <summary>
            /// 设置项目的可见状态
            /// </summary>
            /// <param name="visible">项目是否可见</param>
            /// <param name="visible_frame">项目的自己的坐标系是否可见1可见，0不可见</param>
            public void SetVisible(bool visible, int visible_frame = -1)
            {
                if (visible_frame < 0)
                {
                    visible_frame = visible ? 1 : 0;
                }
                link.check_connection();
                string command = "S_Visible";
                link.send_line(command);
                link.SendItem(this);
                link.send_int(visible ? 1 : 0);
                link.send_int(visible_frame);
                link.check_status();
            }

            /// <summary>
            /// 返回本项目的名称，该名称将是显示在RoboDK工作站的中树节点
            /// </summary>
            /// <returns>name of the item</returns>
            public string Name()
            {
                link.check_connection();
                string command = "G_Name";
                link.send_line(command);
                link.SendItem(this);
                name = link.rec_line();
                link.check_status();
                return name;
            }

            /// <summary>
            /// 复制一个项目
            /// </summary>
            public void Copy()
            {
                link.Copy(this);
            }


            /// <summary>
            /// 判断是否和另一个项目碰撞
            /// </summary>
            /// <param name="item"></param>
            /// <returns></returns>
            public int Collision(Item item)
            {
                link.check_connection();
                string command = "Collided";
                link.send_line(command);
                link.SendItem(this);
                link.SendItem(item);
                int ncollisions = link.rec_int();
                link.check_status();
                return ncollisions;
            }

            /// <summary>
            /// 粘贴
            /// </summary>
            /// <returns></returns>
            public Item Paste()
            {
                Item item = link.Paste(this);
                return item;
            }

            /// <summary>
            /// 设置一个新的父参考系，根据相对位置来设定
            /// </summary>
            /// <param name="parent"></param>
            public void SetParent(Item parent)
            {
                link.check_connection();
                string command = "S_Parent";
                link.send_line(command);
                link.SendItem(this);
                link.SendItem(parent);
                link.check_status();
            }

            /// <summary>
            /// 设置一个新的父参考室，但世界位置不变
            /// </summary>
            /// <param name="parent"></param>
            public void SetParentStatic(Item parent)
            {
                link.check_connection();
                string command = "S_Parent_Static";
                link.send_line(command);
                link.SendItem(this);
                link.SendItem(parent);
                link.check_status();
            }


            /// <summary>
            /// 抓取最近的物品
            /// </summary>
            /// <returns></returns>
            public Item AttachClosest()
            {
                link.check_connection();
                string command = "Attach_Closest";
                link.send_line(command);
                link.SendItem(this);
                Item item = link.ReceiveItem();
                link.check_status();
                return item;
            }


            /// <summary>
            /// 放开所有的物品
            /// </summary>
            /// <param name="parent"></param>
            public void DetachAll(Item parent = null)
            {
                link.check_connection();
                string command = "Detach_All";
                link.send_line(command);
                link.SendItem(this);
                link.SendItem(parent);
                link.check_status();
            }

            /// <summary>
            /// 放开最近的物品
            /// </summary>
            /// <param name="parent"></param>
            /// <returns></returns>
            public Item DetachCloset(Item parent = null)
            {
                link.check_connection();
                string command = "Detach_Closest";
                link.send_line(command);
                link.SendItem(this);
                link.SendItem(parent);
                Item item = link.ReceiveItem();
                link.check_status();
                return item;
            }

            /// <summary>
            /// Set the name of a RoboDK item.
            /// 设置本项目的名称
            /// </summary>
            /// <param name="name"></param>
            public void SetName(string name)
            {
                link.check_connection();
                string command = "S_Name";
                link.send_line(command);
                link.SendItem(this);
                link.send_line(name);
                link.check_status();
            }

            // add more methods

            /// <summary>
            /// Sets the local position (pose) of an object, target or reference frame. For example, the position of an object/frame/target with respect to its parent.
            /// If a robot is provided, it will set the pose of the end efector.
            /// 设置对象，目标或参照帧的本地位置（姿态）。 例如，对象/框架/目标对象的位置。
            /// 如果提供机器人，它将设置终端efector的姿态。
            /// </summary>
            /// <param name="pose">4x4 homogeneous matrix</param>
            public void SetPose(Matrix pose)
            {
                link.check_connection();
                string command = "S_Hlocal";
                link.send_line(command);
                link.SendItem(this);
                link.SendPose(pose);
                link.check_status();
            }

            /// <summary>
            /// Returns the local position (pose) of an object, target or reference frame. For example, the position of an object/frame/target with respect to its parent.
            /// If a robot is provided, it will get the pose of the end efector
            /// </summary>
            /// <returns>4x4 homogeneous matrix (pose)</returns>
            public Matrix Pose()
            {
                link.check_connection();
                string command = "G_Hlocal";
                link.send_line(command);
                link.SendItem(this);
                Matrix pose = link.ReceivePose();
                link.check_status();
                return pose;
            }

            /// <summary>
            /// Sets the position (pose) the object geometry with respect to its own reference frame. This procedure works for tools and objects.
            /// 相对于其自己的参考帧设置对象几何的位置（姿态）。 此过程适用于工具和对象。
            /// </summary>
            /// <param name="pose">4x4 homogeneous matrix</param>
            public void SetGeometryPose(Matrix pose)
            {
                link.check_connection();
                string command = "S_Hgeom";
                link.send_line(command);
                link.SendItem(this);
                link.SendPose(pose);
                link.check_status();
            }

            /// <summary>
            /// Returns the position (pose) the object geometry with respect to its own reference frame. This procedure works for tools and objects.
            /// </summary>
            /// <returns>4x4 homogeneous matrix (pose)</returns>
            public Matrix GeometryPose()
            {
                link.check_connection();
                string command = "G_Hgeom";
                link.send_line(command);
                link.SendItem(this);
                Matrix pose = link.ReceivePose();
                link.check_status();
                return pose;
            }
            
            /// <summary>
            /// Returns the tool pose of an item. If a robot is provided it will get the tool pose of the active tool held by the robot.
            /// 返回本项目的工具姿势。 如果提供机器人，它将获得由机器人保持的主动工具的工具姿势。
            /// </summary>
            /// <returns>4x4 homogeneous matrix (pose)</returns>
            public Matrix PoseTool()
            {
                link.check_connection();
                string command = "G_Tool";
                link.send_line(command);
                link.SendItem(this);
                Matrix pose = link.ReceivePose();
                link.check_status();
                return pose;
            }

            /// <summary>
            /// Returns the reference frame pose of an item. If a robot is provided it will get the tool pose of the active reference frame used by the robot.
            /// 返回项目的参考系姿态。 如果项目是机器人，它将获得机器人使用的活动工具姿势
            /// </summary>
            /// <returns>4x4 homogeneous matrix (pose)</returns>
            public Matrix PoseFrame()
            {
                link.check_connection();
                string command = "G_Frame";
                link.send_line(command);
                link.SendItem(this);
                Matrix pose = link.ReceivePose();
                link.check_status();
                return pose;
            }

            /// <summary>
            /// Sets the reference frame of a robot(user frame). The frame can be either an item or a pose.
            /// If "frame" is an item, it links the robot to the frame item. If frame is a pose, it updates the linked pose of the robot frame (with respect to the robot reference frame).
            /// </summary>
            /// <param name="frame_pose">4x4 homogeneous matrix (pose)</param>
            public void SetPoseFrame(Matrix frame_pose)
            {
                link.check_connection();
                string command = "S_Frame";
                link.send_line(command);
                link.SendPose(frame_pose);
                link.SendItem(this);
                link.check_status();
            }

            /// <summary>
            /// Sets the tool of a robot or a tool object (Tool Center Point, or TCP). The tool pose can be either an item or a 4x4 Matrix.
            /// If the item is a tool, it links the robot to the tool item.If tool is a pose, it updates the current robot TCP.
            /// 设置机器人的工具的方法，如果是工具，就连接到机器人，如果是姿态，则更新TCP的姿态
            /// </summary>
            /// <param name="frame_item">4x4 homogeneous matrix (pose)</param>
            public void SetPoseFrame(Item frame_item)
            {
                link.check_connection();
                string command = "S_Frame_ptr";
                link.send_line(command);
                link.SendItem(frame_item);
                link.SendItem(this);
                link.check_status();
            }

            /// <summary>
            /// Sets the tool of a robot or a tool object (Tool Center Point, or TCP). The tool pose can be either an item or a 4x4 Matrix.
            /// If the item is a tool, it links the robot to the tool item.If tool is a pose, it updates the current robot TCP.
            /// 设置机器人或工具对象（工具中心点或TCP）的工具。 工具姿势可以是一个项目或一个4x4矩阵。
            /// 如果项目是一个工具，它将机器人链接到工具项。如果工具是一个姿势，它会更新当前的机器人TCP的姿态。
            /// </summary>
            /// <param name="tool_pose">4x4 homogeneous matrix (pose)</param>
            public void SetPoseTool(Matrix tool_pose)
            {
                link.check_connection();
                string command = "S_Tool";
                link.send_line(command);
                link.SendPose(tool_pose);
                link.SendItem(this);
                link.check_status();
            }

            /// <summary>
            /// Sets the tool of a robot or a tool object (Tool Center Point, or TCP). The tool pose can be either an item or a 4x4 Matrix.
            /// If the item is a tool, it links the robot to the tool item.If tool is a pose, it updates the current robot TCP.
            /// </summary>
            /// <param name="tool_item">Tool item</param>
            public void SetPoseTool(RoboDK.Item tool_item)
            {
                link.check_connection();
                string command = "S_Tool_ptr";
                link.send_line(command);
                link.SendItem(tool_item);
                link.SendItem(this);
                link.check_status();
            }

            /// <summary>
            /// Sets the global position (pose) of an item. For example, the position of an object/frame/target with respect to the station origin.
            /// 设置绝对姿态，该姿态是相对于工作站的绝对坐标系而言的
            /// </summary>
            /// <param name="pose">4x4 homogeneous matrix (pose)</param>
            public void SetPoseAbs(Matrix pose)
            {
                link.check_connection();
                string command = "S_Hlocal_Abs";
                link.send_line(command);
                link.SendItem(this);
                link.SendPose(pose);
                link.check_status();

            }

            /// <summary>
            /// Returns the global position (pose) of an item. For example, the position of an object/frame/target with respect to the station origin.
            /// 获取相对于工作站绝对位置的坐标信息
            /// </summary>
            /// <returns>4x4 homogeneous matrix (pose)</returns>
            public Matrix PoseAbs()
            {
                link.check_connection();
                string command = "G_Hlocal_Abs";
                link.send_line(command);
                link.SendItem(this);
                Matrix pose = link.ReceivePose();
                link.check_status();
                return pose;
            }

            /// <summary>
            /// Changes the color of a robot/object/tool. A color must must in the format COLOR=[R,G,B,(A=1)] where all values range from 0 to 1.
            /// Alpha (A) defaults to 1 (100% opaque). Set A to 0 to make an object transparent.
            /// </summary>
            /// <param name="tocolor">color to change to</param>
            /// <param name="fromcolor">filter by this color</param>
            /// <param name="tolerance">optional tolerance to use if a color filter is used (defaults to 0.1)</param>
            public void Recolor(double[] tocolor, double[] fromcolor = null, double tolerance = 0.1)
            {
                link.check_connection();
                if (fromcolor == null)
                {
                    fromcolor = new double[] { 0, 0, 0, 0 };
                    tolerance = 2;
                }
                link.check_color(tocolor);
                link.check_color(fromcolor);
                string command = "Recolor";
                link.send_line(command);
                link.SendItem(this);
                double[] combined = new double[9];
                combined[0] = tolerance;
                Array.Copy(fromcolor, 0, combined, 1, 4);
                Array.Copy(tocolor, 0, combined, 5, 4);
                link.SendDoubleArray(combined);
                link.check_status();
            }

            /// <summary>
            /// Apply a scale to an object to make it bigger or smaller.
            /// The scale can be uniform (if scale is a float value) or per axis (if scale is a vector).
            /// 对一个对象应用一个缩放比例，使它变大或变小。 
            /// 刻度可以是均匀的（如果刻度是浮点值）或每个轴（如果刻度是矢量）。
            /// </summary>
            /// <param name="scale">scale to apply as [scale_x, scale_y, scale_z]</param>
            public void Scale(double[] scale)
            {
                link.check_connection();
                if (scale.Length != 3)
                {
                    throw new RDKException("scale must be a single value or a 3-vector value");
                }
                string command = "Scale";
                link.send_line(command);
                link.SendItem(this);
                link.SendDoubleArray(scale);
                link.check_status();
            }

            /// <summary>
            /// Adds a curve provided point coordinates. The provided points must be a list of vertices. A vertex normal can be provided optionally.
            /// 添加曲线提供的点坐标。 所提供的点必须是顶点列表。 可以可选地提供顶点法线
            /// </summary>
            /// <param name="curve_points">matrix 3xN or 6xN -> N must be multiple of 3</param>
            /// <param name="add_to_ref">add_to_ref -> If True, the curve will be added as part of the object in the RoboDK item tree</param>
            /// <param name="projection_type">Type of projection. For example: PROJECTION_ALONG_NORMAL_RECALC will project along the point normal and recalculate the normal vector on the surface projected.</param>
            /// <returns>returns the object where the curve was added or null if failed</returns>
            public Item AddCurve(Matrix curve_points, bool add_to_ref = false, int projection_type = PROJECTION_ALONG_NORMAL_RECALC)
            {
                return link.AddCurve(curve_points, this, add_to_ref, projection_type);
            }

            /// <summary>
            /// Projects a point to the object given its coordinates. The provided points must be a list of [XYZ] coordinates. Optionally, a vertex normal can be provided [XYZijk].
            /// </summary>
            /// <param name="points">matrix 3xN or 6xN -> list of points to project</param>
            /// <param name="projection_type">projection_type -> Type of projection. For example: PROJECTION_ALONG_NORMAL_RECALC will project along the point normal and recalculate the normal vector on the surface projected.</param>
            /// <returns>projected points (empty matrix if failed)</returns>
            public Matrix ProjectPoints(Matrix points, int projection_type = PROJECTION_ALONG_NORMAL_RECALC)
            {
                return link.ProjectPoints(points, this, projection_type);
            }
            

            /// <summary>
            /// 将目标设置为一个笛卡尔目标，笛卡尔目标移动到笛卡尔坐标。
            /// </summary>
            public void SetAsCartesianTarget()
            {
                link.check_connection();
                string command = "S_Target_As_RT";
                link.send_line(command);
                link.SendItem(this);
                link.check_status();
            }

            /// <summary>
            /// Sets a target as a joint target. A joint target moves to a joints position without regarding the cartesian coordinates.
            /// </summary>
            public void SetAsJointTarget()
            {
                link.check_connection();
                string command = "S_Target_As_JT";
                link.send_line(command);
                link.SendItem(this);
                link.check_status();
            }

            //#####Robot item calls####

            /// <summary>
            /// Returns the current joints of a robot or the joints of a target. If the item is a cartesian target, it returns the preferred joints (configuration) to go to that cartesian position.
            /// </summary>
            /// <returns>double x n -> joints matrix</returns>
            public double[] Joints()
            {
                link.check_connection();
                string command = "G_Thetas";
                link.send_line(command);
                link.SendItem(this);
                double[] joints = link.ReceiveArray();
                link.check_status();
                return joints;
            }

            // add more methods

            /// <summary>
            /// Returns the home joints of a robot. These joints can be manually set in the robot "Parameters" menu, then select "Set home position"
            /// </summary>
            /// <returns>double x n -> joints array</returns>
            public double[] JointsHome()
            {
                link.check_connection();
                string command = "G_Home";
                link.send_line(command);
                link.SendItem(this);
                double[] joints = link.ReceiveArray();
                link.check_status();
                return joints;
            }

            /// <summary>
            /// Sets the current joints of a robot or the joints of a target. It the item is a cartesian target, it returns the preferred joints (configuration) to go to that cartesian position.
            /// </summary>
            /// <param name="joints"></param>
            public void SetJoints(double[] joints)
            {
                link.check_connection();
                string command = "S_Thetas";
                link.send_line(command);
                link.SendDoubleArray(joints);
                link.SendItem(this);
                link.check_status();
            }

            /// <summary>
            /// Returns the joint limits of a robot
            /// </summary>
            /// <param name="lower_limits"></param>
            /// <param name="upper_limits"></param>
            public void JointLimits(double[] lower_limits, double[] upper_limits)
            {
                link.check_connection();
                string command = "G_RobLimits";
                link.send_line(command);
                link.SendItem(this);
                lower_limits = link.ReceiveArray();
                upper_limits = link.ReceiveArray();
                double joints_type = link.rec_int() / 1000.0;
                link.check_status();
            }

            /// <summary>
            /// Sets the robot of a program or a target. You must set the robot linked to a program or a target every time you copy paste these objects.
            /// If the robot is not provided, the first available robot will be chosen automatically.
            /// </summary>
            /// <param name="robot">Robot item</param>
            public void SetRobot(Item robot = null)
            {
                link.check_connection();
                string command = "S_Robot";
                link.send_line(command);
                link.SendItem(this);
                link.SendItem(robot);
                link.check_status();
            }

            /// <summary>
            /// Adds an empty tool to the robot provided the tool pose (4x4 Matrix) and the tool name.
            /// </summary>
            /// <param name="tool_pose">pose -> TCP as a 4x4 Matrix (pose of the tool frame)</param>
            /// <param name="tool_name">New tool name</param>
            /// <returns>new item created</returns>
            public Item AddTool(Matrix tool_pose, string tool_name = "New TCP")
            {
                link.check_connection();
                string command = "AddToolEmpty";
                link.send_line(command);
                link.SendItem(this);
                link.SendPose(tool_pose);
                link.send_line(tool_name);
                Item newtool = link.ReceiveItem();
                link.check_status();
                return newtool;
            }

            /// <summary>
            /// Computes the forward kinematics of the robot for the provided joints. The tool and the reference frame are not taken into account.
            /// </summary>
            /// <param name="joints"></param>
            /// <returns>4x4 homogeneous matrix: pose of the robot flange with respect to the robot base</returns>
            public Matrix SolveFK(double[] joints)
            {
                link.check_connection();
                string command = "G_FK";
                link.send_line(command);
                link.SendDoubleArray(joints);
                link.SendItem(this);
                Matrix pose = link.ReceivePose();
                link.check_status();
                return pose;
            }

            /// <summary>
            /// Returns the robot configuration state for a set of robot joints.
            /// </summary>
            /// <param name="joints">array of joints</param>
            /// <returns>3-array -> configuration status as [REAR, LOWERARM, FLIP]</returns>
            public double[] JointsConfig(double[] joints)
            {
                link.check_connection();
                string command = "G_Thetas_Config";
                link.send_line(command);
                link.SendDoubleArray(joints);
                link.SendItem(this);
                double[] config = link.ReceiveArray();
                link.check_status();
                return config;
            }

            /// <summary>
            /// Computes the inverse kinematics for the specified robot and pose. The joints returned are the closest to the current robot configuration (see SolveIK_All())
            /// </summary>
            /// <param name="pose">4x4 matrix -> pose of the robot flange with respect to the robot base frame</param>
            /// <returns>array of joints</returns>
            public double[] SolveIK(Matrix pose)
            {
                link.check_connection();
                string command = "G_IK";
                link.send_line(command);
                link.SendPose(pose);
                link.SendItem(this);
                double[] joints = link.ReceiveArray();
                link.check_status();
                return joints;
            }

            /// <summary>
            /// Computes the inverse kinematics for the specified robot and pose. The function returns all available joint solutions as a 2D matrix.
            /// </summary>
            /// <param name="pose">4x4 matrix -> pose of the robot tool with respect to the robot frame</param>
            /// <returns>double x n x m -> joint list (2D matrix)</returns>
            public Matrix SolveIK_All(Matrix pose)
            {
                link.check_connection();
                string command = "G_IK_cmpl";
                link.send_line(command);
                link.SendPose(pose);
                link.SendItem(this);
                Matrix joints_list = link.ReceiveMatrix();
                link.check_status();
                return joints_list;
            }

            /// <summary>
            /// 使用robot驱动连接到真实的机器人
            /// </summary>
            /// <param name="robot_ip">IP of the robot to connect. Leave empty to use the one defined in RoboDK</param>
            /// <returns>status -> true if connected successfully, false if connection failed</returns>
            public bool Connect(string robot_ip = "")
            {
                link.check_connection();
                string command = "Connect";
                link.send_line(command);
                link.SendItem(this);
                link.send_line(robot_ip);
                int status = link.rec_int();
                link.check_status();
                return status != 0;
            }

            /// <summary>
            /// 断开和真实机器人的连接（当机器人驱动使用的时候）
            /// </summary>
            /// <returns>status -> true if disconnected successfully, false if it failed. It can fail if it was previously disconnected manually for example.</returns>
            public bool Disconnect()
            {
                link.check_connection();
                string command = "Disconnect";
                link.send_line(command);
                link.SendItem(this);
                int status = link.rec_int();
                link.check_status();
                return status != 0;
            }

            /// <summary>
            /// 将机器人的工具移动到目标点位，默认的情况下，这个方法会知道机器人移动到指定位置再返回
            /// </summary>
            /// <param name="itemtarget">target -> target to move to as a target item (RoboDK target item)</param>
            /// <param name="blocking">blocking -> True if we want the instruction to block until the robot finished the movement (default=true)</param>
            public void MoveJ(Item itemtarget, bool blocking = true)
            {
                link.MoveX(itemtarget, null, null, this, 1, blocking);
            }

            /// <summary>
            /// 将机器人移动到指定的关节点，默认的情况下，这个方法会知道机器人移动到指定位置再返回
            /// </summary>
            /// <param name="joints">joints -> joint target to move to.</param>
            /// <param name="blocking">blocking -> True if we want the instruction to block until the robot finished the movement (default=true)</param>
            public void MoveJ(double[] joints, bool blocking = true)
            {
                link.MoveX(null, joints, null, this, 1, blocking);
            }

            /// <summary>
            /// 将机器人的工具移动到目标点位，默认的情况下，这个方法会知道机器人移动到指定位置再返回
            /// </summary>
            /// <param name="target">pose -> pose target to move to. It must be a 4x4 Homogeneous matrix</param>
            /// <param name="blocking">blocking -> True if we want the instruction to block until the robot finished the movement (default=true)</param>
            public void MoveJ(Matrix target, bool blocking = true)
            {
                link.MoveX(null, null, target, this, 1, blocking);
            }

            /// <summary>
            /// Moves a robot to a specific target ("Move Linear" mode). By default, this function blocks until the robot finishes its movements.
            /// </summary>
            /// <param name="itemtarget">target -> target to move to as a target item (RoboDK target item)</param>
            /// <param name="blocking">blocking -> True if we want the instruction to block until the robot finished the movement (default=true)</param>
            public void MoveL(Item itemtarget, bool blocking = true)
            {
                link.MoveX(itemtarget, null, null, this, 2, blocking);
            }

            /// <summary>
            /// Moves a robot to a specific target ("Move Linear" mode). By default, this function blocks until the robot finishes its movements.
            /// </summary>
            /// <param name="joints">joints -> joint target to move to.</param>
            /// <param name="blocking">blocking -> True if we want the instruction to block until the robot finished the movement (default=true)</param>
            public void MoveL(double[] joints, bool blocking = true)
            {
                link.MoveX(null, joints, null, this, 2, blocking);
            }

            /// <summary>
            /// Moves a robot to a specific target ("Move Linear" mode). By default, this function blocks until the robot finishes its movements.
            /// </summary>
            /// <param name="target">pose -> pose target to move to. It must be a 4x4 Homogeneous matrix</param>
            /// <param name="blocking">blocking -> True if we want the instruction to block until the robot finished the movement (default=true)</param>
            public void MoveL(Matrix target, bool blocking = true)
            {
                link.MoveX(null, null, target, this, 2, blocking);
            }

            /// <summary>
            /// Moves a robot to a specific target ("Move Circular" mode). By default, this function blocks until the robot finishes its movements.
            /// </summary>
            /// <param name="itemtarget1">target -> intermediate target to move to as a target item (RoboDK target item)</param>
            /// <param name="itemtarget2">target -> final target to move to as a target item (RoboDK target item)</param>
            /// <param name="blocking">blocking -> True if we want the instruction to block until the robot finished the movement (default=true)</param>
            public void MoveC(Item itemtarget1, Item itemtarget2, bool blocking = true)
            {
                link.MoveC_private(itemtarget1, null, null, itemtarget2, null, null, this, blocking);
            }

            /// <summary>
            /// Moves a robot to a specific target ("Move Circular" mode). By default, this function blocks until the robot finishes its movements.
            /// </summary>
            /// <param name="joints1">joints -> intermediate joint target to move to.</param>
            /// <param name="joints2">joints -> final joint target to move to.</param>
            /// <param name="blocking">blocking -> True if we want the instruction to block until the robot finished the movement (default=true)</param>
            public void MoveC(double[] joints1, double[] joints2, bool blocking = true)
            {
                link.MoveC_private(null, joints1, null, null, joints2, null, this, blocking);
            }

            /// <summary>
            /// Moves a robot to a specific target ("Move Circular" mode). By default, this function blocks until the robot finishes its movements.
            /// </summary>
            /// <param name="target1">pose -> intermediate pose target to move to. It must be a 4x4 Homogeneous matrix</param>
            /// <param name="target2">pose -> final pose target to move to. It must be a 4x4 Homogeneous matrix</param>
            /// <param name="blocking">blocking -> True if we want the instruction to block until the robot finished the movement (default=true)</param>
            public void MoveC(Matrix target1, Matrix target2, bool blocking = true)
            {
                link.MoveC_private(null, null, target1, null, null, target2, this, blocking);
            }

            /// <summary>
            /// Checks if a joint movement is free of collision.
            /// </summary>
            /// <param name="j1">joints -> start joints</param>
            /// <param name="j2">joints -> destination joints</param>
            /// <param name="minstep_deg">(optional): maximum joint step in degrees</param>
            /// <returns>collision : returns 0 if the movement is free of collision. Otherwise it returns the number of pairs of objects that collided if there was a collision.</returns>
            public int MoveJ_Collision(double[] j1, double[] j2, double minstep_deg = -1)
            {
                link.check_connection();
                string command = "CollisionMove";
                link.send_line(command);
                link.SendItem(this);
                link.SendDoubleArray(j1);
                link.SendDoubleArray(j2);
                link.send_int((int)(minstep_deg * 1000.0));
                int collision = link.rec_int();
                link.check_status();
                return collision;
            }

            /// <summary>
            /// Checks if a linear movement is free of collision.
            /// </summary>
            /// <param name="j1">joints -> start joints</param>
            /// <param name="j2">joints -> destination joints</param>
            /// <param name="minstep_deg">(optional): maximum joint step in degrees</param>
            /// <returns>collision : returns 0 if the movement is free of collision. Otherwise it returns the number of pairs of objects that collided if there was a collision.</returns>
            public int MoveL_Collision(double[] j1, double[] j2, double minstep_deg = -1)
            {
                link.check_connection();
                string command = "CollisionMoveL";
                link.send_line(command);
                link.SendItem(this);
                link.SendDoubleArray(j1);
                link.SendDoubleArray(j2);
                link.send_int((int)(minstep_deg * 1000.0));
                int collision = link.rec_int();
                link.check_status();
                return collision;
            }
            

            /// <summary>
            /// Sets the speed and/or the acceleration of a robot.
            /// </summary>
            /// <param name="speed_linear">linear speed in mm/s (-1 = no change)</param>
            /// <param name="accel_linear">linear acceleration in mm/s2 (-1 = no change)</param>
            /// <param name="speed_joints">joint speed in deg/s (-1 = no change)</param>
            /// <param name="accel_joints">joint acceleration in deg/s2 (-1 = no change)</param>
            public void SetSpeed(double speed_linear, double accel_linear = -1, double speed_joints = -1, double accel_joints = -1)
            {
                link.check_connection();
                string command = "S_Speed4";
                link.send_line(command);
                link.SendItem(this);
                double[] speed_accel = new double[4];
                speed_accel[0] = speed_linear;
                speed_accel[1] = accel_linear;
                speed_accel[2] = speed_joints;
                speed_accel[3] = accel_joints;
                link.SendDoubleArray(speed_accel);
                link.check_status();

            }

            /// <summary>
            /// Sets the robot movement smoothing accuracy (also known as zone data value).
            /// </summary>
            /// <param name="zonedata">zonedata value (int) (robot dependent, set to -1 for fine movements)</param>
            public void SetZoneData(double zonedata)
            {
                link.check_connection();
                string command = "S_ZoneData";
                link.send_line(command);
                link.send_int((int)(zonedata * 1000.0));
                link.SendItem(this);
                link.check_status();
            }

            /// <summary>
            /// Displays a sequence of joints
            /// </summary>
            /// <param name="sequence">joint sequence as a 6xN matrix or instruction sequence as a 7xN matrix</param>
            public void ShowSequence(Matrix sequence)
            {
                link.check_connection();
                string command = "Show_Seq";
                link.send_line(command);
                link.SendMatrix(sequence);
                link.SendItem(this);
                link.check_status();
            }


            /// <summary>
            /// Checks if a robot or program is currently running (busy or moving)
            /// </summary>
            /// <returns>busy status (true=moving, false=stopped)</returns>
            public bool Busy()
            {
                link.check_connection();
                string command = "IsBusy";
                link.send_line(command);
                link.SendItem(this);
                int busy = link.rec_int();
                link.check_status();
                return (busy > 0);
            }

            /// <summary>
            /// 停止运行一段程序或是机器人的动作
            /// </summary>
            /// <returns></returns>
            public void Stop()
            {
                link.check_connection();
                string command = "Stop";
                link.send_line(command);
                link.SendItem(this);
                link.check_status();
            }

            /// <summary>
            /// 等待机器人完成它的动作才返回方法（阻塞的方式）
            /// </summary>
            /// <param name="timeout_sec">timeout -> Max time to wait for robot to finish its movement (in seconds)</param>
            public void WaitMove(double timeout_sec = 300)
            {
                link.check_connection();
                string command = "WaitMove";
                link.send_line(command);
                link.SendItem(this);
                link.check_status();
                // link.COM.ReceiveTimeout = (int)(timeout_sec * 1000.0);
                link.check_status();
            }

            /// <summary>
            /// 将程序保存到一个文件
            /// </summary>
            /// <param name="filename">File path of the program</param>
            /// <returns>success</returns>
            public bool MakeProgram(string filename)
            {
                link.check_connection();
                string command = "MakeProg";
                link.send_line(command);
                link.SendItem(this);
                link.send_line(filename);
                int prog_status = link.rec_int();
                string prog_log_str = link.rec_line();
                link.check_status();
                bool success = false;
                if (prog_status > 1)
                {
                    success = true;
                }
                return success; 
            }

            /// <summary>
            /// Sets if the program will be run in simulation mode or on the real robot.
            /// Use: "PROGRAM_RUN_ON_SIMULATOR" to set the program to run on the simulator only or "PROGRAM_RUN_ON_ROBOT" to force the program to run on the robot.
            /// </summary>
            /// <returns>number of instructions that can be executed</returns>
            public void SetRunType(int program_run_type)
            {
                link.check_connection();
                string command = "S_ProgRunType";
                link.send_line(command);
                link.SendItem(this);
                link.send_int(program_run_type);
                link.check_status();
            }

            /// <summary>
            /// Runs a program. It returns the number of instructions that can be executed successfully (a quick program check is performed before the program starts)
            /// This is a non-blocking call. Use IsBusy() to check if the program execution finished.
            /// Notes:
            /// if setRunMode(RUNMODE_SIMULATE) is used  -> the program will be simulated (default run mode)
            /// if setRunMode(RUNMODE_RUN_ROBOT) is used -> the program will run on the robot (default when RUNMODE_RUN_ROBOT is used)
            /// if setRunMode(RUNMODE_RUN_ROBOT) is used together with program.setRunType(PROGRAM_RUN_ON_ROBOT) -> the program will run sequentially on the robot the same way as if we right clicked the program and selected "Run on robot" in the RoboDK GUI        
            /// </summary>
            /// <returns>number of instructions that can be executed</returns>
            public int RunProgram()
            {
                link.check_connection();
                string command = "RunProg";
                link.send_line(command);
                link.SendItem(this);
                int prog_status = link.rec_int();
                link.check_status();
                return prog_status;
            }


            /// <summary>
            /// Runs a program. It returns the number of instructions that can be executed successfully (a quick program check is performed before the program starts)
            /// Program parameters can be provided for Python calls.
            /// This is a non-blocking call.Use IsBusy() to check if the program execution finished.
            /// Notes: if setRunMode(RUNMODE_SIMULATE) is used  -> the program will be simulated (default run mode)
            /// if setRunMode(RUNMODE_RUN_ROBOT) is used ->the program will run on the robot(default when RUNMODE_RUN_ROBOT is used)
            /// if setRunMode(RUNMODE_RUN_ROBOT) is used together with program.setRunType(PROGRAM_RUN_ON_ROBOT) -> the program will run sequentially on the robot the same way as if we right clicked the program and selected "Run on robot" in the RoboDK GUI
            /// </summary>
            /// <param name="parameters">Number of instructions that can be executed</param>
            public int RunCode(string parameters = null)
            {
                link.check_connection();
                if (parameters == null)
                {
                    string command = "RunProg";
                    link.send_line(command);
                    link.SendItem(this);
                }
                else
                {
                    string command = "RunProgParam";
                    link.send_line(command);
                    link.SendItem(this);
                    link.send_line(parameters);
                }
                int progstatus = link.rec_int();
                link.check_status();
                return progstatus;
            }

            /// <summary>
            /// Adds a program call, code, message or comment inside a program.
            /// </summary>
            /// <param name="code">string of the code or program to run</param>
            /// <param name="run_type">INSTRUCTION_* variable to specify if the code is a progra</param>
            public int RunCodeCustom(string code, int run_type = INSTRUCTION_CALL_PROGRAM)
            {
                link.check_connection();
                string command = "RunCode2";
                link.send_line(command);
                link.SendItem(this);
                link.send_line(code.Replace("\n\n", "<br>").Replace("\n", "<br>"));
                link.send_int(run_type);
                int progstatus = link.rec_int();
                link.check_status();
                return progstatus;
            }

            /// <summary>
            /// Generates a pause instruction for a robot or a program when generating code. Set it to -1 (default) if you want the robot to stop and let the user resume the program anytime.
            /// </summary>
            /// <param name="time_ms">Time in milliseconds</param>
            public void Pause(double time_ms = -1)
            {
                link.check_connection();
                string command = "RunPause";
                link.send_line(command);
                link.SendItem(this);
                link.send_int((int)(time_ms * 1000.0));
                link.check_status();
            }


            /// <summary>
            /// Sets a variable (output) to a given value. This can also be used to set any variables to a desired value.
            /// </summary>
            /// <param name="io_var">io_var -> digital output (string or number)</param>
            /// <param name="io_value">io_value -> value (string or number)</param>
            public void SetDO(string io_var, string io_value)
            {
                link.check_connection();
                string command = "setDO";
                link.send_line(command);
                link.SendItem(this);
                link.send_line(io_var);
                link.send_line(io_value);
                link.check_status();
            }

            /// <summary>
            /// Waits for an input io_id to attain a given value io_value. Optionally, a timeout can be provided.
            /// </summary>
            /// <param name="io_var">io_var -> digital output (string or number)</param>
            /// <param name="io_value">io_value -> value (string or number)</param>
            /// <param name="timeout_ms">int (optional) -> timeout in miliseconds</param>
            public void WaitDI(string io_var, string io_value, double timeout_ms = -1)
            {
                link.check_connection();
                string command = "waitDI";
                link.send_line(command);
                link.SendItem(this);
                link.send_line(io_var);
                link.send_line(io_value);
                link.send_int((int)(timeout_ms * 1000.0));
                link.check_status();
            }



            /// <summary>
            /// Adds a new robot move joint instruction to a program.
            /// </summary>
            /// <param name="itemtarget">target to move to</param>
            public void AddMoveJ(Item itemtarget)
            {
                link.check_connection();
                string command = "Add_INSMOVE";
                link.send_line(command);
                link.SendItem(itemtarget);
                link.SendItem(this);
                link.send_int(1);
                link.check_status();
            }

            /// <summary>
            /// Adds a new robot move linear instruction to a program.
            /// </summary>
            /// <param name="itemtarget">target to move to</param>
            public void AddMoveL(Item itemtarget)
            {
                link.check_connection();
                string command = "Add_INSMOVE";
                link.send_line(command);
                link.SendItem(itemtarget);
                link.SendItem(this);
                link.send_int(2);
                link.check_status();
            }
            

            /// <summary>
            /// Returns the number of instructions of a program.
            /// </summary>
            /// <returns></returns>
            public int InstructionCount()
            {
                link.check_connection();
                string command = "Prog_Nins";
                link.send_line(command);
                link.SendItem(this);
                int nins = link.rec_int();
                link.check_status();
                return nins;
            }

            /// <summary>
            /// Returns the program instruction at position id
            /// </summary>
            /// <param name="ins_id"></param>
            /// <param name="name"></param>
            /// <param name="instype"></param>
            /// <param name="movetype"></param>
            /// <param name="isjointtarget"></param>
            /// <param name="target"></param>
            /// <param name="joints"></param>
            public void Instruction(int ins_id, out string name, out int instype, out int movetype, out bool isjointtarget, out Matrix target, out double[] joints)
            {
                link.check_connection();
                string command = "Prog_GIns";
                link.send_line(command);
                link.SendItem(this);
                link.send_int(ins_id);
                name = link.rec_line();
                instype = link.rec_int();
                movetype = 0;
                isjointtarget = false;
                target = null;
                joints = null;
                if (instype == INS_TYPE_MOVE)
                {
                    movetype = link.rec_int();
                    isjointtarget = link.rec_int() > 0 ? true : false;
                    target = link.ReceivePose();
                    joints = link.ReceiveArray();
                }
                link.check_status();
            }


            /// <summary>
            /// Sets the program instruction at position id
            /// </summary>
            /// <param name="ins_id"></param>
            /// <param name="name"></param>
            /// <param name="instype"></param>
            /// <param name="movetype"></param>
            /// <param name="isjointtarget"></param>
            /// <param name="target"></param>
            /// <param name="joints"></param>
            public void SetInstruction(int ins_id, string name, int instype, int movetype, bool isjointtarget, Matrix target, double[] joints)
            {
                link.check_connection();
                string command = "Prog_SIns";
                link.send_line(command);
                link.SendItem(this);
                link.send_int(ins_id);
                link.send_line(name);
                link.send_int(instype);
                if (instype == INS_TYPE_MOVE)
                {
                    link.send_int(movetype);
                    link.send_int(isjointtarget ? 1 : 0);
                    link.SendPose(target);
                    link.SendDoubleArray(joints);
                }
                link.check_status();
            }


            /// <summary>
            /// Returns the list of program instructions as an MxN matrix, where N is the number of instructions and M equals to 1 plus the number of robot axes.
            /// </summary>
            /// <param name="instructions">the matrix of instructions</param>
            /// <returns>Returns 0 if success</returns>
            public int InstructionList(out Matrix instructions)
            {
                link.check_connection();
                string command = "G_ProgInsList";
                link.send_line(command);
                link.SendItem(this);
                instructions = link.ReceiveMatrix();
                int errors = link.rec_int();
                link.check_status();
                return errors;
            }

            /// <summary>
            /// Returns a list of joints an MxN matrix, where M is the number of robot axes plus 4 columns. Linear moves are rounded according to the smoothing parameter set inside the program.
            /// </summary>
            /// <param name="error_msg">Returns a human readable error message (if any)</param>
            /// <param name="joint_list">Returns the list of joints as [J1, J2, ..., Jn, ERROR, MM_STEP, DEG_STEP, MOVE_ID] if a file name is not specified</param>
            /// <param name="mm_step">Maximum step in millimeters for linear movements (millimeters)</param>
            /// <param name="deg_step">Maximum step for joint movements (degrees)</param>
            /// <param name="save_to_file">Provide a file name to directly save the output to a file. If the file name is not provided it will return the matrix. If step values are very small, the returned matrix can be very large.</param>
            /// <returns>Returns 0 if success, otherwise, it will return negative values</returns>
            public int InstructionListJoints(out string error_msg, out Matrix joint_list, double mm_step = 10.0, double deg_step = 5.0, string save_to_file = "")
            {
                link.check_connection();
                string command = "G_ProgJointList";
                link.send_line(command);
                link.SendItem(this);
                double[] ste_mm_deg = { mm_step, deg_step };
                link.SendDoubleArray(ste_mm_deg);
                //joint_list = save_to_file;
                if (save_to_file.Length <= 0)
                {
                    link.send_line("");
                    joint_list = link.ReceiveMatrix();
                }
                else
                {
                    link.send_line(save_to_file);
                    joint_list = null;
                }
                int error_code = link.rec_int();
                error_msg = link.rec_line();
                link.check_status();
                return error_code;
            }
            
        }

    }

}
