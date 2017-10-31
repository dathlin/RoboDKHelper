using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using RoboDKHelper;
using System.Threading;
using HslCommunication.LogNet;

namespace RoboDkTest
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
        }

        private RoboDK roboDK;
        private void Form1_Load(object sender, EventArgs e)
        {
            roboDK = new RoboDK("127.0.0.1");
        }
        private void Form1_FormClosing(object sender, FormClosingEventArgs e)
        {
            roboDK.Disconnect();
        }


        private void button1_Click(object sender, EventArgs e)
        {
            RoboDK.Item part = roboDK.GetItem("Part 1");
            part.SetPose(part.Pose().Translate(-100, 0, 0));
        }

        private void button3_Click(object sender, EventArgs e)
        {
            // y平移测试
            roboDK.GetItem("Part 1").Copy();
            RoboDK.Item part = roboDK.GetItem("PalletA").Paste();
            part.SetName("Part 1000");
            part.Scale(new double[] { 2, 2, 2 });
            part.SetPose(part.Pose().Translate(0, 0, 400));
        }
        private void button11_Click(object sender, EventArgs e)
        {
            RoboDK.Item robot = roboDK.GetItem("UR10 A", RoboDK.ITEM_TYPE_ROBOT);
            RoboDK.Item taget = roboDK.GetItem("Put Conveyor", RoboDK.ITEM_TYPE_TARGET);
            robot.MoveJ(taget);
        }

        private void button13_Click(object sender, EventArgs e)
        {
            RoboDK.Item robot = roboDK.GetItem("UR10 A", RoboDK.ITEM_TYPE_ROBOT);
            RoboDK.Item taget = roboDK.GetItem("PalletApproachA", RoboDK.ITEM_TYPE_TARGET);
            robot.MoveJ(taget);
        }
        private void button12_Click(object sender, EventArgs e)
        {
            // 启动皮带
            RoboDK.Item program = roboDK.GetItem("MoveConveyor");
            program.RunCode();
        }

        private void button14_Click(object sender, EventArgs e)
        {
            // 停止皮带
            RoboDK.Item program = roboDK.GetItem("MoveConveyor");
            program.Stop();
        }
        private void button9_Click(object sender, EventArgs e)
        {
            PrepareSimulation();
        }
        private void PrepareSimulation()
        {
            RoboDK RDK = new RoboDK("127.0.0.1");
            RoboDK.Item frame_pallet = RDK.GetItem("PalletA", RoboDK.ITEM_TYPE_FRAME);

            // 获取参数
            string SizeBox = RDK.GetParam("SizeBox"); // 150,200,150  箱子大小
            string SizePallet = RDK.GetParam("SizePallet"); // 5,3,3  横纵列

            float[] SizeBoxXyz = SizeBox.Split(',').Select((m) => float.Parse(m.Trim())).ToArray(); // [150,200,150]
            int[] SizePalletXyz = SizePallet.Split(',').Select((m) => int.Parse(m.Trim())).ToArray(); // [5,3,3]

            float SizeBoxZ = SizeBoxXyz[2]; // 箱子的高度很重要，用来计算位置和碰撞检测的


            RDK.Render(false);

            CleanUp(RDK.GetItemList(RoboDK.ITEM_TYPE_OBJECT), "Part ");
            CleanUp(RDK.GetItemList(RoboDK.ITEM_TYPE_TOOL), "TCP ");

            RDK.GetItem("box100mm").Copy();
            Random random = new Random();

            for (int i = 0; i < SizePalletXyz[0]; i++)
            {
                for (int j = 0; j < SizePalletXyz[1]; j++)
                {
                    for (int k = 0; k < SizePalletXyz[2]; k++)
                    {
                        // 计算位置
                        float location_x = SizeBoxXyz[0] / 2 + i * SizeBoxXyz[0];
                        float location_y = SizeBoxXyz[1] / 2 + j * SizeBoxXyz[1];
                        float location_z = SizeBoxXyz[2] / 2 + k * SizeBoxXyz[2];

                        // 复制新的对象
                        RoboDK.Item newPart = frame_pallet.Paste();
                        newPart.Scale(new double[] { SizeBoxXyz[0] / 100, SizeBoxXyz[1] / 100, SizeBoxXyz[2] / 100 });
                        newPart.SetName("Part " + i.ToString() + j.ToString() + k.ToString());
                        newPart.SetPose(Matrix.Transl(location_x, location_y, location_z));
                        newPart.SetVisible(true, 0);
                        newPart.Recolor(new double[] { random.NextDouble(), random.NextDouble(), random.NextDouble(), 1 });
                    }
                }
            }

            
            RDK.SetParam("SENSOR", "0");
            RDK.Render(true);
            RDK.Disconnect();

            PartsClear();
        }

        private void CleanUp(RoboDK.Item[] items,string startWith)
        {
            foreach(var m in items)
            {
                if(m.Name().StartsWith(startWith))
                {
                    m.Delete();
                }
            }
        }

        #region 传送带物品队列


        private List<RoboDK.Item> ConveyorParts = new List<RoboDK.Item>();
        private object parts_lock = new object();
        private void AddPart(RoboDK.Item item)
        {
            lock(parts_lock)
            {
                ConveyorParts.Add(item);
            }
        }
        private RoboDK.Item GetPartAndRemove()
        {
            lock(parts_lock)
            {
                if (ConveyorParts.Count > 0)
                {
                    RoboDK.Item item = ConveyorParts[0];
                    ConveyorParts.RemoveAt(0);
                    return item;
                }
                else
                {
                    return null;
                }
            }
        }
        private RoboDK.Item GetEndedPart()
        {
            lock(parts_lock)
            {
                if (ConveyorParts.Count > 0)
                {
                    RoboDK.Item item = ConveyorParts[0];
                    return item;
                }
                else
                {
                    return null;
                }
            }
        }

        private RoboDK.Item GetStartedPart()
        {
            lock(parts_lock)
            {
                if (ConveyorParts.Count > 0)
                {
                    RoboDK.Item item = ConveyorParts[ConveyorParts.Count - 1];
                    return item;
                }
                else
                {
                    return null;
                }
            }
        }


        private void PartsClear()
        {
            lock(parts_lock)
            {
                ConveyorParts.Clear();
            }
        }
        
        #endregion

        private void button10_Click(object sender, EventArgs e)
        {
            ThreadPool.QueueUserWorkItem(RobotAGetParts, null);
            ThreadPool.QueueUserWorkItem(SensorActive, null);
            ThreadPool.QueueUserWorkItem(RobotBStoreParts, null);
        }

        private void RobotAGetParts(object obj)
        {
            RoboDK RDK = new RoboDK("127.0.0.1");

            int APPROACH = 100;

            RoboDK.Item robot = RDK.GetItem("UR10 A", RoboDK.ITEM_TYPE_ROBOT);
            RoboDK.Item tool = RDK.GetItem("GripperA", RoboDK.ITEM_TYPE_TOOL);
            RoboDK.Item frame_pallet = RDK.GetItem("PalletA", RoboDK.ITEM_TYPE_FRAME);
            RoboDK.Item frame_conv = RDK.GetItem("ConveyorReference", RoboDK.ITEM_TYPE_FRAME);
            RoboDK.Item frame_conv_moving = RDK.GetItem("MovingRef", RoboDK.ITEM_TYPE_FRAME);

            // 获取目标点
            RoboDK.Item target_pallet_safe = RDK.GetItem("PalletApproachA", RoboDK.ITEM_TYPE_TARGET);
            RoboDK.Item target_conv_safe = RDK.GetItem("ConvApproachA", RoboDK.ITEM_TYPE_TARGET);
            RoboDK.Item target_conv = RDK.GetItem("Put Conveyor", RoboDK.ITEM_TYPE_TARGET);

            // 获取参数
            string SizeBox = RDK.GetParam("SizeBox"); // 150,200,150  箱子大小
            string SizePallet = RDK.GetParam("SizePallet"); // 5,3,3  横纵列


            
            //RoboDK.Item SENSOR2 = RDK.GetItem("Sensor SICK WL4S2", RoboDK.ITEM_TYPE_OBJECT);
            //RoboDK.Item SENSOR3 = RDK.GetItem("Sensor SICK WL4S3", RoboDK.ITEM_TYPE_OBJECT);


            float[] SizeBoxXyz = SizeBox.Split(',').Select((m) => float.Parse(m.Trim())).ToArray(); // [150,200,150]
            int[] SizePalletXyz = SizePallet.Split(',').Select((m) => int.Parse(m.Trim())).ToArray(); // [5,3,3]

            float SizeBoxZ = SizeBoxXyz[2]; // 箱子的高度很重要，用来计算位置和碰撞检测的


            Matrix tool_xyzrpw = tool.PoseTool().Translate(0, 0, SizeBoxZ / 2);
            RoboDK.Item tool_tcp = robot.AddTool(tool_xyzrpw, "TCP A");
            robot.SetPoseTool(tool_tcp);

            for (int k = SizePalletXyz[2] - 1; k >= 0; k--)
            {
                for (int i = SizePalletXyz[0] - 1; i >= 0; i--)
                {
                    for (int j = SizePalletXyz[1] - 1; j >= 0; j--)
                    {
                        // 计算位置
                        float location_x = SizeBoxXyz[0] / 2 + i * SizeBoxXyz[0];
                        float location_y = SizeBoxXyz[1] / 2 + j * SizeBoxXyz[1];
                        float location_z = SizeBoxXyz[2] / 2 + k * SizeBoxXyz[2];
                        


                        // 从传送带抓取
                        robot.SetPoseFrame(frame_pallet);
                        robot.MoveJ(target_pallet_safe);        // 移动到上方的一个安全点
                        

                        Matrix target_i = Matrix.Transl(location_x, location_y, location_z).RotX(Math.PI);

                        
                        robot.MoveJ(target_i.Translate(0, 0, -APPROACH - SizeBoxZ));


                        robot.MoveJ(target_i);
                        

                        RoboDK.Item item = RDK.GetItem("Part " + i.ToString() + j.ToString() + k.ToString());
                        item.SetParentStatic(tool);
                        

                        robot.MoveJ(target_i.Translate(0, 0, -APPROACH - SizeBoxZ));
                        robot.MoveJ(target_pallet_safe);
                    

                        // 放置到传送带上去
                        robot.SetPoseFrame(frame_conv);





                        Matrix target_conv_pose = target_conv.Pose().Translate(0, 0, -SizeBoxZ / 2);
                        Matrix target_conv_app = target_conv_pose.Translate(0, 0, -APPROACH);
                        robot.MoveJ(target_conv_safe);

                        // 移动到上方之前，需要先判断是否有物品阻挡
                        //RoboDK.Item itemStarted = GetStartedPart();
                        //if (itemStarted != null)
                        //{
                        //    while (true)
                        //    {
                        //        if (SENSOR2.Collision(itemStarted) > 0 ||
                        //            SENSOR3.Collision(itemStarted) > 0)
                        //        {
                        //            continue;
                        //        }
                        //        else
                        //        {
                        //            break;
                        //        }
                        //    }
                        //}


                        robot.MoveJ(target_conv_app);
                        robot.MoveL(target_conv_pose);

                        AddPart(item);
                        item.SetParentStatic(frame_conv_moving);
                        

                        robot.MoveL(target_conv_app);
                        robot.MoveJ(target_conv_safe);
                        
                    }
                }
            }

        }

        private void ConvertorControl(object obj)
        {
            RoboDK RDK = new RoboDK("127.0.0.1");
            RoboDK.Item Convertor = RDK.GetItem("Conveyor Belt");


        }


        private void SensorActive(object obj)
        {
            RoboDK RDK = new RoboDK("127.0.0.1");
            
            RoboDK.Item SENSOR = RDK.GetItem("Sensor SICK WL4S", RoboDK.ITEM_TYPE_OBJECT);
            RoboDK.Item program = roboDK.GetItem("MoveConveyor");
            program.RunCode();
            
            while(true)
            {
                RoboDK.Item item = GetEndedPart();
                if (item != null)
                {
                    if (SENSOR.Collision(item) > 0)
                    {
                        Console.WriteLine("检测到碰撞");

                        program.Stop();


                        RDK.SetParam("SENSOR", "1");

                        // 等待东西拿走再继续
                        while(RDK.GetParam("SENSOR")=="1")
                        {
                            Thread.Sleep(10);
                        }
                        program.RunCode();
                    }
                }
                Thread.Sleep(20);
            }
        }





        private void RobotBStoreParts(object obj)
        {
            RoboDK RDK = new RoboDK("127.0.0.1");

            int APPROACH = 100;

            RoboDK.Item robot = RDK.GetItem("UR10 B", RoboDK.ITEM_TYPE_ROBOT);
            RoboDK.Item tool = RDK.GetItem("GripperB", RoboDK.ITEM_TYPE_TOOL);
            RoboDK.Item frame_pallet = RDK.GetItem("PalletB", RoboDK.ITEM_TYPE_FRAME);
            RoboDK.Item frame_conv = RDK.GetItem("ConveyorReference", RoboDK.ITEM_TYPE_FRAME);
            RoboDK.Item frame_conv_moving = RDK.GetItem("MovingRef", RoboDK.ITEM_TYPE_FRAME);

            // 获取目标点
            RoboDK.Item target_pallet_safe = RDK.GetItem("PalletApproachB", RoboDK.ITEM_TYPE_TARGET);
            RoboDK.Item target_conv_safe = RDK.GetItem("ConvApproachB", RoboDK.ITEM_TYPE_TARGET);
            RoboDK.Item target_conv = RDK.GetItem("Get Conveyor", RoboDK.ITEM_TYPE_TARGET);

            // 获取参数
            string SizeBox = RDK.GetParam("SizeBox"); // 150,200,150  箱子大小
            string SizePallet = RDK.GetParam("SizePallet"); // 5,3,3  横纵列

            float[] SizeBoxXyz = SizeBox.Split(',').Select((m) => float.Parse(m.Trim())).ToArray(); // [150,200,150]
            int[] SizePalletXyz = SizePallet.Split(',').Select((m) => int.Parse(m.Trim())).ToArray(); // [5,3,3]

            float SizeBoxZ = SizeBoxXyz[2]; // 箱子的高度很重要，用来计算位置和碰撞检测的


            Matrix tool_xyzrpw = tool.PoseTool().Translate(0, 0, SizeBoxZ / 2);
            RoboDK.Item tool_tcp = robot.AddTool(tool_xyzrpw, "TCP B");

            robot.SetPoseTool(tool_tcp);

            for (int k = 0; k < SizePalletXyz[2]; k++)
            {
                for (int j = 0; j < SizePalletXyz[1]; j++)
                {
                    for (int i = 0; i < SizePalletXyz[0]; i++)
                    {


                        // 计算位置
                        float location_x = SizeBoxXyz[0] / 2 + i * SizeBoxXyz[0];
                        float location_y = SizeBoxXyz[1] / 2 + j * SizeBoxXyz[1];
                        float location_z = SizeBoxXyz[2] / 2 + k * SizeBoxXyz[2];



                        robot.SetPoseFrame(frame_conv);
                        Matrix target_conv_pose = target_conv.Pose().Translate(0, 0, -SizeBoxZ / 2);
                        Matrix target_conv_app = target_conv_pose.Translate(0, 0, -APPROACH);
                        robot.MoveJ(target_conv_safe);
                        robot.MoveJ(target_conv_app);

                        // 等待传感器信号
                        while (RDK.GetParam("SENSOR") == "0")
                        {
                            Thread.Sleep(10);
                        }
                        Thread.Sleep(10);

                        //robot.MoveL(target_conv_pose);

                        
                        RoboDK.Item item = new RoboDK.Item(RDK, GetPartAndRemove().GetItemPtr());
                        
                        robot.MoveL(target_conv_pose);
                        item.SetParentStatic(tool);
                        

                        
                        RDK.SetParam("SENSOR", "0");
                        Matrix matOffect = item.Pose();

                        robot.MoveL(target_conv_app);
                        robot.MoveJ(target_conv_safe);

                        robot.SetPoseFrame(frame_pallet);

                        robot.MoveJ(target_pallet_safe);

                        Matrix target_i = Matrix.Transl(location_x, location_y, location_z).RotX(Math.PI);
                        robot.MoveJ(target_i.Translate(0, 0, -APPROACH - SizeBoxZ));


                        robot.MoveJ(target_i);

                        item.SetParentStatic(frame_pallet);


                        robot.MoveJ(target_i.Translate(0, 0, -APPROACH - SizeBoxZ));
                        robot.MoveJ(target_pallet_safe);
                    }
                }
            }
        }


        #region 日志记录块

        private ILogNet LogNet = new LogNetSingle(Application.StartupPath + @"\Logs\roboDK.log.txt");


        #endregion


    }
}
