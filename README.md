# RoboDKHelper ![Build status](https://ci.appveyor.com/api/projects/status/oajkgccisoe98gip?svg=true) [![NuGet Status](https://img.shields.io/nuget/v/RoboDKHelper.svg)](https://www.nuget.org/packages/RoboDKHelper/) [![Gitter](https://badges.gitter.im/Join%20Chat.svg)](http://shang.qq.com/wpa/qunwpa?idkey=2278cb9c2e0c04fc305c43e41acff940499a34007dfca9e83a7291e726f9c4e8)

## 版权声明
本组件版权归Richard.Hu所有

## 授权协议
使用请遵循MIT协议说明，允许修改源代码并集成到自己的开源和闭源项目上，仅需要出具一份本组件的MIT的License即可。

## 免责声明
使用本组件产生的任何后果和损失，作者概不负责。

## NuGet安装
说明：NuGet为稳定版本，组件的使用必须从NuGet下载，此处发布的项目有可能为还没有通过编译的测试版，NuGet安装如下：

Install-Package RoboDKHelper

## 联系作者
* 技术支持QQ群：[592132877](http://shang.qq.com/wpa/qunwpa?idkey=2278cb9c2e0c04fc305c43e41acff940499a34007dfca9e83a7291e726f9c4e8)
* 邮箱地址：hsl200909@163.com

## 项目目标
本项目的目标在于开发一个C#版本的roboDK软件通信库，并提供持续的维护。

## 项目介绍


## 特别说明
本组件基于RoboDK软件的Python程序和官方的部分C#库进行修改而来，主要是添加了注释，删除了一些不常用的方法，优化了底层通信。

## 代码贡献
热烈欢迎对本项目的代码提出改进意见，可以发起Pull Request。

## 特别感谢
* 无

## 示例代码
<pre>
<code>
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

        }
</code>
</pre>