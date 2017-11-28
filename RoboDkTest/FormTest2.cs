using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using RoboDKHelper;

namespace RoboDkTest
{
    public partial class FormTest2 : Form
    {
        public FormTest2()
        {
            InitializeComponent();
        }


        RoboDK roboDK = new RoboDK("192.168.0.110");
        Random random = new Random();


        private void button1_Click(object sender, EventArgs e)
        {
            RoboDK.Item item = roboDK.GetItem("part1");
            item.SetColor(new double[] { random.NextDouble(), random.NextDouble(), random.NextDouble(), 1 });
        }



    }
}
