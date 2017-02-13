using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Drawing.Configuration;
using System.Drawing.Imaging;
using System.Linq;
using System.Media;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

using Yeti2015.RobotService;
using Yeti2015.Hardware.Lidar;

namespace Yeti2015.DesktopApp
{
    public partial class Form1 : Form
    {
        private Thread consoleThread;
        private Thread renderThread;
        private Thread renderLandmarkThread;
        private Thread renderObstacleThread;

        private BindingList<double[]> debugBindingList;

        public delegate void AddGridRow(double[] items);
        public AddGridRow addGridRowDelegate;
        private bool drawGUI;
        private bool updateList;

        public Form1()
        {
            InitializeComponent();
            DoubleBuffered = true;

            RobotService.Program.NewLidarData += Program_NewLidarData;
            RobotService.Program.NewLocationPoint += Program_NewLocationPoint;
            RobotService.Program.NewLandmarkData += Program_NewLandmarkData;
            RobotService.Program.NewObstacleData += Program_NewObstacleData;
            RobotService.Program.StallDetected += Program_StallDetected;

            debugBindingList = new BindingList<double[]>();
            //dataGridViewDebug.DataSource = debugBindingList;

            addGridRowDelegate = new AddGridRow(AddGridRowMethod);

            drawGUI = checkBoxUpdateGUI.Checked;
            updateList = checkBoxUpdateList.Checked;

            consoleThread = new Thread(() => RobotService.Program.Main(new string[0]));
            consoleThread.Name = "Console Thread";
            consoleThread.Start();
        }

        void Program_StallDetected(object sender, EventArgs e)
        {
            //SystemSounds.Exclamation.Play();
        }

        void Program_NewLandmarkData(object sender, EventArgs e)
        {
            if (drawGUI)
            {
                renderLandmarkThread = new Thread(() => RenderLandmarkPoints(sender));
                renderLandmarkThread.Name = "Render Landmark Thread";
                renderLandmarkThread.Start();
            }
        }

        void Program_NewObstacleData(object sender, EventArgs e)
        {
            if (drawGUI)
            {
                renderObstacleThread = new Thread(() => RenderObstaclePoints(sender));
                renderObstacleThread.Name = "Render Obstacle Thread";
                renderObstacleThread.Start();
            }
        }

        void Program_NewLocationPoint(object sender, EventArgs e)
        {
            if (drawGUI)
            {
                //var itemList = double[](sender);

                //debugBindingList.Add(itemList);

                if (updateList)
                {
                    var itemList = (double[])sender;
                    this.Invoke(this.addGridRowDelegate, itemList);
                }
            }
        }

        void Program_NewLidarData(object sender, EventArgs e)
        {
            if (drawGUI)
            {
                renderThread = new Thread(() => RenderLidarPoints(sender));
                renderThread.Name = "Render Thread";
                renderThread.Start();
            }
        }

        private void RenderObstaclePoints(object sender)
        {
            var pictureGraphics = pictureBox1.CreateGraphics();
            var obstaclePoints = (List<Obstacle>)sender;

            if (obstaclePoints.Count > 0)
            {
                var YetiLocation = RobotService.Program.YetiLocation;

                pictureGraphics.ScaleTransform(pictureBox1.ClientSize.Width / 2000F, pictureBox1.ClientSize.Height / 2000F);
                pictureGraphics.TranslateTransform(1000F, 1000F);
                pictureGraphics.RotateTransform((float)(-YetiLocation.Heading * (180 / Math.PI)));
                pictureGraphics.TranslateTransform(-((float)YetiLocation.X * 100F), ((float)YetiLocation.Y * 100F));

                pictureGraphics.FillRectangles(Brushes.Blue, obstaclePoints.Select(p => new Rectangle((int)(p.X * 100), (int)(-p.Y * 100), 10, 10)).ToArray());
                //pictureGraphics.FillRectangles(Brushes.Red, obstaclePoints.Where(p => (p.moving == true)).Select(p => new Rectangle((int)(p.X * 100), (int)(-p.Y * 100), (int)(p.line_size_of_obs * 100), 5)).ToArray());
                var movingObstacles = (from p in obstaclePoints where p.moving == true select new Rectangle((int)(p.X * 100), (int)(-p.Y * 100), (int)(p.line_size_of_obs * 100), 10)).ToArray();
                if (movingObstacles.Count() > 0) { pictureGraphics.FillRectangles(Brushes.Red, movingObstacles); }
            }
        }

        private void RenderLidarPoints(object sender)
        {
            var pictureGraphics = pictureBox1.CreateGraphics();
            pictureGraphics.Clear(Color.White);

            pictureGraphics.ScaleTransform(pictureBox1.ClientSize.Width / 2000F, pictureBox1.ClientSize.Height / 2000F);
            pictureGraphics.TranslateTransform(1000F, 1000F);

            pictureGraphics.DrawRectangle(Pens.Red, -30F, 0, 60F, 80F);
            pictureGraphics.DrawEllipse(Pens.Red, -10F, -10F, 20F, 20F);
            pictureGraphics.DrawLine(Pens.Red, 0, 0, -1000, 1000);
            pictureGraphics.DrawLine(Pens.Red, 0, 0, 1000, 1000);

            var lidarPoints = ((int?[]) sender).ToLidarPoints(19000);

            pictureGraphics.FillRectangles(Brushes.Black, lidarPoints.Select(p => new Rectangle((int)(p.X * 100), (int)(-p.Y * 100), 5, 5)).ToArray());
        }

        private void RenderLandmarkPoints(object sender)
        {
            var pictureGraphics = pictureBox1.CreateGraphics();
            var landmarkPoints = (List<LocationPoint>)sender;
            if (landmarkPoints.Count > 0)
            {
                var YetiLocation = RobotService.Program.YetiLocation;
                var TargetLocation = RobotService.Program.TargetLocation;

                pictureGraphics.ScaleTransform(pictureBox1.ClientSize.Width / 2000F, pictureBox1.ClientSize.Height / 2000F);
                pictureGraphics.TranslateTransform(1000F, 1000F);
                pictureGraphics.RotateTransform((float)(-YetiLocation.Heading * (180 / Math.PI)));
                pictureGraphics.TranslateTransform(-((float)YetiLocation.X * 100F), ((float)YetiLocation.Y * 100F));

                pictureGraphics.FillRectangles(Brushes.Magenta, landmarkPoints.Select(p => new Rectangle((int)(p.X * 100), (int)(-p.Y * 100), 10, 10)).ToArray());

                pictureGraphics.DrawLine(new Pen(Color.Green, 10), -10, 0, 40, 0);
                pictureGraphics.DrawLine(new Pen(Color.Green, 10), 0, -40, 0, 10);

                pictureGraphics.DrawLine(new Pen(Color.Orange, 10), (float)(TargetLocation.location.X * 100) - 20, -(float)(TargetLocation.location.Y * 100), (float)(TargetLocation.location.X * 100) + 20, -(float)(TargetLocation.location.Y * 100));
                pictureGraphics.DrawLine(new Pen(Color.Orange, 10), (float)(TargetLocation.location.X * 100), -(float)(TargetLocation.location.Y * 100) - 20, (float)(TargetLocation.location.X * 100), -(float)(TargetLocation.location.Y * 100) + 20);
            }
        }

        private void AddGridRowMethod(double[] items)
        {
            dataGridViewDebug.Rows.Add(items[0], items[1], items[2], items[3], items[4], items[5], items[6], items[7], items[8], items[9]);
            dataGridViewDebug.FirstDisplayedScrollingRowIndex = dataGridViewDebug.RowCount - 1;
        }

        private void pictureBox1_Click(object sender, EventArgs e)
        {
        }

        private void Form1_FormClosed(object sender, FormClosedEventArgs e)
        {
            //if (renderThread != null)
            //{
            //    renderThread.Abort();
            //}
            //else
            //{
            //    MessageBox.Show("Check that the Lidar is on and connected to the computer!");
            //}
            consoleThread.Abort();
        }

        private void checkBoxUpdateGUI_CheckedChanged(object sender, EventArgs e)
        {
            drawGUI = checkBoxUpdateGUI.Checked;
            checkBoxUpdateList.Enabled = drawGUI;
        }

        private void checkBoxUpdateList_CheckedChanged(object sender, EventArgs e)
        {
            updateList = checkBoxUpdateList.Checked;
        }
    }
}
