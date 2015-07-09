using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;
using System.IO.Ports;

namespace PDACmd
{
    public partial class Form1 : Form
    {
            //SerialPort robotPort = new SerialPort("COM20", 9600, Parity.None, 8, StopBits.One);
         public Form1()
        {
            InitializeComponent();
            //robotPort.PortName = "COM33";
            robotPort.PortName = "COM7";
            robotPort.Open();
            robotPort.Write(new byte[] { 0xaa, 1, 7, (byte)'S' }, 0, 4);
            // Turn on Full Mode
            ///////robotPort.Write(new byte[] { 128, 132 }, 0, 2);

            // Define the song
            ////////robotPort.Write(new byte[] { 140, 0, 4, 62, 12, 66, 12, 69, 12, 74, 36 }, 0, 11);

            // Play the song
            //robotPort.Write(new byte[] { 141, 0 }, 0, 2);
            
            // Get banner 
            robotPort.Write(new byte[] { 1, 7, (byte)'h' }, 0, 3);
            //timer1.Enabled = true;
           
        }

        private void btnForward_Click(object sender, EventArgs e)
        {
            //btnForward.Enabled = false;
            //SerialPort robotPort = new SerialPort("COM7", 9600, Parity.None, 8, StopBits.One);
            //robotPort.Open();

            // Turn on Full Mode
            //robotPort.Write(new byte[] { 1, 7, (byte)'l' }, 0, 3);
            //robotPort.Write(new byte[] { 1, 7, (byte)'r' }, 0, 3);
            //robotPort.Write(new byte[] { 1, 7, (byte)'R' }, 0, 3);
            //robotPort.Write(new byte[] { 1, 7, (byte)'S' }, 0, 3);
            //robotPort.Write(new byte[] { 1, 7, (byte)'F', 0, 100 }, 0, 5);
            //robotPort.Write(new byte[] { 1, 7, (byte)'h' }, 0, 3);
            //timer1.Enabled = true;
            // 201, speed Y MSB, LSB, Radius MSB, LSB, Distance Upper MSB, LSB, Lower MSB, LSB 
            robotPort.Write(new byte[] { 201, 0, 90, 0, 0, 0, 0, 0, 0 }, 0, 9);
            //robotPort.Write(new byte[] { 201, 0xfe, 0xf2, 0x80, 0, 0, 0, 0x04, 0 }, 0, 9);

            //robotPort.Close();
            //btnForward.Enabled = true;
        }

        private void btnStop_Click(object sender, EventArgs e)
        {
            robotPort.Write(new byte[] { 1, 7, (byte)'S' }, 0, 3);
            timer1.Enabled = false;
            /*
            // Drive in a square
            robotPort.Write(new byte[] { 
                152, 17, 137, 1, 44, 128, 0, 156, 1, 
                144, 137, 1, 44, 0, 1, 157, 0, 90, 153 }, 0, 19);

            // Execute script
            robotPort.Write(new byte[] { 153 }, 0, 1);

            //robotPort.Close();
            */
        }

        private void btnReverse_Click(object sender, EventArgs e)
        {
            // 201, speed Y MSB, LSB, Radius MSB, LSB, Distance Upper MSB, LSB, Lower MSB, LSB 
            robotPort.Write(new byte[] { 201, 0xff, 0xa6, 0, 0, 0, 0, 0, 0 }, 0, 9);

        }

        private void btnSpinLeft_Click(object sender, EventArgs e)
        {
            // 201, speed Y MSB, LSB, Radius MSB, LSB, Distance Upper MSB, LSB, Lower MSB, LSB 
            robotPort.Write(new byte[] { 201, 0, 100, 0, 1, 0, 0, 0x03, 0x00 }, 0, 9);
        }

        private void btnSpinRight_Click(object sender, EventArgs e)
        {
            // 137, speed Y MSB, LSB, Radius MSB, LSB 
            robotPort.Write(new byte[] { 137, 0, 90, 0xFF, 0xFF }, 0, 5);
        }

        private void PrintMessage(byte[] message)
        {
            if (InvokeRequired)
            {
                BeginInvoke(new MethodInvoker(delegate()
                {
                    PrintMessage(message);
                }));
            }
            else
            {
                string buffered_string = "";

                foreach (byte d in message)
                {
                    buffered_string += (char)d;
                }

                richTextBox1.AppendText(buffered_string);
                richTextBox1.ScrollToCaret();

            }
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            //robotPort.Open();
            //robotPort.Write(new byte[] { 1, 7, (byte)'S' }, 0, 3);
        }

        private void robotPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            byte[] data_buffer = new byte[robotPort.BytesToRead];

            robotPort.Read(data_buffer, 0, robotPort.BytesToRead);

            PrintMessage(data_buffer);
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            //robotPort.Write(new byte[] { 1, 7, (byte)'g' }, 0, 3);
            robotPort.Write(new byte[] { 1, 7, (byte)'h' }, 0, 3);
            //robotPort.Write(new byte[] { 1, 7, (byte)'S' }, 0, 3);
        }

        private void btnTurnNorth_Click(object sender, EventArgs e)
        {
            // 202, speed Y MSB, LSB, Heading MSB, LSB 
            robotPort.Write(new byte[] { 202, 0, 90, 0, 0 }, 0, 5);
        }

        private void btnTurnEast_Click(object sender, EventArgs e)
        {
            // 202, speed Y MSB, LSB, Heading MSB, LSB 
            robotPort.Write(new byte[] { 202, 0, 90, 0, 90 }, 0, 5);
        }

        private void btnTurnSouth_Click(object sender, EventArgs e)
        {
            // 202, speed Y MSB, LSB, Heading MSB, LSB 
            robotPort.Write(new byte[] { 202, 0, 90, 0, 180 }, 0, 5);
        }

        private void btnTurnWest_Click(object sender, EventArgs e)
        {
            // 202, speed Y MSB, LSB, Heading MSB, LSB 
            robotPort.Write(new byte[] { 202, 0, 90, 0x01, 0x0E }, 0, 5);
        }

        private void btnLoopLeftFwd_Click(object sender, EventArgs e)
        {
            // 137, speed Y MSB, LSB, Radius MSB, LSB 
            robotPort.Write(new byte[] { 137, 0, 90, 0x01, 0xF4 }, 0, 5);
        }

        private void btnLoopRightFwd_Click(object sender, EventArgs e)
        {
            // 137, speed Y MSB, LSB, Radius MSB, LSB 
            robotPort.Write(new byte[] { 137, 0, 90, 0xFE, 0x0C }, 0, 5);
        }

        private void btnLoopLeftRvr_Click(object sender, EventArgs e)
        {
            // 137, speed Y MSB, LSB, Radius MSB, LSB 
            robotPort.Write(new byte[] { 137, 0xFF, 0xA6, 0x01, 0xF4 }, 0, 5);
        }

        private void btnLoopRightRvr_Click(object sender, EventArgs e)
        {
            // 137, speed Y MSB, LSB, Radius MSB, LSB 
            robotPort.Write(new byte[] { 137, 0xFF, 0xA6, 0xFE, 0x0C }, 0, 5);
        }
    }
}