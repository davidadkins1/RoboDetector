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
            robotPort.PortName = "COM17";
            robotPort.Open();
            robotPort.Write(new byte[] { 1, 7, (byte)'S' }, 0, 3);
            // Turn on Full Mode
            robotPort.Write(new byte[] { 128, 132 }, 0, 2);

            // Define the song
            robotPort.Write(new byte[] { 140, 0, 4, 62, 12, 66, 12, 69, 12, 74, 36 }, 0, 11);

            // Play the song
            robotPort.Write(new byte[] { 141, 0 }, 0, 2);
            
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
            //robotPort.Write(new byte[] { 1, 7, (byte)'F', 39, 30 }, 0, 5);
            robotPort.Write(new byte[] { 201, 0, 80, 0, 1, 0, 0, 0, 239 }, 0, 9);

            //robotPort.Close();
            //btnForward.Enabled = true;
        }

        private void btnStop_Click(object sender, EventArgs e)
        {
            robotPort.Write(new byte[] { 1, 7, (byte)'S' }, 0, 3);
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
            robotPort.Write(new byte[] { 1, 7, (byte)'g' }, 0, 3);
        }
    }
}