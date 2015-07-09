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
            robotPort.PortName = "COM1";
            robotPort.Open();
            //robotPort.Write(new byte[] { 0xaa, 1, 7, (byte)'S' }, 0, 4);
            // Turn on Full Mode
            ///////robotPort.Write(new byte[] { 128, 132 }, 0, 2);

            // Define the song
            ////////robotPort.Write(new byte[] { 140, 0, 4, 62, 12, 66, 12, 69, 12, 74, 36 }, 0, 11);

            // Play the song
            //robotPort.Write(new byte[] { 141, 0 }, 0, 2);
            
            // Get banner 
            //robotPort.Write(new byte[] { 1, 7, (byte)'h' }, 0, 3);
            //timer1.Enabled = true;
           
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
                //string buffered_string = "";

                foreach (byte d in message)
                {
                    //buffered_string += (char)d;
                    //richTextBox1.AppendText(d.ToString("X") + ",");
                    richTextBox1.AppendText(d.ToString() + ",");
                }

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

            if ((142 == data_buffer[0]) && (6 == data_buffer[1]))
            {
                robotPort.Write(new byte[] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xff, 0, 0, 0, 0, 0, 0, 0x38, 0x40, 0xfe, 0x0c,
                                             26, 0x10, 0x68, 0x10, 0x68, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x1f, 0, 0, 0, 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0, 52);
            }

            PrintMessage(data_buffer);
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            //robotPort.Write(new byte[] { 1, 7, (byte)'g' }, 0, 3);
            //robotPort.Write(new byte[] { 1, 7, (byte)'h' }, 0, 3);
            //robotPort.Write(new byte[] { 1, 7, (byte)'S' }, 0, 3);
            robotPort.Write(new byte[] { 1, 7, (byte)'M' }, 0, 3);
        }

        private void btnForward_Click(object sender, EventArgs e)
        {
                    }

    }
}