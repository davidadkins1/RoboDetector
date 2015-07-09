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
        //SerialPort robotPort = new SerialPort("COM1", 9600, Parity.None, 8, StopBits.One);

        public Form1()
        {
            InitializeComponent();
        }

        private void button1_Click(object sender, EventArgs e)
        {
            button1.Enabled = false;
            //SerialPort robotPort = new SerialPort("COM7", 9600, Parity.None, 8, StopBits.One);
            //robotPort.Open();

            // Turn on Full Mode
            robotPort.Write(new byte[] { (byte)'D', (byte)'a', (byte)'v' }, 0, 3);
/*            robotPort.Write(new byte[] { 1, 7, (byte)'l' }, 0, 3);
            robotPort.Write(new byte[] { 1, 7, (byte)'r' }, 0, 3);
            robotPort.Write(new byte[] { 1, 7, (byte)'R' }, 0, 3);
            robotPort.Write(new byte[] { 1, 7, (byte)'S' }, 0, 3);
            robotPort.Write(new byte[] { 1, 7, (byte)'F' }, 0, 3);
            robotPort.Write(new byte[] { 1, 7, (byte)'h' }, 0, 3);
*/
            //robotPort.Close();
            button1.Enabled = true;
        }

        private void button2_Click(object sender, EventArgs e)
        {
           

            // Drive in a square
            robotPort.Write(new byte[] { 
                152, 17, 137, 1, 44, 128, 0, 156, 1, 
                144, 137, 1, 44, 0, 1, 157, 0, 90, 153 }, 0, 19);

            // Execute script
            robotPort.Write(new byte[] { 153 }, 0, 1);

            //robotPort.Close();
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            robotPort.Open();
        }

        private void robotPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            byte[] sp_buffer = new byte[robotPort.BytesToRead];

            robotPort.Read(sp_buffer, 0, robotPort.BytesToRead);
            PrintMessage(sp_buffer);
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
                string buffer = "";

                foreach (byte d in message)
                {
                    buffer += (char)d;
                }
                richTextBox1.AppendText(buffer);
            }
        }


    }
}