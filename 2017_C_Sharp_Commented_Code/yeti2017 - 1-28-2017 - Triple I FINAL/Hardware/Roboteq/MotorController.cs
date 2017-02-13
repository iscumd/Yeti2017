using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace Yeti2015.Hardware.Roboteq
{
    public class MotorController : IDisposable
    {
        private SerialPort serialPort;
        private object serialPortLock = new object();
        private string sendQueue;

        public MotorController(string portName)
        {
            serialPort = new SerialPort(portName, 9600, Parity.Even, 7, StopBits.One);
            ////serialPort.DataReceived += serialPort_DataReceived;
        }

        /// <summary>
        /// Initiates the connection to the Roboteq.
        /// </summary>
        public void Connect()
        {
            serialPort.Open();
            SendRawCommand("\r\r\r\r\r\r\r\r\r\r");
        }

        /// <summary>
        /// Terminates the connection with the Roboteq.
        /// </summary>
        public void Disconnect()
        {
            serialPort.Close();
        }

        /// <summary>
        /// Sends a raw string to the Roboteq.
        /// </summary>
        /// <param name="command">The raw string to send.</param>
        /// <returns>Reserved for future use.</returns>
        public string SendRawCommand(string command)
        {
            ////var commandThread = new Thread(new ThreadStart(() => SendRawCommandThread(command)));

            ////commandThread.Start();

            SendRawCommandThread(command);

            return string.Empty;
        }

        private string SendRawCommandThread(string command)
        {
            serialPort.Write(command);

            return string.Empty;
        }

        /// <summary>
        /// Sends the reset command to the Roboteq.
        /// </summary>
        public void ResetController()
        {
            SendRawCommand("%rrrrrr");
        }

        /// <summary>
        /// Sets the power level of a motor by channel
        /// </summary>
        /// <param name="value">A value between -1 and 1.</param>
        /// <param name="channel">The channel to set.</param>
        /// <returns>Reserved for future use.</returns>
        public bool SetMotorValue(float value, Channel channel)
        {
            float constrainedValue = Math.Abs(value) > 1 ? Math.Sign(value) : value;
            var rawValue = (byte)Math.Abs(constrainedValue * 127);
            var returnValue = SetMotorValueRaw(rawValue, channel, value < 1);
            return returnValue;
        }

        public bool SetMotorValueRaw(byte value, Channel channel, bool reverse)
        {
            if (value < 0 || value > 127)
            {
                throw new ArgumentOutOfRangeException("value", value, "The Roboteq only accepts values between 0 (0x00) and 127 (0x7F).");
            }

            lock (serialPortLock)
            {
                var commandCharacter = channel.ToString()[0];

                if (reverse)
                {
                    commandCharacter = char.ToLower(commandCharacter);
                }

                SendRawCommand(string.Format("!{0}{1:X2}\r\n", commandCharacter, value));
            }

            return true;
        }

        public void SetMotorValues(float channel1Value, float channel2Value)
        {
            var char1ToSend = channel1Value >= 0 ? 'A' : 'a';
            var value1ToSend = (int)((Math.Abs(channel1Value) <= 1 ? Math.Abs(channel1Value) : 1) * 127);
            var char2ToSend = channel2Value >= 0 ? 'B' : 'b';
            var value2ToSend = (int)((Math.Abs(channel2Value) <= 1 ? Math.Abs(channel2Value) : 1) * 127);

            serialPort.DiscardOutBuffer();
            SendRawCommand(string.Format("!{0}{1:X2}\r\n!{2}{3:X2}\r\n", char1ToSend, value1ToSend, char2ToSend, value2ToSend));

        }

        public bool SetAccessoryOutput(bool turnOn)
        {
            lock (serialPortLock)
            {
                char commandCharacter = turnOn ? 'C' : 'c';
                SendRawCommand(string.Format("!{0}", commandCharacter));
            }

            return true;
        }

        public int[] GetMotorPowerLevel()
        {
            throw new NotImplementedException();

            var motorPowerLevels = new int[2];
            motorPowerLevels[0] = 0;
            motorPowerLevels[1] = 1;
            return motorPowerLevels;
        }

        public int[] GetMotorPowerLevelRaw()
        {
            SendRawCommand("?V");

            throw new NotImplementedException();

            var motorPowerLevels = new int[2];
            motorPowerLevels[0] = 0;
            motorPowerLevels[1] = 1;
            return motorPowerLevels;
        }

        public int[] GetMotorAmps()
        {
            SendRawCommand("?A");

            throw new NotImplementedException();

            var motorAmps = new int[2];
            motorAmps[0] = 0;
            motorAmps[1] = 1;
            return motorAmps;
        }

        public int[] GetAnalogInputs(int[] inputIds)
        {
            throw new NotImplementedException();

            var analogInputs = new int[2];
            analogInputs[0] = 0;
            analogInputs[1] = 1;
            return analogInputs;
        }

        public int[] GetAnalogInputsRaw(int[] inputIds)
        {
            ////SendRawCommand("?P");
            ////SendRawCommand("?R");

            throw new NotImplementedException();

            var analogInputs = new int[2];
            analogInputs[0] = 0;
            analogInputs[1] = 1;
            return analogInputs;
        }

        public int[] GetHeatsinkTemperatures()
        {
            throw new NotImplementedException();

            var heatsinkTemperatures = new int[2];
            heatsinkTemperatures[0] = 0;
            heatsinkTemperatures[1] = 1;
            return heatsinkTemperatures;
        }

        public int[] GetHeatsinkTemperaturesRaw()
        {
            SendRawCommand("?M");

            throw new NotImplementedException();

            var heatsinkTemperatures = new int[2];
            heatsinkTemperatures[0] = 0;
            heatsinkTemperatures[1] = 1;
            return heatsinkTemperatures;
        }

        public int[] GetBatteryVoltage()
        {
            throw new NotImplementedException();

            var batteryVoltages = new int[2];
            batteryVoltages[0] = 0;
            batteryVoltages[1] = 1;
            return batteryVoltages;
        }

        public int[] GetBatteryVoltageRaw()
        {
            SendRawCommand("?E");

            throw new NotImplementedException();

            var batteryVoltages = new int[2];
            batteryVoltages[0] = 0;
            batteryVoltages[1] = 1;
            return batteryVoltages;
        }

        public bool[] GetDigitalInputs()
        {
            SendRawCommand("?I");

            throw new NotImplementedException();

            var digitalInputs = new bool[2];
            digitalInputs[0] = false;
            digitalInputs[1] = false;
            return digitalInputs;
        }

        public int GetEncoderCounter(int encoderId, bool absolute)
        {
            ////SendRawCommand(string.Format("?Q{0}", commandDigit));

            throw new NotImplementedException();

            return 0;
        }

        public bool ResetEncoderCounter(int encoderId)
        {
            ////SendRawCommand(string.Format("!Q{0}", commandDigit));

            throw new NotImplementedException();

            return true;
        }

        public bool SetEncoderCounter(int encoderId, int value)
        {
            ////SendRawCommand(string.Format("!Q{0}", commandDigit));

            throw new NotImplementedException();

            return true;
        }

        public int[] GetEncoderSpeed()
        {
            SendRawCommand("?Z");

            throw new NotImplementedException();

            var speeds = new int[2];
            speeds[0] = 0;
            speeds[1] = 1;
            return speeds;
        }

        public int[] GetEncoderDisatnce()
        {
            SendRawCommand("?D");

            throw new NotImplementedException();

            var distances = new int[2];
            distances[0] = 0;
            distances[1] = 1;
            return distances;
        }

        public int[] GetEncoderSpeedOrDisatnce()
        {
            SendRawCommand("?K");

            throw new NotImplementedException();

            var speedsOrDistances = new int[2];
            speedsOrDistances[0] = 0;
            speedsOrDistances[1] = 1;
            return speedsOrDistances;
        }

        public bool[] GetEncoderLimitSwitches()
        {
            SendRawCommand("?W");

            throw new NotImplementedException();

            var limitSwitches = new bool[4];
            limitSwitches[0] = false;
            limitSwitches[1] = false;
            return limitSwitches;
        }

        public void Dispose()
        {
            serialPort.Dispose();
        }

        void serialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            throw new NotImplementedException();
        }
    }
}
