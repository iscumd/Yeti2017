using System;
using System.Collections.Generic;
using System.Linq;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading.Tasks;

namespace Yeti2015.Hardware.Lidar
{
    public class LidarController
    {
        TcpClient lidarClient;
        NetworkStream lidarStream;

        public LidarController(string ipAddress)
        {
            lidarClient = new TcpClient(ipAddress, 2111); // Port found in manual
            ////lidarClient = new TcpClient(ipAddress, 31416); // Port for testing
            lidarStream = lidarClient.GetStream();
        }

        public LidarController(string ipAddress, int port)
        {
            lidarClient = new TcpClient(ipAddress, port);
            lidarStream = lidarClient.GetStream();
        }

        static string receivedData = "";
        public int?[] ScanData()
        {
            string headerToFind = "DIST1 3F800000 00000000 FFF92230 9C4 439 ";
            string footerToFind = " 0 0 0 0 0 0\x03";

            
            //if (receivedData.IndexOf(headerToFind) == -1)// justread once
                receivedData = ScanDataRaw();
            
            //System.Diagnostics.Debug.Print(receivedData);

            int headerIndex = receivedData.IndexOf(headerToFind);
            int footerIndex = receivedData.IndexOf(footerToFind, headerIndex);

            int receivedDataTrimmedStartIndex = headerIndex + headerToFind.Length;
            int receivedDataTrimmedLength = footerIndex - receivedDataTrimmedStartIndex;
            string receivedDataTrimmed = receivedData.Substring(receivedDataTrimmedStartIndex, receivedDataTrimmedLength);

            char[] delim = { ' ' };
            string[] receivedDataStringArray = receivedDataTrimmed.Split(delim, StringSplitOptions.RemoveEmptyEntries);
            int?[] receivedDataArray = new int?[receivedDataStringArray.Length];

            for (int i = 0; i < receivedDataStringArray.Length; i++)
            {
                string dataValueString = receivedDataStringArray[i];
                int dataValue = Convert.ToInt32(dataValueString, 16);

                if (dataValue >= 0x20)
                {
                    receivedDataArray[i] = dataValue;
                }
                else
                {
                    receivedDataArray[i] =  null;
                    ////receivedDataArray[i] = int.MaxValue;
                }
            }

            return receivedDataArray;
        }

        public string ScanDataRaw()
        {

            string headerToFind = "DIST1 3F800000 00000000 FFF92230 9C4 439 ";
            string footerToFind = " 0 0 0 0 0 0\x03";

            string requestCommand = "\x02sRN LMDscandata\x03";
            byte[] requestCommandBytes = Encoding.ASCII.GetBytes(requestCommand);
            int loopreps = 20;
            string receivedTrimmedData = "";

            while (loopreps == 20)
            {
                lidarStream.Write(requestCommandBytes, 0, requestCommandBytes.Length);
                receivedTrimmedData = "";

                System.Threading.Thread.Sleep(10);

                loopreps = 0;
                while ((receivedTrimmedData.IndexOf(footerToFind) == -1 || receivedTrimmedData.IndexOf(headerToFind) == -1 || receivedTrimmedData.IndexOf(footerToFind) <= receivedTrimmedData.IndexOf(headerToFind)) && loopreps < 20)
                {
                    if (lidarStream.DataAvailable)
                    {
                        byte[] receivedDataBytes = new byte[lidarClient.ReceiveBufferSize];
                        lidarStream.Read(receivedDataBytes, 0, lidarClient.ReceiveBufferSize);
                        string receivedData = Encoding.ASCII.GetString(receivedDataBytes);
                        receivedTrimmedData += receivedData.TrimEnd('\x00');
                    }
                    else
                    {
                        bool lameBreakPoint = true;
                    }

                    loopreps += 1;
                }
            }

            if (loopreps == 20)
            {
                //for deuggig
                int breakpointholder = 0;

            }

            

            return receivedTrimmedData;
        }
    }
}
