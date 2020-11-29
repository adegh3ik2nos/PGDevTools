
using System;
using System.Net;
using System.Net.Sockets;
using System.Text;

namespace test20
{
    class Program
    {
        static void Main()
        {
            //データを送信するリモートホストとポート番号
            string remoteHost = "127.0.0.1";
            IPAddress remoteAddress = IPAddress.Parse(remoteHost);
            int remotePort = 2002;
            
            UdpClient udpClient = new UdpClient();
            if (null != udpClient)
            {
                //ビルドのクエリを投げる
                string sendMsg = "test";
                byte[] buff = Encoding.UTF8.GetBytes(sendMsg);
                udpClient?.Send(buff, buff.Length, remoteHost, remotePort);

                //戻り値取得
                IPEndPoint remoteEP = null;
                buff = udpClient?.Receive(ref remoteEP);
                string rcvMsg = Encoding.UTF8.GetString(buff);
                int rtv = int.Parse(rcvMsg);

                Console.WriteLine(rtv);

                //UdpClientを閉じる
                udpClient?.Close();
            }


            Console.ReadLine();
        }
    }
}
