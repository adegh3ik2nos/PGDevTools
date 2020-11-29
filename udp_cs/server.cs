
using System;
using System.Text;
using System.Runtime.InteropServices;
using System.Net.Sockets;
using System.Net;

namespace test10
{
    class Program
    {
        // Win32 APIであるSetConsoleCtrlHandler関数の宣言
        public enum CtrlTypes
        {
            CTRL_C_EVENT = 0,
            CTRL_BREAK_EVENT = 1,
            CTRL_CLOSE_EVENT = 2,
            CTRL_LOGOFF_EVENT = 5,
            CTRL_SHUTDOWN_EVENT = 6
        }
        delegate bool HandlerRoutine(CtrlTypes CtrlType);
        [DllImport("Kernel32")]
        static extern bool SetConsoleCtrlHandler(HandlerRoutine Handler, bool Add);

        //サーバ終了通知
        private static bool _OnKill(CtrlTypes ctrlType)
        {
            m_udpClient?.Close();

            return false;
        }

        static void Main()
        {
            //終了時ハンドル
            HandlerRoutine killHandler;
            killHandler = new HandlerRoutine(_OnKill);
            SetConsoleCtrlHandler(killHandler, true);


            //バインドするローカルIPとポート番号
            string localIp = "127.0.0.1";
            IPAddress localAddress = IPAddress.Parse(localIp);
            int localPort = 2002;

            //UdpClientを作成し、ローカルエンドポイントにバインドする
            IPEndPoint localEP =new IPEndPoint(localAddress, localPort);
            m_udpClient = new UdpClient(localEP);
            if(null != m_udpClient)
            {
                while (true)
                {
                    IPEndPoint remoteEP = null;
                    byte[] rcvBytes = null;
                    rcvBytes = m_udpClient?.Receive(ref remoteEP);
                    string rcvMsg = Encoding.UTF8.GetString(rcvBytes);

                    //受信したデータと送信者の情報を表示する
                    Console.WriteLine("受信したデータ:{0}", rcvMsg);
                    Console.WriteLine("送信元アドレス:{0}/ポート番号:{1}", remoteEP.Address, remoteEP.Port);

                    for(int i = 0; i < 100000; ++i)
                    {
                        Console.WriteLine();
                    }

                    //戻り値をクライアントに戻す
                    string sendMsg = 1.ToString();
                    byte[] buff = Encoding.UTF8.GetBytes(sendMsg);
                    m_udpClient?.Send(buff, buff.Length, remoteEP.Address.ToString(), remoteEP.Port);
                }
            }
        }

        static UdpClient m_udpClient;
    }
}
