#include "udp_Interface.h"

namespace stmotion_controller
{
    namespace udp
    {
        UDP_Interface::UDP_Interface()
        {
        }

        void UDP_Interface::Setup(int port_define)
        {
            try
            {
                socket = std::make_shared<UDP_Socket>();
                socket->Setup(IP_addr, port_define, timeout_us);
                std::cout << "UDP Connection to Robot Established!" << std::endl;
                std::cout << "IP address: " << IP_addr << std::endl;
                std::cout << "Port: " << port_define << std::endl;
            }
            catch (const std::exception &e)
            {
                throw;
            }
        }

        void UDP_Interface::ServerSetup(int port_define)
        {
            try
            {
                socket = std::make_shared<UDP_Socket>();
                socket->ServerSetup(port_define);
                std::cout << "UDP Connection to Robot Established!" << std::endl;
                std::cout << "IP address: " << IP_addr << std::endl;
                std::cout << "Port: " << port_define << std::endl;
            }
            catch (const std::exception &e)
            {
                throw;
            }
        }
        void UDP_Interface::Shutdown()
        {
            socket->Close();
        }

        void UDP_Interface::SendJointPos(float *joint_pos_cmd)
        {
            char joint_cmd_buff[24];
            memset(joint_cmd_buff, 0, 24);
            memcpy(joint_cmd_buff, joint_pos_cmd, 24);
            socket->SendTo(&joint_cmd_buff, sizeof(joint_cmd_buff));
        }

        void UDP_Interface::GetJointPos(float *joint_pos_fbk)
        {


            memset(com_buffer, 0, 24);
            socket->RecvFrom(com_buffer, 24);
            memcpy(joint_pos_fbk, com_buffer, 24);
        }

    }
}