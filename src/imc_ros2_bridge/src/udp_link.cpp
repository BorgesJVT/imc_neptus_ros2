#include "udp_link.hpp"

#include <stdio.h>
#include <chrono>
#include <thread>
#include "rclcpp/rclcpp.hpp"

void die(const char *s)
{
    perror(s);
    exit(1);
}

void UDPLink::run() {
	while(threadActive)
	{
	    this->handle_receive();
	    std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}

void UDPLink::run2() {

        /*
	for(int i=0;i<5;i++) {
		printf("Thread2: %d\n",i+offset2);
		std::this_thread::sleep_for(std::chrono::milliseconds(2000));
	}
	*/
	
    /*
	while(threadActive)
	{
	    std::string address = "127.0.0.1"; //"224.0.75.69"; //Neptus address
	    IMC::Announce msg;
	    msg.sys_name = "imc_ros2_bridge";
            // 0=CCU, 1=HUMANSENSOR, 2 = UUV, 3 = ASV, 4=UAV, 5=UGV, 6=STATICSENSOR
            msg.sys_type = 2; // UUV = Unmanned underwater veh.
            msg.owner = 0;
            msg.services = "imc+udp://" + address + ":" + "6002" + "/;";
            this->publish_multicast(msg, address);
            
            
            IMC::Heartbeat msg_heartbeat;
            this->publish(msg_heartbeat, address);
            
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            
    }
    */
	
}

UDPLink::UDPLink(std::function<void (IMC::Message*)> recv_handler,
                    const std::string& bridge_addr,
                    const int& bridge_port,
                    int imc_id, 
                    int imc_src)
: recv_handler_(recv_handler)
{
    //std::string neptus_adddress = "224.0.75.69";
    //const char *address = neptus_adddress.c_str();
    //int i , recv_len;
   
    //create a UDP socket
    if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {
	    die("socket");
    }
	
    // zero out the structure
    memset((char *) &si_me, 0, sizeof(si_me));
    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(bridge_port);
    si_me.sin_addr.s_addr = htonl(INADDR_ANY);//inet_addr("224.0.75.69");//htonl(INADDR_ANY); //inet_addr("224.0.75.69");
    
    //int set_option_on = 1;
    //if(setsockopt(s, SOL_SOCKET, SO_REUSEADDR, (char*) &set_option_on, sizeof(set_option_on)) < 0)
    //    die("setsockopt");
    
    // bind socket to port
    if( bind(s , (const struct sockaddr*)&si_me, sizeof(si_me) ) == -1)
        die("bind");


    //threadActive = true;

	
}

UDPLink::~UDPLink()
{
    threadActive = false;
}

void UDPLink::handle_receive()
{
    int recv_len;
    char buf[4096];
    socklen_t slen = sizeof(si_other);

    if ((recv_len = recvfrom(s, buf, 4096, 0, (struct sockaddr *)&si_other, &slen)) < 0)
    {
        threadActive = false;
        die("recvfrom()");
    }
    
    std::cout << "recv_len: " << recv_len << std::endl;
 
    for (int i = 0; i < recv_len; ++i) {
        IMC::Message* m = parser_.parse(buf[i]);
        //std::cout << "data: " << buf[i] << std::endl;
                
        if (m) {
            recv_handler_(m);
            delete m;
        }
    }
}

//announce
void UDPLink::publish_multicast(IMC::Message& msg, const std::string& multicast_addr)
{
    msg.setSource(imc_src);
    msg.setSourceEntity(imc_id);
    msg.setDestination(0);
    msg.setTimeStamp(rclcpp::Clock().now().seconds());
    
    char out_buffer_[4096];
    uint16_t rv = IMC::Packet::serialize(&msg, (uint8_t*)out_buffer_, sizeof(out_buffer_));
    
    for (int multicast_port : announce_ports)
    {
    	//std::cout << "Multicast_port: " << multicast_port << std::endl;
    	si_other.sin_family = AF_INET;
    	si_other.sin_port = htons(multicast_port);
    	si_other.sin_addr.s_addr = inet_addr(multicast_addr.c_str());
    	
    	if (sendto(s, out_buffer_, rv, 0, (struct sockaddr*)&si_other, sizeof(si_other)) == -1)
        {
            threadActive = false;
	        die("sendto()");
        }
    }
}

//heartbeat
void UDPLink::publish(IMC::Message& msg, const std::string& address)
{
   msg.setSource(imc_src);
   msg.setSourceEntity(imc_id);
   msg.setDestination(0);
   msg.setTimeStamp(rclcpp::Clock().now().seconds());
    
   char out_buffer_[4096];
   uint16_t rv = IMC::Packet::serialize(&msg, (uint8_t*)out_buffer_, sizeof(out_buffer_));

   si_other.sin_family = AF_INET;
   si_other.sin_port = htons(6001);
   si_other.sin_addr.s_addr = inet_addr(address.c_str());
   
   if (sendto(s, out_buffer_, rv, 0, (struct sockaddr*)&si_other, sizeof(si_other)) == -1)
   {
        threadActive = false;
	    die("sendto()");
   }

}



