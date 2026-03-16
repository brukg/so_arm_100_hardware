#include <iostream>
#include "SCServo.h"
#include <thread>
#include <chrono>

SMS_STS sm_st;

int main(int argc, char ** argv)
{
    const char* port = "/dev/ttyUSB0";  // Use absolute path

    // First optional positional argument is the serial port
    if (argc > 1) {
        port = argv[1];
    }
    if (argc > 2) {
        std::cout << "Invalid argument [" << argv[2] << "]. "
                  << "Expect one optional argument [serial_port]"
                  << std::endl;
        return -1;
    }
    std::cout << "serial: " << port << std::endl;

    if(!sm_st.begin(1000000, port)){
        std::cout<<"Failed to init sms/sts motor!"<<std::endl;
        return -1;
    }
    
    // Add delay after initialization
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    std::cout << "Attempting ping..." << std::endl;
    int ID = sm_st.Ping(1);
    std::cout << "Ping returned: " << ID << std::endl;
    
    if(ID!=-1){
        std::cout<<"ID:"<<ID<<std::endl;
    }else{
        std::cout<<"Ping servo ID error!"<<std::endl;
    }

    // Add delay before end
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    sm_st.end();
    return 0;
}
