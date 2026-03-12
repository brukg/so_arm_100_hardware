#include <iostream>
#include "SCServo.h"
#include <thread>
#include <chrono>

SMS_STS sm_st;

int main(int /*argc*/, char ** /*argv*/)
{
    const char* port = "/dev/ttyUSB0";  // Use absolute path
    std::cout<<"serial:"<<port<<std::endl;
    
    if(!sm_st.begin(1000000, port)){
        std::cout<<"Failed to init sms/sts motor!"<<std::endl;
        return 0;
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
    return 1;
} 