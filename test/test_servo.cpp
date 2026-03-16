#include <iostream>
#include "SCServo.h"
#include <thread>
#include <chrono>

SMS_STS sm_st;

int main(int argc, char ** argv)
{
    const char* port = "/dev/ttyUSB0";

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
    std::cout << "serial:" << port << std::endl;
    
    if(!sm_st.begin(1000000, port)){
        std::cout << "Failed to init sms/sts motor!" << std::endl;
        return 0;
    }

    // Read data from all servos
    for(int id = 1; id <= 6; id++) {
        std::cout << "\nReading servo " << id << "..." << std::endl;
        
        if (sm_st.FeedBack(id) != -1) {
            // Position (0-4095)
            int pos = sm_st.ReadPos(id);
            
            // Speed (-32767 - 32767)
            int speed = sm_st.ReadSpeed(id);
            
            // Load (-1000 - 1000)
            int load = sm_st.ReadLoad(id);
            
            // Voltage (V)
            double voltage = sm_st.ReadVoltage(id) / 10.0;
            
            // Temperature (°C)
            int temp = sm_st.ReadTemper(id);
            
            // Move status (0=stopped, 1=moving)
            int move = sm_st.ReadMove(id);
            
            std::cout << "  Position: " << pos << " (0-4095)" << std::endl;
            std::cout << "  Speed: " << speed << std::endl;
            std::cout << "  Load: " << load << std::endl;
            std::cout << "  Voltage: " << voltage << "V" << std::endl;
            std::cout << "  Temperature: " << temp << "°C" << std::endl;
            std::cout << "  Moving: " << (move ? "yes" : "no") << std::endl;
        } else {
            std::cout << "  Failed to read feedback from servo " << id << std::endl;
        }
    }

    sm_st.end();
    return 0;
}
