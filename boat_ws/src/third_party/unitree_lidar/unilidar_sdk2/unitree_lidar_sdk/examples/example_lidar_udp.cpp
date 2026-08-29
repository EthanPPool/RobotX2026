/**********************************************************************
 Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "example.h"

int main(int argc, char *argv[])
{
    UnitreeLidarReader *lreader = createUnitreeLidarReader();

    std::string lidar_ip = "192.168.1.62";
    std::string local_ip = "192.168.1.2";

    unsigned short lidar_port = 6101;
    unsigned short local_port = 6201;

    if (lreader->initializeUDP(lidar_port, lidar_ip, local_port, local_ip))
    {
        printf("Unilidar initialization failed! Exit here!\n");
        exit(-1);
    }
    else
    {
        printf("Unilidar initialization succeed!\n");
    }

    std::cout << "[TEST] calling startLidarRotation()" << std::endl;
    lreader->startLidarRotation();
    std::cout << "[TEST] startLidarRotation() returned" << std::endl;
    sleep(1);

    uint32_t workMode = 0;

    std::cout << "[TEST] calling setLidarWorkMode("
              << workMode << ")" << std::endl;

    lreader->setLidarWorkMode(workMode);

    std::cout << "[TEST] setLidarWorkMode() returned" << std::endl;
    sleep(1);

    std::cout << "[TEST] calling resetLidar()" << std::endl;

    lreader->resetLidar();

    std::cout << "[TEST] resetLidar() returned" << std::endl;
    sleep(1);

    std::cout << "[TEST] entering exampleProcess()" << std::endl;

    exampleProcess(lreader);

    std::cout << "[TEST] exampleProcess() returned" << std::endl;

    return 0;
}
