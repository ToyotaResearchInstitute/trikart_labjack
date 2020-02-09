#include <LabJackM.h>
#include <iostream>

int main(int argc, char** argv)
{
    int temp = LJM_CloseAll();
    std::cout << "Error code: " << temp << std::endl;
    return 0;
}