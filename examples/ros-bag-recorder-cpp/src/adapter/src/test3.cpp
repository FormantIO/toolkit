#include <iostream>
#include <cstdlib>
 
int main()
{
    if(const char* env_p = std::getenv("FORMANT_BAG_RECORDER_CONFIG_LOCATIONs"))
        std::cout << "Your PATH is: " << env_p << '\n';
    else
        std::cout << "ENV not set" << std::endl; 
}