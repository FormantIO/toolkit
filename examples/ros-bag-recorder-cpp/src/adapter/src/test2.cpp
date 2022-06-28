#include <iostream> // std::cin, std::cout
#include <iomanip>  // std::get_time
#include <ctime>    // struct std::tm

int main()
{
    struct std::tm tm;
    std::istringstream ss("16:35:12");
    ss >> std::get_time(&tm, "%H:%M:%S"); // or just %T in this case
    std::time_t time = mktime(&tm);
}