
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    std::cout << "Before sleep..." << std::endl;
    int i=0;
while(1)


{   // std::this_thread::sleep_for(std::chrono::seconds(3));

    std::cout << "After sleep..." << i++<<std::endl;}

    printf("Hello world!\n");
    // 休眠 3 秒钟


    return 0;
}




