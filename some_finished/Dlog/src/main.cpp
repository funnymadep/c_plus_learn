#include "dlogger.h"

int main(){
    Duration time;
    DLogger loger("/mnt/hgfs/workplace/log/Dlog");
    // DLogger loger;
    loger.log("warning", "/mnt/hgfs/workplace/log/Dlog/src/main.cpp" , 5, DLogger::Log_Warning);
    std::cout << time.duration() << std::endl;
    return 0;
}