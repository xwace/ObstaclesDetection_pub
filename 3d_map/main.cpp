#include "process/process.h"
#include <thread>
int main()
{
    NS_PROCESS::Process process;
    process.init();
    process.process();
   
    return 0;

}


