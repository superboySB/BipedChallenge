#include "biped_ctrl_server.hpp"


int main(int argc, char *argv[])
{
    BipedCtrlServer server("0.0.0.0", 8800);
    server.launch();
    
    return 0;
}