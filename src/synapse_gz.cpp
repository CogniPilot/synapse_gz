#include "clients/gz_client.hpp"
#include "clients/tcp_client.hpp"
#include "synapse_tinyframe/TinyFrame.h"

#include <iostream>
#include <memory>

std::atomic<bool> g_stop{false};
std::shared_ptr<TcpClient> g_tcp_client;
std::shared_ptr<GzClient> g_gz_client;
std::shared_ptr<TinyFrame> g_tf;

void signal_handler(int signum) {
    (void)signum;
    g_stop = true;
}

void tcp_entry_point()
{
    std::cout << "tcp thread started" << std::endl;
    while (not g_stop) {
        g_tcp_client->run_for(std::chrono::seconds(1));
    }
    std::cout << "tcp thread stopped" << std::endl;
}

void gz_entry_point()
{
    std::cout << "gz thread started" << std::endl;
    while (not g_stop) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << "gz thread stopped" << std::endl;
}

int main(int argc, char ** argv) {
    (void) argc;
    if (argc < 3) {
        std::cerr << argv[0] << "\thost port" << std::endl;
        return -1;
    }
    signal(SIGINT, signal_handler);

    // parameters
    std::string prefix = "/world/default/model/elm4/link/sensors/sensor/";

    // create tinyframe
    g_tf = std::make_shared<TinyFrame>(*(TF_Init(TF_MASTER)));

    // create gz client
    g_gz_client = std::make_shared<GzClient>(prefix, g_tf);

    // create tcp client
    g_tcp_client = std::make_shared<TcpClient>(argv[1], std::atoi(argv[2]), g_tf);
    
    // start threads
    std::thread tcp_thread(tcp_entry_point);
    std::thread gz_thread(gz_entry_point);

    // join threads
    tcp_thread.join();
    gz_thread.join();

    return 0;
}

// vi: ts=4 sw=4 et
