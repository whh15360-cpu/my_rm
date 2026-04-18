// Plotter.cpp
#include "Plotter.hpp"
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

namespace tools
{

Plotter::Plotter(std::string host, uint16_t port)
{
    socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_ < 0)
    {
        std::cerr << "[Plotter] Failed to create socket" << std::endl;
        return;
    }

    memset(&destination_, 0, sizeof(destination_));
    destination_.sin_family = AF_INET;
    destination_.sin_port = htons(port);
    destination_.sin_addr.s_addr = inet_addr(host.c_str());
}

Plotter::~Plotter()
{
    if (socket_ >= 0)
    {
        close(socket_);
    }
}

void Plotter::plot(const nlohmann::json& json)
{
    if (socket_ < 0) return;

    std::string data = json.dump() + "\n";  // 确保换行符，适配 PlotJuggler

    std::lock_guard<std::mutex> lock(mutex_);
    sendto(socket_, data.c_str(), data.length(), 0,
           (struct sockaddr*)&destination_, sizeof(destination_));
}

}  // namespace tools
