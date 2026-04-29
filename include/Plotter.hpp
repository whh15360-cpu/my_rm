#ifndef TOOLS_PLOTTER_HPP
#define TOOLS_PLOTTER_HPP

#include <netinet/in.h>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>

namespace tools
{
class Plotter
{
public:
    Plotter(std::string host = "127.0.0.1", uint16_t port = 9870);
    ~Plotter();

    void plot(const nlohmann::json& json);

private:
    int socket_;
    sockaddr_in destination_;
    std::mutex mutex_;
};
}  // namespace tools

#endif  // TOOLS_PLOTTER_HPP
