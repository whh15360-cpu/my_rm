#include "Serial.hpp"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include <sys/ioctl.h>

Serial::Serial() : fd(-1) {}

Serial::~Serial()
{
    close();
}

bool Serial::open(const char* device, speed_t baudrate)
{
    // 使用 O_RDWR | O_NOCTTY，去掉 O_NDELAY 以避免非阻塞读的副作用
    fd = ::open(device, O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        std::cerr << "[Serial] 打开串口失败: " << strerror(errno) << std::endl;
        return false;
    }

    if (!configure(baudrate))
    {
        close();
        return false;
    }

    return true;
}

bool Serial::configure(speed_t baudrate)
{
    struct termios options;
    if (tcgetattr(fd, &options) < 0)
    {
        std::cerr << "[Serial] 获取串口属性失败: " << strerror(errno) << std::endl;
        return false;
    }

    // 设置波特率
    cfsetispeed(&options, baudrate);
    cfsetospeed(&options, baudrate);

    // 控制模式标志：启用接收器，忽略调制解调器控制线
    options.c_cflag |= (CLOCAL | CREAD);
    // 8 数据位
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    // 1 停止位
    options.c_cflag &= ~CSTOPB;
    // 无奇偶校验
    options.c_cflag &= ~PARENB;
    // 禁用硬件流控
    options.c_cflag &= ~CRTSCTS;

    // 本地模式：原始输入，禁用回显、信号、规范输入
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // 输入模式：禁用软件流控（XON/XOFF），禁止换行符转换
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_iflag &= ~(INLCR | ICRNL | IGNCR);

    // 输出模式：原始输出，不进行后处理
    options.c_oflag &= ~OPOST;

    // 设置读取超时：VMIN=0, VTIME=10 表示非阻塞读，每 0.1 秒轮询一次，整体超时 1 秒
    // 若需要阻塞直到收到指定字节，可将 VMIN 设为非零，此处保持通用
    options.c_cc[VMIN] = 0;      // 不等待最小字节数
    options.c_cc[VTIME] = 10;    // 超时 1 秒（10 * 0.1 秒）

    // 清空缓冲区
    tcflush(fd, TCIOFLUSH);

    // 立即生效配置
    if (tcsetattr(fd, TCSANOW, &options) < 0)
    {
        std::cerr << "[Serial] 设置串口属性失败: " << strerror(errno) << std::endl;
        return false;
    }

    return true;
}

ssize_t Serial::write(const void* data, size_t len)
{
    if (fd < 0) return -1;
    return ::write(fd, data, len);
}

ssize_t Serial::read(void* buffer, size_t len)
{
    if (fd < 0) return -1;
    return ::read(fd, buffer, len);
}

void Serial::close()
{
    if (fd >= 0)
    {
        ::close(fd);
        fd = -1;
    }
}

bool Serial::is_open() const
{
    return fd != -1;
}

// ----- 便捷方法实现 -----
bool Serial::writeString(const std::string& str, bool appendNewline)
{
    std::string out = str;
    if (appendNewline) out += '\n';
    ssize_t ret = write(out.c_str(), out.size());
    return (ret == (ssize_t)out.size());
}

std::string Serial::readLine()
{
    std::string line;
    char ch;
    while (true)
    {
        ssize_t n = read(&ch, 1);
        if (n <= 0)
        {
            // 超时或无数据
            if (line.empty()) return "";
            else break; // 如果已经收到部分数据，遇到超时就返回已有数据
        }
        if (ch == '\n') break;
        line.push_back(ch);
    }
    return line;
}

void Serial::flush()
{
    if (fd >= 0)
        tcflush(fd, TCIOFLUSH);
}

int Serial::available()
{
    if (fd < 0) return 0;
    int bytes = 0;
    if (ioctl(fd, FIONREAD, &bytes) == -1)
        return 0;
    return bytes;
}
