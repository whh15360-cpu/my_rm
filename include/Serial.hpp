#ifndef SERIAL_HPP
#define SERIAL_HPP

#include <termios.h>
#include <cstddef>
#include <cstdint>
#include <string>

class Serial
{
public:
    Serial();
    ~Serial();

    // 打开串口，device: 设备路径（如 "/dev/ttyAMA0"），baudrate: 波特率常量（如 B115200）
    bool open(const char* device, speed_t baudrate);

    // 发送原始数据
    ssize_t write(const void* data, size_t len);

    // 接收原始数据
    ssize_t read(void* buffer, size_t len);

    // 关闭串口
    void close();

    // 检查串口是否打开
    bool is_open() const;

    // ----- 以下为新增的便捷方法，不破坏原有接口 -----
    // 发送字符串（自动添加 '\n' 可选）
    bool writeString(const std::string& str, bool appendNewline = true);

    // 读取一行数据（以 '\n' 结尾），返回空字符串表示超时或错误
    std::string readLine();

    // 清空输入/输出缓冲区
    void flush();

    // 获取输入缓冲区中可读的字节数
    int available();

private:
    int fd; // 文件描述符

    bool configure(speed_t baudrate);
};

#endif // SERIAL_HPP
