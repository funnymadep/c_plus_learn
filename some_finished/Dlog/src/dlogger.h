//dlogger.h
#ifndef DLOGGER_H
#define DLOGGER_H

#include <iostream>
#include <ctime>
#include <mutex>
#include <string>
#include <thread>
#include <queue>

extern std::mutex mt;

//用于获取运行时间
class Duration
{
public:
    //创建对象时，自动获取开始时间
    explicit Duration() : m_start(clock()){}

    //返回从创建对象到现在的运行时间
    static inline double duration();

private:
    clock_t m_start;
};

double Duration::duration()
{
    Duration d;
    return (double)(clock() - d.m_start) / CLOCKS_PER_SEC;
}

class DLogger
{
public:
    enum LogLevel
    {
        Log_Info = 0,
        Log_Debug,
        Log_Warning,
        Log_Error,
        Log_Fatal,
        Level_Num
    };

public:
    //输入日志目录，默认当前文件夹
    explicit DLogger(const std::string &dir = "");

    //日志接口 log(日志信息， 发生信息的文件名， 发生信息的代码行数， 日志等级(默认info))
    void log(const char* msg, LogLevel level = Log_Info, const char* fileName = "", int line = 0);

    void setDeleteTime(int day);

    int getDeleteTime();

    void deleteFile();

private:
    std::string m_fileName;
    static const std::string m_levelMsg[Level_Num];
    int deleteTime;
    std::queue<std::string> data;
};

extern std::mutex mt;

#endif