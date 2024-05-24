#include "dlogger.h"
#include <cstring>

std::mutex mt;

const std::string DLogger::m_levelMsg[Level_Num] =
    {
        "Info",
        "Debug",
        "Warning",
        "Error",
        "Fatal"};

/**
 * 为日志文件提供一个目录，不提供目录则表示当前工作目录
 */
DLogger::DLogger(const std::string &dir)
{
    char buf[64];
    time_t now = time(0);
    tm *ltm = localtime(&now);

    sprintf(buf, "%d-%d-%d.log", ltm->tm_year + 1900, ltm->tm_mon + 1, ltm->tm_mday);

    if (dir.empty())
    {
        m_fileName = buf;
    }
    else
    {
        m_fileName = dir + "/" + buf;
    }
}

/**
 * 记录日志
 * @param msg      [日志信息]
 * @param fileName [日志信息文件名]
 * @param line     [日志信息所在行]
 * @param level    [日志信息等级]
 */
void DLogger::log(const char *msg, LogLevel level, const char *fileName, int line)
{
    mt.lock();

    FILE *fp = ::fopen(m_fileName.data(), "a+");
    if (!fp)
    {
        mt.unlock();
        return;
    }
    data.push(m_fileName);
    if (data.size() > deleteTime)
    {
        remove(data.front().c_str());
        data.pop();
    }
    char buf[1024];

    time_t now = time(0);
    tm *ltm = localtime(&now);

    if (line != 0)
    {
        ::sprintf(buf, "%d/%d/%d %d:%d:%d, %s, %s, %s, %d\n",
                  1900 + ltm->tm_year, ltm->tm_mon + 1, ltm->tm_mday,
                  ltm->tm_hour, ltm->tm_min, ltm->tm_sec, msg, m_levelMsg[level].data(),
                  fileName, line);
    }
    else
    {
        ::sprintf(buf, "%d/%d/%d %d:%d:%d, %s, %s %s\n",
                  1900 + ltm->tm_year, ltm->tm_mon + 1, ltm->tm_mday,
                  ltm->tm_hour, ltm->tm_min, ltm->tm_sec, line, msg, m_levelMsg[level].data(),
                  fileName);
    }

    ::fwrite(buf, sizeof(char), strlen(buf), fp);
    ::fclose(fp);

    mt.unlock();
}

void DLogger::setDeleteTime(int day)
{
    deleteTime = day;
    std::thread deleteFileThread(deleteFile);
}

int DLogger::getDeleteTime()
{
    return this->deleteTime;
}
