#pragma once

#include <ctime>
#include <cstdlib>
#include <chrono>
/**
 * @brief The TicToc class
 * 用于计时的小工具
 */
class TicToc
{
  public:
    TicToc()
    {
        tic();
    }

    /**
     * @brief tic 计时初始化
     */
    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    /**
     * @brief toc 计时结束
     * @return 从上次调用tic()开始到现在调用的时间。单位 毫秒
     */
    double toc()
    {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }

  private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};
