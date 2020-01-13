//
// Created by lixin04 on 20年1月13日.
//

#ifndef LVO_TIC_TOC_HPP
#define LVO_TIC_TOC_HPP

#include <ctime>
#include <cstdlib>
#include <chrono>

class TicToc {
public:
    TicToc() {
        tic();
    }

    void tic() {
        start = std::chrono::system_clock::now();
    }

    double toc() {
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        return elapsed_seconds.count() * 1000;
    }

private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
};


#endif //LVO_TIC_TOC_HPP
