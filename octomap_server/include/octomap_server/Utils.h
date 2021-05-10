//
// Created by jbs on 21. 5. 8..
//

#ifndef OCTOMAP_SERVER_UTILS_H
#define OCTOMAP_SERVER_UTILS_H

/**
 * For formatting table on cmd
 */
#define TABLE_CELL_WIDTH 15
#define TABLE_TAG_WIDTH 10
#define PRECISION 2

#define FILL_CELL_LEFT(S) std::cout<< std::setfill(' ') << std::left << std::setw(TABLE_CELL_WIDTH) << std::setprecision(PRECISION)<<S;
#define FILL_TAG(S) std::cout<< std::setfill(' ') << std::left << std::setw(TABLE_TAG_WIDTH) << S;
#define FILL_CELL_RIGHT(S) std::cout<< std::setfill(' ') << std::right << std::setw(TABLE_CELL_WIDTH) << std::setprecision(PRECISION)<< S;
#define TOP_RULE_STAR(W) std::cout << std::endl; std::cout << std::setfill('*') << std::setw(TABLE_CELL_WIDTH * (W) + TABLE_TAG_WIDTH) << "*" <<std::endl;
#define MID_RULE_DASH(W) std::cout << std::endl; std::cout << std::setfill('-') << std::setw(TABLE_CELL_WIDTH * (W) + TABLE_TAG_WIDTH) << "-" <<std::endl;
#define BOTTOM_RULE_EQ(W)  std::cout << std::setfill('=') << std::setw(TABLE_CELL_WIDTH * (W) + TABLE_TAG_WIDTH) << "=" <<std::endl;
#define TITLE(W,S) std::cout << std::setfill(' ') << std::left << std::setw(TABLE_CELL_WIDTH * (W) + TABLE_TAG_WIDTH) << S;
#define NEW_LINE std::cout << std::endl;

using namespace std::chrono;
namespace octomap_server {
    class Timer {
        steady_clock::time_point t0;
        double measuredTime;
    public:
        Timer() { t0 = steady_clock::now(); }

        double stop(bool isMillisecond = true) {
            if (isMillisecond)
                measuredTime = duration_cast<milliseconds>(steady_clock::now() - t0).count();
            else
                measuredTime = duration_cast<microseconds>(steady_clock::now() - t0).count();
            return measuredTime;
        }
    };
}
#endif //OCTOMAP_SERVER_UTILS_H
