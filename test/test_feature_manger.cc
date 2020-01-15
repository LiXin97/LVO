//
// Created by lixin04 on 20年1月13日.
//

#include "Odometry.hpp"

int main()
{
    LVO::Odometry odo;

    std::vector< std::pair<int, int> > feature_frame;
    feature_frame.emplace_back(0, 0 );
    feature_frame.push_back( std::make_pair(0, 1) );
    feature_frame.push_back( std::make_pair(1, 0) );
    feature_frame.push_back( std::make_pair(1, 1) );
    feature_frame.push_back( std::make_pair(1, 2) );
    feature_frame.push_back( std::make_pair(2, 2) );
    feature_frame.push_back( std::make_pair(2, 3) );
    feature_frame.push_back( std::make_pair(2, 4) );
    feature_frame.push_back( std::make_pair(2, 5) );
    feature_frame.push_back( std::make_pair(2, 6) );
    feature_frame.push_back( std::make_pair(2, 7) );

    for(auto &it:feature_frame)
    {
        Eigen::Vector4d observe;
        odo.insert_ob(it.first, it.second, observe);
    }
    odo.print_SWobs();

//    odo.remove_frame(0);
//    std::cout << "remove 0 over" << std::endl;
//    odo.print_SWobs();
//    odo.remove_frame(1);
//    std::cout << "remove 1 over" << std::endl;
//    odo.print_SWobs();
//    odo.remove_frame(2);
//    std::cout << "remove 2 over" << std::endl;
//    odo.print_SWobs();
//    odo.remove_frame(10);
//    odo.print_SWobs();

    std::set<long> remaind_ids;
    remaind_ids.insert(3);
    remaind_ids.insert(7);
    remaind_ids.insert(71);

    odo.keep_SW_frame(remaind_ids);
    odo.print_SWobs();

    return 0;
}