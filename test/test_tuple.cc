//
// Created by lixin04 on 20年1月15日.
//

#include <tuple>
#include <iostream>
#include <string>
#include <stdexcept>
#include <map>

std::tuple<double, char, std::string> get_student(int id)
{
    if (id == 0) return std::make_tuple(3.8, 'A', "Lisa Simpson");
    if (id == 1) return std::make_tuple(2.9, 'C', "Milhouse Van Houten");
    if (id == 2) return std::make_tuple(1.7, 'D', "Ralph Wiggum");
    throw std::invalid_argument("id");
}

void test_const_cout(const std::map<long, bool>& long_bool_map)
{
    auto iter = long_bool_map.find(100);
    std::cout << iter->second << std::endl;
}

int main()
{
    std::map<long, bool> long_bool_map;
    long_bool_map.emplace( 100, true );
//    long_bool_map.emplace( 101, true );
//    long_bool_map.emplace( 102, true );
//    long_bool_map.emplace( 103, true );
//    long_bool_map.emplace( 104, true );

    test_const_cout(long_bool_map);

    std::cout << "long_bool_map.size() = " << long_bool_map.size() << std::endl;
    long_bool_map.erase(100);
    std::cout << "long_bool_map.size() = " << long_bool_map.size() << std::endl;



    auto student0 = get_student(0);
    std::cout << "ID: 0, "
              << "GPA: " << std::get<0>(student0) << ", "
              << "grade: " << std::get<1>(student0) << ", "
              << "name: " << std::get<2>(student0) << '\n';

    double gpa1;
    char grade1;
    std::string name1;
    std::tie(gpa1, grade1, name1) = get_student(1);
    std::cout << "ID: 1, "
              << "GPA: " << gpa1 << ", "
              << "grade: " << grade1 << ", "
              << "name: " << name1 << '\n';

    // C++17 结构化绑定：
//    auto [ gpa2, grade2, name2 ] = get_student(2);
//    std::cout << "ID: 2, "
//              << "GPA: " << gpa2 << ", "
//              << "grade: " << grade2 << ", "
//              << "name: " << name2 << '\n';
}