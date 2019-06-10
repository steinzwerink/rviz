#ifndef POSE_HPP_
#define POSE_HPP_

#include <iostream>
#include <vector>
#include <string>
#include <thread>

namespace pose
{
class pose
{
private:
public:
    std::string pose_name;
    int duration;

    pose();
    pose(const std::string &pose_name, const int &duration);
    virtual ~pose();
    pose &operator=(const pose &other);
};

} // namespace pose

#endif