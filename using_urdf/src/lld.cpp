#include "using_urdf/lld.h"

lld::lld::lld()
{
}

lld::lld::~lld()
{
}

bool lld::lld::detectDriver()
{
    bool armdetected = true;
    try
    {
        boost::asio::io_service ioservice;
        boost::asio::serial_port serial(ioservice, 9600);
    }
    catch (const std::exception &e)
    {
        armdetected = false;
    }
    return armdetected;
}

double lld::lld::remap(double value, double istart, double istop, double ostart, double ostop)
{
    return ostart + (ostop - ostart) * ((value - istart) / (istop - istart));
}

void lld::lld::sendAction(const uint16_t pose, const std::vector<uint16_t> servo, const std::vector<int16_t> angle, const uint16_t duration, const uint16_t size, lld *l)
{
    double begin2 = ros::Time::now().toNSec();
    double begin = ros::Time::now().toNSec();
    double previoustime = ros::Time::now().toNSec();
    double rate = 0.001;
    double framerate = (duration / 1000) / rate;
    double millard = 1000000000;
    std::vector<double> steps = {0, 0, 0, 0, 0, 0, 0};
    for (int i = 0; i < size; ++i)
    {
        auto mappedVal = l->remap(angle[i], l->c.pwmlower, l->c.pwmUpper, l->constraintsA[servo[i] - 1][0], l->constraintsA[servo[i] - 1][1]);
        if (l->positions[servo[i] - 1] != mappedVal)
        {
            steps[servo[i] - 1] = mappedVal - l->positions[servo[i] - 1];
            l->endVec[servo[i] - 1] = mappedVal;
        }
        else
        {
            steps[servo[i] - 1] = 0;
        }
    }

    if (l->endVec != l->positions)
    {
        while (ros::ok())
        {
            double now = ros::Time::now().toNSec();

            if ((((now - previoustime) / millard) >= rate) || (((now - begin) / millard) - (duration / 1000) == 0))
            {
                previoustime = now;

                std::vector<double> currentPositions = l->positions;

                for (int i = 0; i < l->positions.size(); ++i)
                {
                    l->positions[i] = currentPositions[i] + steps[i] / framerate;
                }
                l->setJoints();
            }
            if (((now - begin) / millard) - (duration / 1000) == 0)
            {
                l->positions = l->endVec;
                l->setJoints();
                for (int i = 0; i < 5000; ++i)
                {
                    l->setJoints();
                }
                break;
            }
        }
    }
}

const std::vector<double> lld::lld::getPosition()
{
    return this->positions;
}

void lld::lld::initialize()
{
    this->joint_state.header.stamp = ros::Time::now();
    this->joint_state.position.resize(7);
    this->joint_state.name.resize(7);
    uint8_t counter = 0;

    for (const auto &n : joints)
    {
        this->joint_state.name[counter] = n;
        this->joint_state.position[counter] = this->positions[counter];
        ++counter;
    }

    this->joint_pub.publish(this->joint_state);
}

void lld::lld::setJoints()
{
    this->joint_state.header.stamp = ros::Time::now();
    this->joint_state.position.resize(7);
    this->joint_state.name.resize(7);
    uint8_t counter = 0;

    for (const auto &n : joints)
    {
        this->joint_state.name[counter] = n;
        this->joint_state.position[counter] = this->positions[counter];
        ++counter;
    }

    this->joint_pub.publish(this->joint_state);
}
