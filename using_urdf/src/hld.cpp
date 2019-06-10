#include "state.h"

hld::Ready::Ready(hld *m)
{
    this->onEntry(m);
}

hld::Ready::~Ready()
{
}

void hld::Ready::onEntry(hld *m)
{
    ROS_INFO("STATE: READY_ON_ENTRY");
    m->setCurrentStatename("READY");
    this->onDo(m);
}

void hld::Ready::onDo(hld *m)
{
    ROS_INFO("STATE: READY_ON_DO");
    m->startListening();
    if (m->getReceived())
    {
        this->onExit(m);
    }
}

void hld::Ready::onExit(hld *m)
{
    ROS_INFO("STATE: READY_ON_EXIT");
    m->setReceived(false);
    m->setCurrent(new Active(m));
    delete this;
}

hld::Initialized::Initialized(hld *m)
{
    this->onEntry(m);
}

hld::Initialized::~Initialized(){};

void hld::Initialized::onEntry(hld *m)
{
    ROS_INFO("STATE: INITIALIZED_ON_ENTRY");
    m->setCurrentStatename("INITIALIZED");
    this->onDo(m);
}

void hld::Initialized::onDo(hld *m)
{
    ROS_INFO("STATE: INITIALIZED_ON_DO");
    this->onExit(m);
    // if(!m->detectlowlvl()){
    //     ROS_ERROR("No robot arm  availabe");
    //     ros::shutdown();
    // }else{
    //     this->onExit(m);
    // }
}

void hld::Initialized::onExit(hld *m)
{
    ROS_INFO("STATE: INITIALIZED_ON_EXIT");
    m->setCurrent(new Ready(m));
    delete this;
}

hld::Active::Active(hld *m)
{
    this->onEntry(m);
}

hld::Active::~Active()
{
}

void hld::Active::onEntry(hld *m)
{
    ROS_INFO("STATE: ACTIVE_ON_ENTRY");
    m->setCurrentStatename("ACTIVE");
    this->onDo(m);
}

void hld::Active::onDo(hld *m)
{
    ROS_INFO("STATE: ACTIVE_ON_DO");

    ROS_INFO("Executing: %s", m->getPosName().c_str());
    ROS_INFO("Duration: %i", m->goalptr->duration);
    
    m->lld.sendAction(m->goalptr->pose, m->goalptr->servo, m->goalptr->angle, m->goalptr->duration, m->goalptr->size, &m->lld);

    m->setResult();
    ROS_INFO("%s: Succeeded", m->getPosName().c_str());
    // set the action state to succeeded
    m->setSucceeded();
    this->onExit(m);
}

void hld::Active::onExit(hld *m)
{
    ROS_INFO("STATE: ACTIVE_ON_EXIT");

    if (m->getCurrentStatename() == "EMERGENCY")
    {
        m->setCurrent(new Emergency(m));
    }
    if (m->getCurrentStatename() == "ACTIVE")
    {
        m->setCurrent(new Ready(m));
    }
    delete this;
}

hld::Emergency::Emergency(hld *m)
{
    this->onEntry(m);
}

hld::Emergency::~Emergency()
{
}

void hld::Emergency::onEntry(hld *m)
{
    ROS_INFO("STATE: EMERGENCY_ON_ENTRY");
    m->setCurrentStatename("EMERGENCY");
    m->setResult();
    m->setAborted();
    this->onDo(m);
}

void hld::Emergency::onDo(hld *m)
{
    ROS_INFO("STATE: EMERGENCY_ON_DO");
    this->onExit(m);
}

void hld::Emergency::onExit(hld *m)
{
    ROS_INFO("STATE: EMERGENCY_ON_EXIT");
    ROS_ERROR("NOODSTOP");
    ros::shutdown();
    m->setCurrent(new Ready(m));
    delete this;
}

hld::hld::hld(std::string name, std::string port) : as_(nh_, name, boost::bind(&hld::hld::executeCB, this, _1), false), action_name_(name)
{
    this->current = new Initialized(this);
    //this->startListening();
}

hld::hld::~hld()
{
}

void hld::hld::setCurrent(State *s)
{
    this->current = s;
}

void hld::hld::setCurrentStatename(std::string s)
{
    this->currentStatename = s;
}

bool hld::hld::detectlowlvl()
{
    return (this->lld.detectDriver());
}

void hld::hld::executeCB(const using_urdf::communicatorGoalConstPtr &goal)
{
    this->goalptr = goal;
    if (this->getCurrentStatename() == "READY")
    {
        this->setReceived(true);
        this->setCurrent(new Ready(this));
    }
    
}

void hld::hld::startListening()
{
    as_.start();
}

void hld::hld::stopListening()
{
    as_.shutdown();
}

void hld::hld::setAborted()
{
    as_.setAborted(result_.result);
}

void hld::hld::setPreempted()
{
    as_.setPreempted();
}

void hld::hld::setSucceeded()
{
    as_.setSucceeded(result_.result);
}
void hld::hld::publishFeedback()
{
    as_.publishFeedback(feedback_.feedback);
}

bool hld::hld::isPreempt()
{
    return as_.isPreemptRequested();
}

void hld::hld::setReceived(const bool inputreceived)
{
    this->received = inputreceived;
}

const bool &hld::hld::getReceived()
{
    return this->received;
}

const std::string &hld::hld::getActionName()
{
    return this->action_name_;
}

const std::string &hld::hld::getPosName()
{
    return this->pose_name;
}

const std::string &hld::hld::getCurrentStatename()
{
    return this->currentStatename;
}

void hld::hld::setFeedback(int16_t angle, uint16_t time)
{
    feedback_.feedback.current_angle = angle;
    feedback_.feedback.current_time = time;
}
void hld::hld::setResult()
{
    this->result_.result.end_angle = this->feedback_.feedback.current_angle;
    this->result_.result.end_time = this->feedback_.feedback.current_time;
}

void hld::hld::sendActionThread(hld *m, const using_urdf::communicatorGoalConstPtr &goal)
{
    
}