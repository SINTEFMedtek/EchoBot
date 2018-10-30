#include "cxRobotTrackingSystemService.h"
#include <iostream>

#include "cxRobotTool.h"

std::vector<ToolPtr> toVector(std::map<QString, RobotToolPtr> map)
{
    std::vector<ToolPtr> retval;
    std::map<QString, RobotToolPtr>::iterator it = map.begin();
    for(; it!= map.end(); ++it)
    {
        retval.push_back(it->second);
    }
    return retval;
}

RobotTrackingSystemService::RobotTrackingSystemService(RobotInterfacePtr robot) :
    mState(Tool::tsNONE),
    mTimer(0),
    mRobotInterface(robot),
    tps(0.05),
    isRobotTrackingEnabled(false)
{
    if(robot == NULL)
        return;

    connect(mRobotInterface.get(), &RobotInterface::startTracking, this, &RobotTrackingSystemService::startTracking);
    connect(mRobotInterface.get(), &RobotInterface::stopTracking, this, &RobotTrackingSystemService::stopTracking);
    connect(mRobotInterface.get(), &RobotInterface::addRobotVisualizationLinks, this, &RobotTrackingSystemService::addLinksSlot);
    connect(mRobotInterface.get(), &RobotInterface::removeRobotVisualizationLinks, this, &RobotTrackingSystemService::removeLinksSlot);
}

RobotTrackingSystemService::~RobotTrackingSystemService()
{
    this->deconfigure();
}

Tool::State RobotTrackingSystemService::getState() const
{
    return mState;
}

void RobotTrackingSystemService::setState(const Tool::State val)
{
    if (mState==val || this->isRobotTrackingEnabled==false)
        return;

    if (val > mState) // up
    {
        if (val == Tool::tsTRACKING)
            this->startTracking();
        else if (val == Tool::tsINITIALIZED)
            this->initialize();
        else if (val == Tool::tsCONFIGURED)
            this->configure();
    }
    else // down
    {
        if (val == Tool::tsINITIALIZED)
            this->stopTracking();
        else if (val == Tool::tsCONFIGURED)
            this->uninitialize();
        else if (val == Tool::tsNONE)
            this->deconfigure();
    }
}

void RobotTrackingSystemService::startTracking()
{
    connect(&mRobotInterface.get()->robot, &corah::Robot::stateUpdated, this, &RobotTrackingSystemService::receiveTransform);
    this->isRobotTrackingEnabled = true;
}

void RobotTrackingSystemService::stopTracking()
{
    disconnect(&mRobotInterface.get()->robot, &corah::Robot::stateUpdated, this, &RobotTrackingSystemService::receiveTransform);
    this->isRobotTrackingEnabled = false;
}

std::vector<ToolPtr> RobotTrackingSystemService::getTools()
{
    return toVector(mTools);
}

TrackerConfigurationPtr RobotTrackingSystemService::getConfiguration()
{
    TrackerConfigurationPtr retval;
    return retval;
}

ToolPtr RobotTrackingSystemService::getReference()
{
    return mReference;
}


void RobotTrackingSystemService::internalSetState(Tool::State state)
{
    mState = state;
    emit stateChanged();
}

void RobotTrackingSystemService::serverIsConnected()
{
    this->internalSetState(Tool::tsINITIALIZED);
}

void RobotTrackingSystemService::serverIsDisconnected()
{
    this->internalSetState(Tool::tsCONFIGURED);
}

void RobotTrackingSystemService::receiveTransform()
{
    corah::RobotState state = mRobotInterface->robot.getCurrentState();

    mRobotTool->toolTransformAndTimestampSlot(state.bMee,state.timestamp);
}

RobotToolPtr RobotTrackingSystemService::getTool(QString devicename)
{
    RobotToolPtr retval;
    std::map<QString, RobotToolPtr>::iterator it = mTools.find(devicename);
    if(it == mTools.end())
    {
        retval = RobotToolPtr(new RobotTool(devicename, mRobotInterface, mServices));
        mTools[devicename] = retval;
        //todo: will this work?
        emit stateChanged();
    }
    else
    {
        retval = it->second;
    }

    return retval;
}

void RobotTrackingSystemService::addLinksSlot()
{
    if(this->isRobotTrackingEnabled)
        mRobotTool->addRobotActors();
}

void RobotTrackingSystemService::removeLinksSlot()
{
    mRobotTool->removeActors();
}