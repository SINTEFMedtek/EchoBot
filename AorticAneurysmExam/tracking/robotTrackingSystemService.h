#ifndef CXROBOTTRACKINGSYSTEMSERVICE_H
#define CXROBOTTRACKINGSYSTEMSERVICE_H

#include "RobotTool.h"
#include "../RobotInterface.h"

/**
 * Implementation of Robot tracking service.
 *
 *
 * \author Andreas Østvik
 */


typedef std::shared_ptr<class RobotTrackingSystemService> RobotTrackingSystemServicePtr;

class RobotTrackingSystemService
{

public:
    RobotTrackingSystemService(RobotInterfacePtr robotInterface);
    virtual ~RobotTrackingSystemService();

    virtual Tool::State getState() const;
    virtual void setState(const Tool::State val);

    virtual std::vector<ToolPtr> getTools();
    virtual TrackerConfigurationPtr getConfiguration();
    ToolPtr getReference(); /// System fails if several TrackingSystemServices define this tool

    virtual void setLoggingFolder(QString loggingFolder); ///<\param loggingFolder path to the folder where logs should be saved

signals:
    void connectToServer();
    void disconnectFromServer();

private slots:
    void configure(); ///< sets up the software
    virtual void deconfigure(); ///< deconfigures the software
    void initialize(); ///< connects to the hardware
    void uninitialize(); ///< disconnects from the hardware
    void startTracking(); ///< starts tracking
    void stopTracking(); ///< stops tracking

    void serverIsConnected();
    void serverIsDisconnected();
    void serverIsConfigured();
    void serverIsDeconfigured();

    void receiveTransform();

    void addLinksSlot();
    void removeLinksSlot();

private:
    void internalSetState(Tool::State state);
    RobotToolPtr getTool(QString devicename);

    Tool::State mState;
    std::map<QString, RobotToolPtr> mTools;
    ToolPtr mReference;

    RobotInterfacePtr mRobotInterface;
    VisServicesPtr mServices;

    RobotToolPtr mRobotTool;

    double mTimer;
    double tps;
    bool isRobotTrackingEnabled;
};


#endif // ROBOTTRACKINGSYSTEMSERVICE_H
