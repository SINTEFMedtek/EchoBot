#ifndef CXROBOTTOOL_H
#define CXROBOTTOOL_H


#include <QTimer>

#include "cxRobotInterface.h"


/**
 * Implementation of Robot Tool (End-effector).
 *
 * \ingroup org_custusx_robot_ur5
 *
 * \author Andreas Østvik
 */

typedef std::shared_ptr<class RobotTool> RobotToolPtr;

class RobotTool
{
    friend class RobotTrackingSystemService;
    Q_OBJECT

public:
    RobotTool(RobotInterfacePtr robot);
    virtual ~RobotTool();

    virtual std::set<Type> getTypes() const;

    virtual vtkPolyDataPtr getGraphicsPolyData() const;
    virtual Transform3D get_prMt() const;
    virtual bool getVisible() const;
    virtual bool isCalibrated() const;
    virtual double getTimestamp() const;
    virtual double getTooltipOffset() const;
    virtual void setTooltipOffset(double val);

    virtual Transform3D getCalibration_sMt() const;

    virtual bool isInitialized() const;

    virtual void setVisible(bool vis);

    void addRobotActors();
    void removeActors();

private slots:
    void toolTransformAndTimestampSlot(Transform3D bMe, double timestamp);
    void calculateTpsSlot();
    void toolVisibleSlot(bool);

private:
    void createPolyData();
    void initiateActors();
    void updateActors();

    std::set<Type> mTypes;

    vtkPolyDataPtr mPolyData;

    QTimer mTpsTimer;
    RobotInterfacePtr mRobotInterface;
    VisServicesPtr mServices;
    double mTimestamp;

    std::set<Type> determineTypesBasedOnUid(const QString uid) const;
    QString mGraphicsFolderName;

    Transform3D get_rMb();

    vtkActorPtr vtkSourceToActor(QString filename);
    vtkActorPtr eeActor, baseActor, link1Actor, link2Actor, link3Actor, link4Actor, link5Actor;

    bool isRobotLinksVisualized;
};

} /* namespace cx */

#endif // CXROBOTTOOL_H
