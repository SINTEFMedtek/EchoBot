#include "cxRobotTool.h"
#include "cxProbeImpl.h"
#include "cxLogger.h"
#include "cxPatientModelService.h"
#include <QDir>

#include <vtkActor.h>
#include <vtkSTLReader.h>
#include <vtkPolyDataMapper.h>
#include <vtkDataSet.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkTransform.h>
#include <vtkAppendPolyData.h>
#include <vtkAssembly.h>
#include <vtkRenderer.h>

#include "corah/robotics/RobotState.h"


RobotTool::RobotTool(RobotInterfacePtr robotInterface):
    mPolyData(NULL),
    mRobotInterface(robotInterface),
    mTimestamp(0),
    isRobotLinksVisualized(false),
    mGraphicsFolderName("/source/plugins/org.custusx.robotinterface/tracking/CADModel/")
{
    QDir dir = QDir::current(); dir.cdUp(); dir.cdUp();
    mGraphicsFolderName = dir.path()+mGraphicsFolderName;

    this->createPolyData();
    this->toolVisibleSlot(true);
}

RobotTool::~RobotTool()
{

}

vtkPolyDataPtr RobotTool::getGraphicsPolyData() const
{
    return mPolyData;
}

bool RobotTool::getVisible() const
{
    return true;
}

bool RobotTool::isCalibrated() const
{
    return true;
}

double RobotTool::getTimestamp() const
{
    return mTimestamp;
}

void RobotTool::toolTransformAndTimestampSlot(Transform3D bMee, double timestamp)
{
    Transform3D rMee = mRobotInterface->robot.get_rMb()*bMee;
    emit toolTransformAndTimestamp(rMee, mTimestamp);

    if(this->isRobotLinksVisualized)
        this->updateActors();
}

void RobotTool::createPolyData()
{
    if (!this->mGraphicsFolderName.isEmpty())
    {
        this->initiateActors();

        vtkSTLReaderPtr eeSTL = vtkSTLReaderPtr::New();

        eeSTL->SetFileName(cstring_cast(QString(mGraphicsFolderName + "ee.stl")));
        eeSTL->Update();

        mPolyData = eeSTL->GetOutput();
    }
    else
    {
        mPolyData = Tool::createDefaultPolyDataCone();
    }
}

void RobotTool::initiateActors()
{
    baseActor = this->vtkSourceToActor("base.stl");
    link1Actor = this->vtkSourceToActor("link1.stl");
    link2Actor = this->vtkSourceToActor("link2.stl");
    link3Actor = this->vtkSourceToActor("link3.stl");
    link4Actor = this->vtkSourceToActor("link4.stl");
    link5Actor = this->vtkSourceToActor("link5.stl");
    eeActor = this->vtkSourceToActor("ee.stl");
}

void RobotTool::addRobotActors()
{
    ViewPtr view = mServices->view()->get3DView();
    corah::RobotState state = mRobotInterface->robot.getCurrentState();

    Transform3D rMb = this->get_rMb();
    Transform3D rMl1 = this->get_rMb()*state.getTransformToJoint(1);
    Transform3D rMl2 = this->get_rMb()*state.getTransformToJoint(1);
    Transform3D rMl3 = this->get_rMb()*state.getTransformToJoint(2);
    Transform3D rMl4 = this->get_rMb()*state.getTransformToJoint(4);
    Transform3D rMl5 = this->get_rMb()*state.getTransformToJoint(5);

    baseActor->SetUserTransform(cx_transform3D_internal::getVtkTransform(&(rMb)));
    link1Actor->SetUserTransform(cx_transform3D_internal::getVtkTransform(&(rMl1)));
    link2Actor->SetPosition(0,0,121);
    link2Actor->SetUserTransform(cx_transform3D_internal::getVtkTransform(&(rMl2)));
    link3Actor->SetUserTransform(cx_transform3D_internal::getVtkTransform(&(rMl3)));
    link4Actor->SetUserTransform(cx_transform3D_internal::getVtkTransform(&(rMl4)));
    link5Actor->SetUserTransform(cx_transform3D_internal::getVtkTransform(&(rMl5)));

    view->getRenderer()->AddActor(link1Actor);
    view->getRenderer()->AddActor(link2Actor);
    view->getRenderer()->AddActor(link3Actor);
    view->getRenderer()->AddActor(link4Actor);
    view->getRenderer()->AddActor(link5Actor);
    view->getRenderer()->AddActor(baseActor);

    this->isRobotLinksVisualized = true;
}

void RobotTool::updateActors()
{
    corah::RobotState state = mRobotInterface->robot.getCurrentState();

    Transform3D rMb = this->get_rMb();
    Transform3D rMl1 = this->get_rMb()*state.getTransformToJoint(1);
    Transform3D rMl2 = this->get_rMb()*state.getTransformToJoint(1);
    Transform3D rMl3 = this->get_rMb()*state.getTransformToJoint(2);
    Transform3D rMl4 = this->get_rMb()*state.getTransformToJoint(4);
    Transform3D rMl5 = this->get_rMb()*state.getTransformToJoint(5);

    baseActor->SetUserTransform(cx_transform3D_internal::getVtkTransform(&(rMb)));
    link1Actor->SetUserTransform(cx_transform3D_internal::getVtkTransform(&(rMl1)));
    link2Actor->SetUserTransform(cx_transform3D_internal::getVtkTransform(&(rMl2)));
    link2Actor->SetOrientation(0,0, state.jointConfiguration(1)*180/M_PI+90);
    link3Actor->SetUserTransform(cx_transform3D_internal::getVtkTransform(&(rMl3)));
    link3Actor->SetOrientation(0,0, state.jointConfiguration(2)*180/M_PI);
    link4Actor->SetUserTransform(cx_transform3D_internal::getVtkTransform(&(rMl4)));
    link5Actor->SetUserTransform(cx_transform3D_internal::getVtkTransform(&(rMl5)));
}

void RobotTool::removeActors()
{
    ViewPtr view = mServices->view()->get3DView();

    view->getRenderer()->RemoveActor(link1Actor);
    view->getRenderer()->RemoveActor(link2Actor);
    view->getRenderer()->RemoveActor(link3Actor);
    view->getRenderer()->RemoveActor(link4Actor);
    view->getRenderer()->RemoveActor(link5Actor);
    view->getRenderer()->RemoveActor(baseActor);

    this->isRobotLinksVisualized = false;
}

Transform3D RobotTool::get_prMt() const
{
    return mServices->patient()->get_rMpr().inverse()*mRobotInterface->robot.get_rMb()*mRobotInterface->robot.getCurrentState().bMee;
}

Transform3D RobotTool::get_rMb()
{
    return (mRobotInterface->robot.get_rMb());
}

vtkActorPtr RobotTool::vtkSourceToActor(QString filename)
{
    vtkSTLReaderPtr source = vtkSTLReaderPtr::New();
    source->SetFileName(cstring_cast(QString(mGraphicsFolderName + filename)));

    vtkPolyDataMapperPtr mapper = vtkPolyDataMapperPtr::New();
    mapper->SetInputConnection(source->GetOutputPort());

    vtkActorPtr actor = vtkActorPtr::New();
    actor->SetMapper(mapper);

    return actor;
}
