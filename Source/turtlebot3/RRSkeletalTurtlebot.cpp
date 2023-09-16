// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.
#include "RRSkeletalTurtlebot.h"

// RapyutaSimulationPlugins
#include "Robots/Turtlebot3/RRTurtlebotROS2Interface.h"

// Turtlebot3
#include "Turtlebot3.h"

ARRSkeletalTurtlebot::ARRSkeletalTurtlebot()
{
    SetupDefaultTurtlebot();
}

ARRSkeletalTurtlebot::ARRSkeletalTurtlebot(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
    SetupDefaultTurtlebot();
}

void ARRSkeletalTurtlebot::SetupDefaultTurtlebot()
{
    ROS2InterfaceClass = URRTurtlebotROS2Interface::StaticClass();
}

void ARRSkeletalTurtlebot::PreInitializeComponents()
{
    // Super's components created here (like SkeletalMeshBody) rely on JSON params
    // thus must be after [InitPropertiesFromJSON()]
    Super::PreInitializeComponents();

    // 1- Init ROS-based Properties (NOTE: some of which are read by [InitPropertiesFromJSON()] ABOVE)
    // -> Must be after [ARRBaseRobot::PreInitializeComponents()], which calls [CreateROS2Interface()] &
    // before [Super::PostInitializeComponents()], spawning default controller, triggering OnPossess() -> [InitROS2Interface()]
    InitROSBasedProperties();
}

void ARRSkeletalTurtlebot::InitROSBasedProperties()
{
    if (ROS2Interface)
    {
        ROS2Interface->bPublishOdom = true;
        UE_LOG_WITH_INFO_SHORT_NAMED(LogTurtlebot3,
                                     Log,
                                     TEXT("bPublishOdom %d bPublishOdomTf %d OdomPublicationFrequencyHz %f"),
                                     ROS2Interface->bPublishOdom,
                                     ROS2Interface->bPublishOdomTf,
                                     ROS2Interface->OdomPublicationFrequencyHz);

        // Init [TurtlebotROS2Interface] & its ROS props
        TurtlebotROS2Interface = Cast<URRTurtlebotROS2Interface>(ROS2Interface);
    }
    else
    {
        // ELSE: This could only be a BP robot & Editor operations might be being done
        RR_VERIFY_STATIC_BP_ROBOT(this);
    }
}