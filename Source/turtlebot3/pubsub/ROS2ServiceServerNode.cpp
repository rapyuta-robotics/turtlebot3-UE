// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "ROS2ServiceServerNode.h"

// RapyutaSimulationPlugins
#include "Core/RRCoreUtils.h"

// Turtlebot3_UE
#include "turtlebot3/Turtlebot3.h"

void AROS2ServiceServerNode::BeginPlay()
{
    if (false == URRCoreUtils::IsROS2SystemEnabled(this))
    {
        UE_LOG(LogTurtlebot3, Error, TEXT("[%s]ROS2 is not enabled in ARRROS2GameMode"), *GetName());
        PrimaryActorTick.bCanEverTick = false;
        return;
    }

    Super::BeginPlay();
    Init();

    // bound callback function
    FServiceCallback AddTwoIntsSrvCallback;
    AddTwoIntsSrvCallback.BindDynamic(this, &AROS2ServiceServerNode::SrvCallback);

    // Add serivce server to ROS2Node
    AddServiceServer(ServiceName, UROS2AddTwoIntsSrv::StaticClass(), AddTwoIntsSrvCallback);
}

void AROS2ServiceServerNode::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    if (false == URRCoreUtils::IsROS2SystemEnabled(this))
    {
        return;
    }
    Super::EndPlay(EndPlayReason);
}

void AROS2ServiceServerNode::Tick(float DeltaTime)
{
    if (false == URRCoreUtils::IsROS2SystemEnabled(this))
    {
        return;
    }
    Super::Tick(DeltaTime);
}

void AROS2ServiceServerNode::SrvCallback(UROS2GenericSrv* InService)
{
    UROS2AddTwoIntsSrv* AddTwoIntsService = Cast<UROS2AddTwoIntsSrv>(InService);

    FROSAddTwoIntsReq req;
    AddTwoIntsService->GetRequest(req);

    // Add two ints.
    FROSAddTwoIntsRes res;
    res.Sum = req.A + req.B;

    // Set response.
    AddTwoIntsService->SetResponse(res);

    // Log request and response
    UE_LOG(
        LogTurtlebot3, Log, TEXT("[%s][%s][C++][receive request] %d + %d = %d"), *GetName(), *ServiceName, req.A, req.B, res.Sum);
}
