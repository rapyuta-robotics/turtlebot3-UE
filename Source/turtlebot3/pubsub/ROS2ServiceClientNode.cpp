// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "ROS2ServiceClientNode.h"

// UE
#include "Engine/World.h"
#include "TimerManager.h"

// RapyutaSimulationPlugins
#include "Core/RRCoreUtils.h"

// Turtlebot3_UE
#include "turtlebot3/Turtlebot3.h"

void AROS2ServiceClientNode::BeginPlay()
{
    if (false == URRCoreUtils::IsROS2SystemEnabled(this))
    {
        UE_LOG(LogTurtlebot3, Error, TEXT("[%s]ROS2 is not enabled in ARRROS2GameMode"), *GetName());
        PrimaryActorTick.bCanEverTick = false;
        return;
    }

    Super::BeginPlay();
    Init();

    // Create and set parameters
    AddTwoIntsSrvClient = NewObject<UROS2ServiceClient>(this);
    AddTwoIntsSrvClient->RegisterComponent();
    AddTwoIntsSrvClient->ServiceName = ServiceName;
    AddTwoIntsSrvClient->SrvClass = UROS2AddTwoIntsSrv::StaticClass();

    // Bind functions to delegates
    // UROS2ServiceClient::UpdateAndSendRequest will call bounded method.
    AddTwoIntsSrvClient->RequestDelegate.BindDynamic(this, &AROS2ServiceClientNode::SendRequest);

    // Bounded method will be called when receive response
    AddTwoIntsSrvClient->ResponseDelegate.BindDynamic(this, &AROS2ServiceClientNode::ReceiveResponse);

    // Add service to ROSNode(this_)
    AddServiceClient(AddTwoIntsSrvClient);

    AddTwoIntsSrvClient->Init(UROS2QoS::DynamicBroadcaster);

    // set timer to periodically calling service.
    GetWorld()->GetTimerManager().SetTimer(
        TimerHandle, this->AddTwoIntsSrvClient, &UROS2ServiceClient::UpdateAndSendRequest, 1.f, true);
}

void AROS2ServiceClientNode::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    if (false == URRCoreUtils::IsROS2SystemEnabled(this))
    {
        return;
    }
    Super::EndPlay(EndPlayReason);
}

void AROS2ServiceClientNode::Tick(float DeltaTime)
{
    if (false == URRCoreUtils::IsROS2SystemEnabled(this))
    {
        return;
    }
    Super::Tick(DeltaTime);
}

void AROS2ServiceClientNode::SendRequest(UROS2GenericSrv* InService)
{
    // Create and update request
    FROSAddTwoIntsReq req;
    req.A = A++;
    req.B = B++;
    CastChecked<UROS2AddTwoIntsSrv>(InService)->SetRequest(req);

    // Log request
    UE_LOG(LogTurtlebot3, Log, TEXT("[%s][%s][C++][send request] a:%d, b:%d"), *GetName(), *ServiceName, req.A, req.B);
}

void AROS2ServiceClientNode::ReceiveResponse(UROS2GenericSrv* InService)
{
    UROS2AddTwoIntsSrv* AddTwoIntsService = Cast<UROS2AddTwoIntsSrv>(InService);

    FROSAddTwoIntsRes res;
    AddTwoIntsService->GetResponse(res);

    // Log response
    UE_LOG(LogTurtlebot3, Log, TEXT("[%s][%s][C++][receive response] sum:%d"), *GetName(), *ServiceName, res.Sum);
}
