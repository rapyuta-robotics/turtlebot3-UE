// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "ROS2ServiceServerNode.h"

AROS2ServiceServerNode::AROS2ServiceServerNode()
{
    PrimaryActorTick.bCanEverTick = true;
}

void AROS2ServiceServerNode::BeginPlay()
{
    Super::BeginPlay();
    Init();

    FServiceCallback AddTwoIntsSrvCallback;
    AddTwoIntsSrvCallback.BindDynamic(this, &AROS2ServiceServerNode::SrvCallback);
    AddServiceServer(ServiceName, UROS2AddTwoIntsSrv::StaticClass(), AddTwoIntsSrvCallback);
}

void AROS2ServiceServerNode::SrvCallback(UROS2GenericSrv* Service)
{
    UROS2AddTwoIntsSrv* AddTwoIntsService = Cast<UROS2AddTwoIntsSrv>(Service);

    FROSAddTwoInts_Request req;
    AddTwoIntsService->GetRequest(req);

    FROSAddTwoInts_Response res;
    res.sum = req.a + req.b;

    AddTwoIntsService->SetResponse(res);

    UE_LOG(
        LogTurtlebot3, Log, TEXT("[%s][%s][C++][receive request] %d + %d = %d"), *GetName(), *ServiceName, req.a, req.b, res.sum);
}
