// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "ROS2ServiceServerNode.h"

AROS2ServiceServerNode::AROS2ServiceServerNode()
{
    Node = CreateDefaultSubobject<UROS2NodeComponent>(TEXT("ROS2NodeComponent"));

    // these parameters can be change from BP
    Node->Name = TEXT("service_server_node");
    Node->Namespace = TEXT("cpp");
}

void AROS2ServiceServerNode::BeginPlay()
{
    Super::BeginPlay();
    Node->Init();

    ROS2_CREATE_SERVICE_SERVER(Node, this, ServiceName, UROS2AddTwoIntsSrv::StaticClass(), &AROS2ServiceServerNode::SrvCallback);
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
