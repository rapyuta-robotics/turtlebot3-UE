// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "ROS2ServiceClientNode.h"

#include <Engine/World.h>
#include <TimerManager.h>

// RapyutaSimulationPlugins

AROS2ServiceClientNode::AROS2ServiceClientNode()
{
    Node = CreateDefaultSubobject<UROS2Node>(TEXT("ROS2NodeComponent"));
    // Node->RegisterComponent();

    // these parameters can be change from BP
    Node->Name = TEXT("service_client_node");
    Node->Namespace = TEXT("cpp");
}

void AROS2ServiceClientNode::BeginPlay()
{
    Super::BeginPlay();
    Node->Init();

    // Create Service client
    ROS2_CREATE_SERVICE_CLIENT_WITH_QOS(Node,
                                        this,
                                        ServiceName,
                                        UROS2AddTwoIntsSrv::StaticClass(),
                                        &AROS2ServiceClientNode::ReceiveResponse,    // Callback for response
                                        UROS2QoS::DynamicBroadcaster,
                                        AddTwoIntsSrvClient)

    // set timer to periodically calling service.
    FTimerDelegate sendRequest = FTimerDelegate::CreateLambda(
        [this]
        {
            // Create request
            FROSAddTwoIntsReq req;
            req.A = A++;
            req.B = B++;
            CastChecked<UROS2AddTwoIntsSrv>(AddTwoIntsSrvClient->Service)->SetRequest(req);

            // SendRequest
            AddTwoIntsSrvClient->SendRequest();

            // Log request
            UE_LOG(LogTurtlebot3, Log, TEXT("[%s][%s][C++][send request] a:%d, b:%d"), *GetName(), *ServiceName, req.A, req.B);
        });

    GetWorld()->GetTimerManager().SetTimer(TimerHandle, sendRequest, 1.0f, true);
}

void AROS2ServiceClientNode::ReceiveResponse(UROS2GenericSrv* InService)
{
    UROS2AddTwoIntsSrv* AddTwoIntsService = Cast<UROS2AddTwoIntsSrv>(InService);

    FROSAddTwoIntsRes res;
    AddTwoIntsService->GetResponse(res);

    // Log response
    UE_LOG(LogTurtlebot3, Log, TEXT("[%s][%s][C++][receive response] sum:%d"), *GetName(), *ServiceName, res.Sum);
}
