// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "ROS2ActionServerNode.h"

#include "Kismet/GameplayStatics.h"
#include "rclcUtilities.h"

AROS2ActionServerNode::AROS2ActionServerNode()
{
    PrimaryActorTick.bCanEverTick = true;
}

void AROS2ActionServerNode::BeginPlay()
{
    Super::BeginPlay();
    Init();

    // Create and set parameters
    FibonacciActionServer = NewObject<UROS2ActionServer>(this);
    FibonacciActionServer->RegisterComponent();
    FibonacciActionServer->ActionName = ActionName;
    FibonacciActionServer->ActionClass = UROS2FibonacciAction::StaticClass();

    // bound callback function
    FActionCallback UpdateFeedbackDelegate;
    FActionCallback UpdateResultDelegate;
    FActionGoalCallback HandleGoalDelegate;
    FSimpleCallback HandleCancelDelegate;
    FSimpleCallback HandleAcceptedDelegate;

    UpdateFeedbackDelegate.BindDynamic(this, &AROS2ActionServerNode::UpdateFeedbackCallback);
    UpdateResultDelegate.BindDynamic(this, &AROS2ActionServerNode::UpdateResultCallback);
    HandleGoalDelegate.BindDynamic(this, &AROS2ActionServerNode::HandleGoalCallback);
    HandleCancelDelegate.BindDynamic(this, &AROS2ActionServerNode::HandleCancelCallback);
    HandleAcceptedDelegate.BindDynamic(this, &AROS2ActionServerNode::HandleAcceptedCallback);
    FibonacciActionServer->SetDelegates(
        UpdateFeedbackDelegate, UpdateResultDelegate, HandleGoalDelegate, HandleCancelDelegate, HandleAcceptedDelegate);

    // FeedbackMsg.goal_id.Reserve(16);
    // GoalRequest.goal_id.Reserve(16);
    for (int i = 0; i < 16; i++)
    {
        FeedbackMsg.goal_id.Add(0);
        GoalRequest.goal_id.Add(0);
    }

    // Add action server to ROS2Node
    AddActionServer(FibonacciActionServer);
}

void AROS2ActionServerNode::UpdateFeedbackCallback(UROS2GenericAction* InAction)
{
    UROS2FibonacciAction* FibonacciAction = Cast<UROS2FibonacciAction>(InAction);

    // Calculate fibonacci and send feedoback

    // send result when finish by UpdateAndSendResult
    if (Count++ < GoalRequest.order)
    {
        FeedbackMsg.sequence.Add(FeedbackMsg.sequence[Count] - FeedbackMsg.sequence[Count - 1]);
        // Log request and response
        UE_LOG(LogTurtlebot3,
               Log,
               TEXT("[%s][%s][C++][update feedback callback] added %d"),
               *GetName(),
               *ActionName,
               FeedbackMsg.sequence[Count + 1]);
    }
    else
    {
        FibonacciActionServer->UpdateAndSendResult();
    }
}

void AROS2ActionServerNode::UpdateResultCallback(UROS2GenericAction* InAction)
{
    UROS2FibonacciAction* FibonacciAction = Cast<UROS2FibonacciAction>(InAction);

    // set result
    FROSFibonacci_GetResult_Response ResultResponse;
    ResultResponse.status = GOAL_STATE_SUCCEEDED;
    ResultResponse.sequence = FeedbackMsg.sequence;
    FibonacciAction->GetResultResponse(ResultResponse);

    // stop timer
    GetWorld()->GetTimerManager().ClearTimer(ActionTimerHandle);

    // Log request and response
    UE_LOG(LogTurtlebot3, Log, TEXT("[%s][%s][C++][update result callback]"), *GetName(), *ActionName);
}

bool AROS2ActionServerNode::HandleGoalCallback(UROS2GenericAction* InAction)
{
    UROS2FibonacciAction* FibonacciAction = Cast<UROS2FibonacciAction>(InAction);

    // set and send goal response
    FROSFibonacci_SendGoal_Response goalResponse;
    goalResponse.accepted = true;
    builtin_interfaces__msg__Time stamp =
        UROS2Utils::FloatToROSStamp(UGameplayStatics::GetTimeSeconds(reinterpret_cast<UObject*>(GetWorld())));
    goalResponse.stamp_sec = stamp.sec;
    goalResponse.stamp_nanosec = stamp.nanosec;
    FibonacciAction->SetGoalResponse(goalResponse);

    FibonacciActionServer->SendGoalResponse();

    // Log request and response
    UE_LOG(LogTurtlebot3, Log, TEXT("[%s][%s][C++][goal callback]"), *GetName(), *ActionName);

    if (goalResponse.accepted)
    {
        FibonacciAction->GetGoalRequest(GoalRequest);
        FeedbackMsg.sequence.Empty();
        FeedbackMsg.sequence.Add(0);
        FeedbackMsg.sequence.Add(1);
        Count = 1;
        for (int i = 0; i < 16; i++)
        {
            FeedbackMsg.goal_id[0] = GoalRequest.goal_id[0];
        }
    }

    // return value is used by ROS2ActionServer to decide whether it calls accepted callback or not.
    return goalResponse.accepted;
}

void AROS2ActionServerNode::HandleCancelCallback()
{
    // stop execution timer
    GetWorld()->GetTimerManager().ClearTimer(ActionTimerHandle);

    // send cancel response
    FibonacciActionServer->ProcessAndSendCancelResponse();

    // Log request and response
    UE_LOG(LogTurtlebot3, Log, TEXT("[%s][%s][C++][cancle callback]"), *GetName(), *ActionName);
}

void AROS2ActionServerNode::HandleAcceptedCallback()
{
    Count = 0;
    // set timer to periodically calling service.
    GetWorld()->GetTimerManager().SetTimer(
        ActionTimerHandle, this->FibonacciActionServer, &UROS2ActionServer::UpdateAndSendFeedback, 1.f, true);

    // Log request and response
    UE_LOG(LogTurtlebot3, Log, TEXT("[%s][%s][C++][accepted callback] Start fibonacci calculation]"), *GetName(), *ActionName);
}
