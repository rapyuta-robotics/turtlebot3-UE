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

    // Add action server to ROS2Node
    AddActionServer(FibonacciActionServer);
}

void AROS2ActionServerNode::UpdateFeedbackCallback(UROS2GenericAction* InAction)
{
    UROS2FibonacciAction* FibonacciAction = Cast<UROS2FibonacciAction>(InAction);

    // Calculate fibonacci and send feedoback

    // send result when finish by UpdateAndSendResult
    if (Count++ < GoalRequest.Order)
    {
        FeedbackMsg.Sequence.Add(FeedbackMsg.Sequence[Count] + FeedbackMsg.Sequence[Count - 1]);
        FibonacciAction->SetFeedback(FeedbackMsg);
        // Log request and response
        UE_LOG(LogTurtlebot3,
               Log,
               TEXT("[%s][%s][C++][update feedback callback] added %d"),
               *GetName(),
               *ActionName,
               FeedbackMsg.Sequence.Last(0));
    }
    else
    {
        FibonacciActionServer->UpdateAndSendResult();
    }
}

void AROS2ActionServerNode::UpdateResultCallback(UROS2GenericAction* InAction)
{
    UROS2FibonacciAction* FibonacciAction = Cast<UROS2FibonacciAction>(InAction);

    // for log
    FString resultString;

    // set result
    FROSFibonacciGetResultResponse ResultResponse;
    ResultResponse.Status = GOAL_STATE_SUCCEEDED;
    for (auto s : FeedbackMsg.Sequence)
    {
        ResultResponse.Sequence.Add(s);
        resultString += FString::FromInt(s) + ", ";
    }
    FibonacciAction->SetResultResponse(ResultResponse);

    // stop timer
    GetWorld()->GetTimerManager().ClearTimer(ActionTimerHandle);

    // Log request and response
    UE_LOG(LogTurtlebot3, Log, TEXT("[%s][%s][C++][update result callback] result is: %s"), *GetName(), *ActionName, *resultString);
}

bool AROS2ActionServerNode::HandleGoalCallback(UROS2GenericAction* InAction)
{
    UROS2FibonacciAction* FibonacciAction = Cast<UROS2FibonacciAction>(InAction);

    // set and send goal response
    FROSFibonacciSendGoalResponse goalResponse;
    goalResponse.bAccepted = true;
    goalResponse.Stamp = UGameplayStatics::GetTimeSeconds(reinterpret_cast<UObject*>(GetWorld()));
    FibonacciAction->SetGoalResponse(goalResponse);
    FibonacciActionServer->SendGoalResponse();

    // Log request and response
    UE_LOG(LogTurtlebot3, Log, TEXT("[%s][%s][C++][goal callback]"), *GetName(), *ActionName);

    if (goalResponse.bAccepted)
    {
        FibonacciAction->GetGoalRequest(GoalRequest);
        FibonacciAction->SetGoalIdToFeedback(FeedbackMsg);
        FeedbackMsg.Sequence.Empty();
        FeedbackMsg.Sequence.Add(0);
        FeedbackMsg.Sequence.Add(1);
        Count = 1;
    }

    // return value is used by ROS2ActionServer to decide whether it calls accepted callback or not.
    return goalResponse.bAccepted;
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
    UE_LOG(LogTurtlebot3, Log, TEXT("[%s][%s][C++][accepted callback] Start fibonacci calculation"), *GetName(), *ActionName);
}
