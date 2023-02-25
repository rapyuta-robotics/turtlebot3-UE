// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "ROS2ActionClientNode.h"

#include "Kismet/GameplayStatics.h"
#include "rclcUtilities.h"

AROS2ActionClientNode::AROS2ActionClientNode()
{
    Node = CreateDefaultSubobject<UROS2NodeComponent>(TEXT("ROS2NodeComponent"));

    // these parameters can be change from BP
    Node->Name = TEXT("action_client_node");
    Node->Namespace = TEXT("cpp");
}

void AROS2ActionClientNode::BeginPlay()
{
    Super::BeginPlay();
    Node->Init();

    ROS2_CREATE_ACTION_CLIENT(Node,
                              this,
                              ActionName,
                              UROS2FibonacciAction::StaticClass(),
                              &AROS2ActionClientNode::GoalResponseCallback,
                              &AROS2ActionClientNode::ResultCallback,
                              &AROS2ActionClientNode::FeedbackCallback,
                              &AROS2ActionClientNode::CancelCallback,
                              FibonacciActionClient);

    SendGoal();
}

void AROS2ActionClientNode::SendGoal()
{
    // Create goal
    UROS2FibonacciAction* FibonacciAction = Cast<UROS2FibonacciAction>(FibonacciActionClient->Action);
    FROSFibonacciSGReq goalRequest;
    goalRequest.Order = Order;
    FibonacciAction->SetGoalRequest(goalRequest);

    // send goal
    if (!FibonacciActionClient->SendGoal())
    {
        // if it failes, retry after 1s
        UE_LOG_WITH_INFO_NAMED(
            LogTurtlebot3, Warning, TEXT("[%s][C++][send goal] failed to sendo goal. retry in 1s..."), *ActionName);
        GetWorld()->GetTimerManager().SetTimer(ActionTimerHandle, this, &AROS2ActionClientNode::SendGoal, 1.f, false);
    }
    else
    {
        UE_LOG_WITH_INFO_NAMED(LogTurtlebot3, Log, TEXT("[%s][C++][send goal] order: %i"), *ActionName, Order);
    }
}

void AROS2ActionClientNode::FeedbackCallback(UROS2GenericAction* InAction)
{
    UROS2FibonacciAction* FibonacciAction = Cast<UROS2FibonacciAction>(InAction);
    FROSFibonacciFB feedback;
    FibonacciAction->GetFeedback(feedback);

    UE_LOG_WITH_INFO_NAMED(LogTurtlebot3,
                           Log,
                           TEXT("[%s][C++][received feedback callback] last element of feedback sequence: %d"),
                           *ActionName,
                           feedback.Sequence.Last(0));
}

void AROS2ActionClientNode::ResultCallback(UROS2GenericAction* InAction)
{
    UROS2FibonacciAction* FibonacciAction = Cast<UROS2FibonacciAction>(InAction);
    FROSFibonacciGRRes resultResponse;
    FibonacciAction->GetResultResponse(resultResponse);

    // Log request and response
    FString resultString;
    for (int s : resultResponse.Sequence)
    {
        resultString += FString::FromInt(s) + ", ";
    }
    UE_LOG_WITH_INFO_NAMED(
        LogTurtlebot3, Log, TEXT("[%s][C++][received result callback] result is: %s"), *ActionName, *resultString);

    // update order and send next goal
    Order++;
    SendGoal();
}

void AROS2ActionClientNode::GoalResponseCallback(UROS2GenericAction* InAction)
{
    UROS2FibonacciAction* FibonacciAction = Cast<UROS2FibonacciAction>(InAction);
    FROSFibonacciSGRes goalResponse;
    FibonacciAction->GetGoalResponse(goalResponse);

    if (!goalResponse.bAccepted)
    {
        UE_LOG_WITH_INFO_NAMED(LogTurtlebot3,
                               Warning,
                               TEXT("[%s][C++][receive goal response callback] goal request is rejected. retry in 1s..."),
                               *ActionName);
    }
    else
    {
        UE_LOG_WITH_INFO_NAMED(
            LogTurtlebot3, Log, TEXT("[%s][C++][receive goal response callback] goal request is accepted."), *ActionName);
        FibonacciActionClient->SendResultRequest();
    }
}

void AROS2ActionClientNode::CancelCallback()
{
    int cancelResult = FibonacciActionClient->Action->GetCancelResponseReturnCode();
    if (cancelResult != FROSCancelGoalRes::ERROR_NONE)
    {
        UE_LOG_WITH_INFO_NAMED(
            LogTurtlebot3, Log, TEXT("[%s][C++][received cancel response callback] failed to cancel action"), *ActionName);
    }
    else
    {
        UE_LOG_WITH_INFO_NAMED(
            LogTurtlebot3, Log, TEXT("[%s][C++][received cancel response callback] succeeded to cancel action"), *ActionName);
    }
}
