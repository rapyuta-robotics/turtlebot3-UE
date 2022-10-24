// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "ROS2ActionClientNode.h"

// UE
#include "Kismet/GameplayStatics.h"

// rclUE
#include "rclcUtilities.h"

// RapyutaSimulationPlugins
#include "Core/RRCoreUtils.h"

// Turtlebot3_UE
#include "turtlebot3/Turtlebot3.h"

void AROS2ActionClientNode::BeginPlay()
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
    FibonacciActionClient = NewObject<UROS2ActionClient>(this);
    FibonacciActionClient->RegisterComponent();
    FibonacciActionClient->ActionName = ActionName;
    FibonacciActionClient->ActionClass = UROS2FibonacciAction::StaticClass();

    // bound callback function
    FActionCallback SetGoal;
    FActionCallback Feedback;
    FActionCallback Result;
    FActionCallback GoalResponse;
    FSimpleCallback Cancel;

    SetGoal.BindDynamic(this, &AROS2ActionClientNode::SetGoalCallback);
    Feedback.BindDynamic(this, &AROS2ActionClientNode::FeedbackCallback);
    Result.BindDynamic(this, &AROS2ActionClientNode::ResultCallback);
    GoalResponse.BindDynamic(this, &AROS2ActionClientNode::GoalResponseCallback);
    Cancel.BindDynamic(this, &AROS2ActionClientNode::CancelCallback);
    FibonacciActionClient->SetDelegates(SetGoal, Feedback, Result, GoalResponse, Cancel);

    // Add action Client to ROS2Node
    AddActionClient(FibonacciActionClient);

    GetWorld()->GetTimerManager().SetTimer(ActionTimerHandle, this, &AROS2ActionClientNode::SendGoal, 1.f, true);
}

void AROS2ActionClientNode::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    if (false == URRCoreUtils::IsROS2SystemEnabled(this))
    {
        return;
    }
    Super::EndPlay(EndPlayReason);
}

void AROS2ActionClientNode::Tick(float DeltaTime)
{
    if (false == URRCoreUtils::IsROS2SystemEnabled(this))
    {
        return;
    }
    Super::Tick(DeltaTime);
}

void AROS2ActionClientNode::SetGoalCallback(UROS2GenericAction* InAction)
{
    UROS2FibonacciAction* FibonacciAction = Cast<UROS2FibonacciAction>(InAction);
    FROSFibonacciSGReq goalRequest;
    goalRequest.Order = Order;
    FibonacciAction->SetGoalRequest(goalRequest);

    // Log request and response
    UE_LOG(LogTurtlebot3, Log, TEXT("[%s][%s][C++][update set goal] order: %i"), *GetName(), *ActionName, Order);
}

void AROS2ActionClientNode::FeedbackCallback(UROS2GenericAction* InAction)
{
    UROS2FibonacciAction* FibonacciAction = Cast<UROS2FibonacciAction>(InAction);
    FROSFibonacciFB feedback;
    FibonacciAction->GetFeedback(feedback);

    UE_LOG(LogTurtlebot3,
           Log,
           TEXT("[%s][%s][C++][received feedback callback] last element of feedback sequence: %d"),
           *GetName(),
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
    UE_LOG(
        LogTurtlebot3, Log, TEXT("[%s][%s][C++][received result callback] result is: %s"), *GetName(), *ActionName, *resultString);

    Order++;
    GetWorld()->GetTimerManager().SetTimer(ActionTimerHandle, this, &AROS2ActionClientNode::SendGoal, 1.f, true);
}

void AROS2ActionClientNode::GoalResponseCallback(UROS2GenericAction* InAction)
{
    UROS2FibonacciAction* FibonacciAction = Cast<UROS2FibonacciAction>(InAction);
    FROSFibonacciSGRes goalResponse;
    FibonacciAction->GetGoalResponse(goalResponse);

    if (!goalResponse.bAccepted)
    {
        UE_LOG(LogTurtlebot3,
               Log,
               TEXT("[%s][%s][C++][receive goal response callback] goal request is rejected."),
               *GetName(),
               *ActionName);
    }
    else
    {
        UE_LOG(LogTurtlebot3,
               Log,
               TEXT("[%s][%s][C++][receive goal response callback] goal request is accepted."),
               *GetName(),
               *ActionName);
        FibonacciActionClient->GetResultRequest();
    }
}

void AROS2ActionClientNode::CancelCallback()
{
    // Log request and response
    UE_LOG(LogTurtlebot3, Log, TEXT("[%s][%s][C++][received cancel response callback]"), *GetName(), *ActionName);
}

void AROS2ActionClientNode::SendGoal()
{
    if (FibonacciActionClient->UpdateAndSendGoal())
    {
        GetWorld()->GetTimerManager().ClearTimer(ActionTimerHandle);
    }
}
