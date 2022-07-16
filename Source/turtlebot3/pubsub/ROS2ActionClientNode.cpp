// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "ROS2ActionClientNode.h"

#include "Kismet/GameplayStatics.h"
#include "rclcUtilities.h"

AROS2ActionClientNode::AROS2ActionClientNode()
{
    PrimaryActorTick.bCanEverTick = true;
}

void AROS2ActionClientNode::BeginPlay()
{
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

void AROS2ActionClientNode::SetGoalCallback(UROS2GenericAction* InAction)
{
    UROS2FibonacciAction* FibonacciAction = Cast<UROS2FibonacciAction>(InAction);
    FROSFibonacci_SendGoal_Request goaRequest;
    // unique_identifier_msgs__msg__UUID goal_id;
    for (int i = 0; i < 16; i++)
    {
        goaRequest.goal_id.Add(0);
    }
    goaRequest.order = Order;
    FibonacciAction->SetGoalRequest(goaRequest);

    // Log request and response
    UE_LOG(LogTurtlebot3, Log, TEXT("[%s][%s][C++][update set goal] order: %i"), *GetName(), *ActionName, Order);
}

void AROS2ActionClientNode::FeedbackCallback(UROS2GenericAction* InAction)
{
    UROS2FibonacciAction* FibonacciAction = Cast<UROS2FibonacciAction>(InAction);
    FROSFibonacci_FeedbackMessage feedback;
    for (int i = 0; i < 16; i++)
    {
        feedback.goal_id.Add(0);
    }
    FibonacciAction->GetFeedback(feedback);

    UE_LOG(LogTurtlebot3,
           Log,
           TEXT("[%s][%s][C++][received feedback callback] last element of feedback sequence: %d"),
           *GetName(),
           *ActionName,
           feedback.sequence.Last(0));
}

void AROS2ActionClientNode::ResultCallback(UROS2GenericAction* InAction)
{
    UROS2FibonacciAction* FibonacciAction = Cast<UROS2FibonacciAction>(InAction);
    FROSFibonacci_GetResult_Response resultResponse;
    FibonacciAction->GetResultResponse(resultResponse);

    // Log request and response
    FString resultString;
    for (int s : resultResponse.sequence)
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
    FROSFibonacci_SendGoal_Response goalResponse;
    FibonacciAction->GetGoalResponse(goalResponse);

    if (!goalResponse.accepted)
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
