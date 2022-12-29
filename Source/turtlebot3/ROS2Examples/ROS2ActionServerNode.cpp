// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "ROS2ActionServerNode.h"

#include "Kismet/GameplayStatics.h"
#include "rclcUtilities.h"

AROS2ActionServerNode::AROS2ActionServerNode()
{
    Node = CreateDefaultSubobject<UROS2NodeComponent>(TEXT("ROS2NodeComponent"));
    Node->RegisterComponent();

    // these parameters can be change from BP
    Node->Name = TEXT("action_server_node");
    Node->Namespace = TEXT("cpp");
}

void AROS2ActionServerNode::BeginPlay()
{
    Super::BeginPlay();
    Node->Init();

    ROS2_CREATE_ACTION_SERVER(Node,
                              this,
                              ActionName,
                              UROS2FibonacciAction::StaticClass(),
                              &AROS2ActionServerNode::GoalCallback,
                              &AROS2ActionServerNode::CancelCallback,
                              &AROS2ActionServerNode::ResultCallback,
                              FibonacciActionServer);
}

void AROS2ActionServerNode::Execute()
{
    UROS2FibonacciAction* FibonacciAction = Cast<UROS2FibonacciAction>(FibonacciActionServer->Action);

    // send feedback
    if (Count++ <= GoalRequest.Order)
    {
        FeedbackMsg.Sequence.Add(FeedbackMsg.Sequence[Count] + FeedbackMsg.Sequence[Count - 1]);
        FibonacciAction->SetFeedback(FeedbackMsg);
        // Log request and response
        UE_LOG(LogTurtlebot3,
               Log,
               TEXT("[%s][%s][C++][update feedback] added %d"),
               *GetName(),
               *ActionName,
               FeedbackMsg.Sequence.Last(0));
        FibonacciActionServer->SendFeedback();
    }
    // send result when finish by UpdateAndSendResult
    else
    {
        // for log
        FString resultString;

        // set result
        FROSFibonacciGRRes ResultResponse;
        ResultResponse.GRResStatus = GOAL_STATE_SUCCEEDED;
        for (auto s : FeedbackMsg.Sequence)
        {
            ResultResponse.Sequence.Add(s);
            resultString += FString::FromInt(s) + ", ";
        }
        FibonacciAction->SetResultResponse(ResultResponse);
        FibonacciActionServer->SendResultResponse();

        // stop timer
        GetWorld()->GetTimerManager().ClearTimer(ActionTimerHandle);

        // Log request and response
        UE_LOG(LogTurtlebot3, Log, TEXT("[%s][%s][C++][send result] result is: %s"), *GetName(), *ActionName, *resultString);
        ;
    }
}

void AROS2ActionServerNode::GoalCallback(UROS2GenericAction* InAction)
{
    // retrieve goal request value
    UROS2FibonacciAction* FibonacciAction = Cast<UROS2FibonacciAction>(InAction);
    FibonacciAction->GetGoalRequest(GoalRequest);

    // set and send goal response
    FROSFibonacciSGRes goalResponse;
    goalResponse.bAccepted = true;    // always accept goal
    goalResponse.Stamp = UGameplayStatics::GetTimeSeconds(reinterpret_cast<UObject*>(GetWorld()));
    Cast<UROS2FibonacciAction>(FibonacciActionServer->Action)->SetGoalResponse(goalResponse);
    FibonacciActionServer->SendGoalResponse();

    // Log request and response
    UE_LOG(LogTurtlebot3, Log, TEXT("[%s][%s][C++][goal callback]"), *GetName(), *ActionName);
}

void AROS2ActionServerNode::CancelCallback()
{
    // stop execution timer
    GetWorld()->GetTimerManager().ClearTimer(ActionTimerHandle);

    // send cancel response. always success
    FibonacciActionServer->ProcessAndSendCancelResponse(FROSCancelGoalRes::ERROR_NONE);

    // Log request and response
    UE_LOG(LogTurtlebot3, Log, TEXT("[%s][%s][C++][cancle callback]"), *GetName(), *ActionName);
}

void AROS2ActionServerNode::ResultCallback()
{
    // initialize feedback msg
    Cast<UROS2FibonacciAction>(FibonacciActionServer->Action)->SetGoalIdToFeedback(FeedbackMsg);
    FeedbackMsg.Sequence.Empty();
    FeedbackMsg.Sequence.Add(0);
    FeedbackMsg.Sequence.Add(1);
    Count = 0;

    // set timer to execute action.
    GetWorld()->GetTimerManager().SetTimer(ActionTimerHandle, this, &AROS2ActionServerNode::Execute, 1.f, true);

    // Log request and response
    UE_LOG(LogTurtlebot3, Log, TEXT("[%s][%s][C++][result callback] Start fibonacci calculation"), *GetName(), *ActionName);
}
