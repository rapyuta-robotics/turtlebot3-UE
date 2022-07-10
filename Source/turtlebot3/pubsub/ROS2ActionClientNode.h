/**
 * @file ROS2ActionClientNode.h
 * @brief ROS2 Service Client example class
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once
// UE
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

// rclUE
#include <Actions/ROS2FibonacciAction.h>
#include <ROS2ActionClient.h>
#include <ROS2Node.h>

// Turtlebot3_UE
#include "turtlebot3/Turtlebot3.h"

#include "ROS2ActionClientNode.generated.h"

/**
 * @brief ROS2 Service Client example class. This actor has Fibonacci action client.
 *
 */
UCLASS()
class TURTLEBOT3_API AROS2ActionClientNode : public AROS2Node
{
    GENERATED_BODY()

public:
    AROS2ActionClientNode();

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UROS2ActionClient* FibonacciActionClient = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString ActionName = TEXT("fibonacci_action");

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    int Order = 3;

protected:
    virtual void BeginPlay() override;

    UPROPERTY()
    FTimerHandle ActionTimerHandle;

    UFUNCTION()
    void SetGoalCallback(UROS2GenericAction* InAction);
    UFUNCTION()
    void FeedbackCallback(UROS2GenericAction* InAction);
    UFUNCTION()
    void ResultCallback(UROS2GenericAction* InAction);
    UFUNCTION()
    void GoalResponseCallback(UROS2GenericAction* InAction);
    UFUNCTION()
    void CancelCallback();

    UFUNCTION()
    void SendGoal();

private:
    FROSFibonacci_FeedbackMessage FeedbackMsg;
    FROSFibonacci_SendGoal_Request GoalRequest;
};
