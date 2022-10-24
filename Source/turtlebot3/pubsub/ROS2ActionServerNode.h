/**
 * @file ROS2ActionServerNode.h
 * @brief ROS2 Service Server example class
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once
// UE
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

// rclUE
#include <Actions/ROS2Fibonacci.h>
#include <ROS2ActionServer.h>
#include <ROS2Node.h>

// Turtlebot3_UE
#include "turtlebot3/Turtlebot3.h"

#include "ROS2ActionServerNode.generated.h"

/**
 * @brief ROS2 Action server example class. This actor has Fibonacci action server.
 *
 */
UCLASS()
class TURTLEBOT3_API AROS2ActionServerNode : public AROS2Node
{
    GENERATED_BODY()

public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UROS2ActionServer* FibonacciActionServer = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString ActionName = TEXT("fibonacci_action");

protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void Tick(float DeltaTime);

    UPROPERTY()
    FTimerHandle ActionTimerHandle;

    UFUNCTION()
    void UpdateFeedbackCallback(UROS2GenericAction* InAction);
    UFUNCTION()
    void UpdateResultCallback(UROS2GenericAction* InAction);
    UFUNCTION()
    bool HandleGoalCallback(UROS2GenericAction* InAction);
    UFUNCTION()
    void HandleCancelCallback();
    UFUNCTION()
    void HandleAcceptedCallback();

private:
    UPROPERTY()
    FROSFibonacciFB FeedbackMsg;
    UPROPERTY()
    FROSFibonacciSGReq GoalRequest;
    UPROPERTY()
    uint64 Count = 0;
};
