/**
 * @file ROS2ServiceClientNode.h
 * @brief ROS2 Service Client example class
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once
#include "CoreMinimal.h"

// rclUE
#include <ROS2Node.h>
#include <ROS2ServiceClient.h>
#include <Srvs/ROS2AddTwoInts.h>

// Turtlebot3_UE
#include "turtlebot3/Turtlebot3.h"

#include "ROS2ServiceClientNode.generated.h"

/**
 * @brief ROS2 Service client example class. This actor has AddTwoInts service client.
 *
 */
UCLASS()
class TURTLEBOT3_API AROS2ServiceClientNode : public AActor
{
    GENERATED_BODY()

protected:
    AROS2ServiceClientNode();

    virtual void BeginPlay() override;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UROS2NodeComponent* Node = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UROS2ServiceClient* AddTwoIntsSrvClient = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString ServiceName = TEXT("add_two_ints");

    //! AddTwoInts input
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    int A = 1;

    //! AddTwoInts input
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    int B = 2;

    /**
     * @brief
     * This method is bounded with AddTwoIntsSrvClient::RequestDelegate in #BeginPlay
     * and called when receive response.
     * @param InService
     */
    UFUNCTION()
    void ReceiveResponse(UROS2GenericSrv* InService);

    UPROPERTY()
    FTimerHandle TimerHandle;
};
