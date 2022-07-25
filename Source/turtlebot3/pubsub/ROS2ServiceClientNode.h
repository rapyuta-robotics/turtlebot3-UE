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
#include <Srvs/ROS2AddTwoIntsSrv.h>

// Turtlebot3_UE
#include "turtlebot3/Turtlebot3.h"

#include "ROS2ServiceClientNode.generated.h"

/**
 * @brief ROS2 Service client example class. This actor has AddTwoInts service client.
 *
 */
UCLASS()
class TURTLEBOT3_API AROS2ServiceClientNode : public AROS2Node
{
    GENERATED_BODY()

protected:
    // Called when the game starts or when spawned
    virtual void BeginPlay() override;

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
     * @brief Set and update service request.
     * This method is bounded with AddTwoIntsSrvClient::RequestDelegate in #BeginPlay
     * and called with UROS2ServiceClient::UpdateAndSendRequest
     *
     * @param InService
     */
    UFUNCTION()
    void SendRequest(UROS2GenericSrv* InService);

    /**
     * @brief
     * This method is bounded with AddTwoIntsSrvClient::RequestDelegate in #BeginPlay
     * and called when receive response.
     * @param InService
     */
    UFUNCTION()
    void ReceiveResponse(UROS2GenericSrv* InService);
};
