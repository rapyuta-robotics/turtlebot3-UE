/**
 * @file ROS2ServiceServerNode.h
 * @brief ROS2 Service Server example class
 * @copyright Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
 */

#pragma once
// UE
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

// rclUE
#include <ROS2Node.h>
#include <Srvs/ROS2AddTwoInts.h>

// Turtlebot3_UE
#include "turtlebot3/Turtlebot3.h"

#include "ROS2ServiceServerNode.generated.h"

/**
 * @brief ROS2 Service server example class. This actor has AddTwoInts service server.
 *
 */
UCLASS()
class TURTLEBOT3_API AROS2ServiceServerNode : public AActor
{
    GENERATED_BODY()

public:
    AROS2ServiceServerNode();

    virtual void BeginPlay() override;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString ServiceName = TEXT("add_two_ints");

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UROS2NodeComponent* Node = nullptr;

    /**
     * @brief Service callback function
     *
     * @param Service
     */
    UFUNCTION()
    void SrvCallback(UROS2GenericSrv* InService);
};
