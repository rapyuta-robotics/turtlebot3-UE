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
class TURTLEBOT3_API AROS2ServiceServerNode : public AROS2Node
{
    GENERATED_BODY()

public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString ServiceName = TEXT("add_two_ints");

protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void Tick(float DeltaTime);

    /**
     * @brief Service callback function
     *
     * @param Service
     */
    UFUNCTION()
    void SrvCallback(UROS2GenericSrv* InService);
};
