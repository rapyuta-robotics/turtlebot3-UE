// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once
// UE
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

// rclUE
#include <ROS2Node.h>
#include <Srvs/ROS2AddTwoIntsSrv.h>

// Turtlebot3_UE
#include "turtlebot3/Turtlebot3.h"

#include "ROS2ServiceServerNode.generated.h"

UCLASS()
class TURTLEBOT3_API AROS2ServiceServerNode : public AROS2Node
{
    GENERATED_BODY()

public:
    AROS2ServiceServerNode();

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString ServiceName = TEXT("add_two_ints");

protected:
    virtual void BeginPlay() override;

    UFUNCTION()
    void SrvCallback(UROS2GenericSrv* Service);
};
