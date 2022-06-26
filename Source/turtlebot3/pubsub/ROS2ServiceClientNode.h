// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once
#include "CoreMinimal.h"

// rclUE
#include <ROS2Node.h>
#include <ROS2ServiceClient.h>
#include <Srvs/ROS2AddTwoIntsSrv.h>

// Turtlebot3_UE
#include "turtlebot3/Turtlebot3.h"

#include "ROS2ServiceClientNode.generated.h"

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

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    int A = 1;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    int B = 2;

    UFUNCTION()
    void SendRequest(UROS2GenericSrv* InService);

    UFUNCTION()
    void ReceiveResponse(UROS2GenericSrv* InService);
};
