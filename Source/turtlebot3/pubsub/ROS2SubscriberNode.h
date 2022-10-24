// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once
// UE
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

// rclUE
#include <Msgs/ROS2Str.h>
#include <ROS2Node.h>

#include "ROS2SubscriberNode.generated.h"

UCLASS()
class TURTLEBOT3_API AROS2SubscriberNode : public AROS2Node
{
    GENERATED_BODY()

public:
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString TopicName = TEXT("test_topic");

protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
    virtual void Tick(float DeltaTime);

    UFUNCTION()
    void MsgCallback(const UROS2GenericMsg* InMsg);
};
