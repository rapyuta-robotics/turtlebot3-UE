// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once
// UE
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

// rclUE
#include "ROS2Subscriber.h"

#include <Msgs/ROS2Str.h>

#include "ROS2SubscriberNode.generated.h"

UCLASS()
class TURTLEBOT3_API AROS2SubscriberNode : public AActor
{
    GENERATED_BODY()

public:
    AROS2SubscriberNode();

    virtual void BeginPlay() override;

    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    UFUNCTION()
    void MsgCallback(const UROS2GenericMsg* InMsg);

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UROS2Node* Node = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString TopicName = TEXT("test_topic");
};
