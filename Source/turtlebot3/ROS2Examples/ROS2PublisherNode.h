// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once
#include "CoreMinimal.h"

// rclUE
#include "ROS2Node.h"

#include "ROS2PublisherNode.generated.h"

class URRROS2StringPublisher;
UCLASS()
class TURTLEBOT3_API AROS2PublisherNode : public AROS2Node
{
    GENERATED_BODY()

protected:
    // Called when the game starts or when spawned
    virtual void BeginPlay() override;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    URRROS2StringPublisher* StringPublisher = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString TopicName = TEXT("test_topic");

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString Message = TEXT("Hello from C++");
};
