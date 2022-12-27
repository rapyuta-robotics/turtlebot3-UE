// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once
#include "CoreMinimal.h"

// rclUE
#include "Msgs/ROS2Str.h"
#include "ROS2Publisher.h"

#include "ROS2PublisherNode.generated.h"

UCLASS()
class TURTLEBOT3_API AROS2PublisherNode : public AActor
{
    GENERATED_BODY()

public:
    AROS2PublisherNode();

    virtual void BeginPlay() override;

    UFUNCTION()
    void UpdateMessage(UROS2GenericMsg* InMessage);

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UROS2NodeComponent* Node = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    UROS2Publisher* Publisher = nullptr;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString TopicName = TEXT("test_topic");

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    float PublicationFrequencyHz = 1.f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FString Message = TEXT("Hello from C++");

    UPROPERTY()
    int32 Count = 0;
};
