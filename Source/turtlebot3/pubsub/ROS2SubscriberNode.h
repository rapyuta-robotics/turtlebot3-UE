// Copyright (C) Rapyuta Robotics

#pragma once
// UE
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"

// rclUE
#include <Msgs/ROS2StringMsg.h>
#include <ROS2Node.h>

#include "ROS2SubscriberNode.generated.h"

UCLASS()
class TURTLEBOT3_API AROS2SubscriberNode : public AROS2Node
{
    GENERATED_BODY()

public:
    AROS2SubscriberNode();

protected:
    virtual void BeginPlay() override;

    UFUNCTION()
    void MsgCallback(const UROS2GenericMsg* Msg);
};
