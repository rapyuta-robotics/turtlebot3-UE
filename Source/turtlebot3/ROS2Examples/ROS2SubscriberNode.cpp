// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "ROS2SubscriberNode.h"

// Turtlebot3_UE
#include "turtlebot3/Turtlebot3.h"

AROS2SubscriberNode::AROS2SubscriberNode()
{
    PrimaryActorTick.bCanEverTick = true;
}

void AROS2SubscriberNode::BeginPlay()
{
    Super::BeginPlay();
    Init();

    // bind callback function with topic subscription
    FSubscriptionCallback cb;
    cb.BindDynamic(this, &AROS2SubscriberNode::MsgCallback);
    AddSubscription(TopicName, UROS2StrMsg::StaticClass(), cb);
}

void AROS2SubscriberNode::MsgCallback(const UROS2GenericMsg* InMsg)
{
    const UROS2StrMsg* stringMsg = Cast<UROS2StrMsg>(InMsg);
    if (stringMsg)
    {
        FROSStr msg;
        stringMsg->GetMsg(msg);
        UE_LOG(LogTurtlebot3, Log, TEXT("[%s] %s"), *GetName(), *msg.Data);
    }
}
