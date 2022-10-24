// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "ROS2SubscriberNode.h"

// RapyutaSimulationPlugins
#include "Core/RRCoreUtils.h"

// Turtlebot3_UE
#include "turtlebot3/Turtlebot3.h"

void AROS2SubscriberNode::BeginPlay()
{
    if (false == URRCoreUtils::IsROS2SystemEnabled(this))
    {
        UE_LOG(LogTurtlebot3, Error, TEXT("[%s]ROS2 is not enabled in ARRROS2GameMode"), *GetName());
        PrimaryActorTick.bCanEverTick = false;
        return;
    }

    Super::BeginPlay();
    Init();

    // bind callback function with topic subscription
    FSubscriptionCallback cb;
    cb.BindDynamic(this, &AROS2SubscriberNode::MsgCallback);
    AddSubscription(TopicName, UROS2StrMsg::StaticClass(), cb);
}

void AROS2SubscriberNode::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    if (false == URRCoreUtils::IsROS2SystemEnabled(this))
    {
        return;
    }
    Super::EndPlay(EndPlayReason);
}

void AROS2SubscriberNode::Tick(float DeltaTime)
{
    if (false == URRCoreUtils::IsROS2SystemEnabled(this))
    {
        return;
    }
    Super::Tick(DeltaTime);
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
