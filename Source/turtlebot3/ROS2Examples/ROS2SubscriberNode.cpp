// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "ROS2SubscriberNode.h"

// Turtlebot3_UE
#include "turtlebot3/Turtlebot3.h"

AROS2SubscriberNode::AROS2SubscriberNode()
{
    Node = CreateDefaultSubobject<UROS2NodeComponent>(TEXT("ROS2NodeComponent"));
    Node->RegisterComponent();
    Node->Name = TEXT("subscriber_node");
    Node->Namespace = TEXT("cpp");
}

void AROS2SubscriberNode::BeginPlay()
{
    Super::BeginPlay();
    Node->Init();

    ROS2_CREATE_SUBSCRIBER(Node, this, TopicName, UROS2StrMsg::StaticClass(), &AROS2SubscriberNode::MsgCallback);
    // // bind callback function with topic subscription
    // FSubscriptionCallback cb;
    // cb.BindDynamic(this, &AROS2SubscriberNode::MsgCallback);
    // AddSubscription(TopicName, UROS2StrMsg::StaticClass(), cb);
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
