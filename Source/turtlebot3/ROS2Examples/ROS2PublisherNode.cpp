// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "ROS2PublisherNode.h"

AROS2PublisherNode::AROS2PublisherNode()
{
    Node = CreateDefaultSubobject<UROS2NodeComponent>(TEXT("ROS2NodeComponent"));
    Node->RegisterComponent();
    Node->Name = TEXT("publisher_node");
    Node->Namespace = TEXT("cpp");
}

void AROS2PublisherNode::BeginPlay()
{
    Super::BeginPlay();

    Node->Init();
    ROS2_CREATE_PUBLISHER(Node,
                          this,
                          TopicName,
                          UROS2Publisher::StaticClass(),
                          UROS2StrMsg::StaticClass(),
                          PublicationFrequencyHz,
                          &AROS2PublisherNode::UpdateMessage);
}

void AROS2PublisherNode::UpdateMessage(UROS2GenericMsg* InMessage)
{
    FROSStr msg;
    msg.Data = FString::Printf(TEXT("%s %d"), *Message, Count++);
    CastChecked<UROS2StrMsg>(InMessage)->SetMsg(msg);
}
