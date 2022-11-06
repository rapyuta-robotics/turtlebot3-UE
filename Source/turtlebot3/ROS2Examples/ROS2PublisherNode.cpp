// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "ROS2PublisherNode.h"

// RapyutaSimulationPlugins
#include "Tools/RRROS2StringPublisher.h"

void AROS2PublisherNode::BeginPlay()
{
    Super::BeginPlay();
    Init();

    // Create String publisher
    StringPublisher = NewObject<URRROS2StringPublisher>(this);
    StringPublisher->Message = Message;
    StringPublisher->TopicName = TopicName;
    StringPublisher->InitializeWithROS2(this->ActorComponent);
}
