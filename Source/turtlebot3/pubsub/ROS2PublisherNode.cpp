// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "ROS2PublisherNode.h"

// RapyutaSimulationPlugins
#include "Core/RRCoreUtils.h"
#include "Tools/RRROS2StringPublisher.h"

// Turtlebot3_UE
#include "turtlebot3/Turtlebot3.h"

void AROS2PublisherNode::BeginPlay()
{
    if (false == URRCoreUtils::IsROS2SystemEnabled(this))
    {
        UE_LOG(LogTurtlebot3, Error, TEXT("[%s]ROS2 is not enabled in ARRROS2GameMode"), *GetName());
        PrimaryActorTick.bCanEverTick = false;
        return;
    }

    Super::BeginPlay();
    Init();

    // Create String publisher
    StringPublisher = NewObject<URRROS2StringPublisher>(this);
    StringPublisher->Message = Message;
    StringPublisher->TopicName = TopicName;
    StringPublisher->InitializeWithROS2(this);
}

void AROS2PublisherNode::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    if (false == URRCoreUtils::IsROS2SystemEnabled(this))
    {
        return;
    }
    Super::EndPlay(EndPlayReason);
}

void AROS2PublisherNode::Tick(float DeltaTime)
{
    if (false == URRCoreUtils::IsROS2SystemEnabled(this))
    {
        return;
    }
    Super::Tick(DeltaTime);
}
