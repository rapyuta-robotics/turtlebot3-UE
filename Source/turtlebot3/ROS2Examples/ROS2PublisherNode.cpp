// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "ROS2PublisherNode.h"

AROS2PublisherNode::AROS2PublisherNode()
{
    Node = CreateDefaultSubobject<UROS2NodeComponent>(TEXT("ROS2NodeComponent"));
    Node->RegisterComponent();

    // these parameters can be change from BP
    Node->Name = TEXT("publisher_node");
    Node->Namespace = TEXT("cpp");
}

void AROS2PublisherNode::BeginPlay()
{
    Super::BeginPlay();

    Node->Init();

    // Create publisher with 3 different way.
    // 1. Non Loop Publisher
    // 2. Loop Publisher
    // 3. Custom Publisher class

    // 1. Non Loop Publisher
    // 1.1 Create publisher
    Publisher = Node->CreatePublisher(TopicName, UROS2Publisher::StaticClass(), UROS2StrMsg::StaticClass(), UROS2QoS::KeepLast);

    // 1.2 Create msg
    FROSStr msg;
    msg.Data = FString::Printf(TEXT("%s from non loop publisher"), *Message);
    CastChecked<UROS2StrMsg>(Publisher->TopicMessage)->SetMsg(msg);

    // 1.3 publish
    Publisher->Publish();

    // 2. Loop Publisher
    ROS2_CREATE_LOOP_PUBLISHER_WITH_QOS(Node,
                                        this,
                                        TopicName,
                                        UROS2Publisher::StaticClass(),
                                        UROS2StrMsg::StaticClass(),
                                        PublicationFrequencyHz,
                                        &AROS2PublisherNode::UpdateMessage,
                                        UROS2QoS::Default,
                                        LoopPublisher);

    // 3. Use Custom Publisher class
    // UpdateMessage is overriden in child class.
    StringPublisher = CastChecked<URRROS2StringPublisher>(
        Node->CreateLoopPublisherWithClass(TopicName, URRROS2StringPublisher::StaticClass(), 1.f));
    StringPublisher->Message = FString::Printf(TEXT("%s from custom class"), *Message);
}

void AROS2PublisherNode::UpdateMessage(UROS2GenericMsg* InMessage)
{
    FROSStr msg;
    msg.Data = FString::Printf(TEXT("%s %d"), *Message, Count++);
    CastChecked<UROS2StrMsg>(InMessage)->SetMsg(msg);
}
