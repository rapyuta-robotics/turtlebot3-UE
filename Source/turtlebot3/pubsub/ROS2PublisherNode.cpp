// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "ROS2PublisherNode.h"

// Sets default values
AROS2PublisherNode::AROS2PublisherNode()
{
    // Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
    PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void AROS2PublisherNode::BeginPlay()
{
    Super::BeginPlay();
    Init();

    // Create String publisher
    StringPublisher = NewObject<UROS2Publisher>(this, UROS2Publisher::StaticClass());
    StringPublisher->RegisterComponent();
    StringPublisher->TopicName = TEXT("test_topic");
    StringPublisher->PublicationFrequencyHz = 1;
    StringPublisher->MsgClass = UROS2StringMsg::StaticClass();
    StringPublisher->UpdateDelegate.BindDynamic(this, &AROS2PublisherNode::MessageUpdate);
    AddPublisher(StringPublisher);
    StringPublisher->Init(UROS2QoS::DynamicBroadcaster);

    // Msg data. You can change from editor.
    // Fill default value if value is empty
    Message = Message.IsEmpty() ? TEXT("Hello from C++") : Message;
}

// MessageUpdate function is called with PublicationFrequencyHz
void AROS2PublisherNode::MessageUpdate(UROS2GenericMsg* TopicMessage)
{
    UROS2StringMsg* StringMessage = Cast<UROS2StringMsg>(TopicMessage);
    StringMessage->Update(Message);
}
