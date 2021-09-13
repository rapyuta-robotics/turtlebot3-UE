// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "ROS2SubscriberNode.h"

// Sets default values
AROS2SubscriberNode::AROS2SubscriberNode()
{
    // Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
    PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void AROS2SubscriberNode::BeginPlay()
{
    Super::BeginPlay();
    Init();

    // bind callback function with topic subscription
    FSubscriptionCallback cb;
    cb.BindDynamic(this, &AROS2SubscriberNode::MsgCallback);
    AddSubscription(TEXT("test_topic"), UROS2StringMsg::StaticClass(), cb);
}

// Callback function
void AROS2SubscriberNode::MsgCallback(const UROS2GenericMsg* Msg)
{
    const UROS2StringMsg* StringMsg = Cast<UROS2StringMsg>(Msg);
    UE_LOG(LogTemp, Log, TEXT("%s"), *FString(StringMsg->MsgToString()));
}
