// Fill out your copyright notice in the Description page of Project Settings.


#include "ROS2TFPublisher.h"
#include "rclcUtilities.h"
#include "Msgs/ROS2TFMsg.h"
#include "TurtlebotAIController.h"

// Sets default values for this component's properties
UROS2TFPublisher::UROS2TFPublisher() : UROS2Publisher()
{
}

void UROS2TFPublisher::UpdateAndPublishMessage_Implementation()
{
	check(State == UROS2State::Initialized);
	check(ownerNode != nullptr);
	
	TArray<FTFData> TFData = Controller->GetTFData();
    UROS2TFMsg* Message = Cast<UROS2TFMsg>(TopicMessage);
    Message->Update(TFData);
    Publish();
}