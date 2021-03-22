// Fill out your copyright notice in the Description page of Project Settings.


#include "ROS2OdomPublisher.h"
#include "rclcUtilities.h"
#include "Msgs/ROS2OdometryMsg.h"
#include "TurtlebotAIController.h"

// Sets default values for this component's properties
UROS2OdomPublisher::UROS2OdomPublisher() : UROS2Publisher()
{
}

void UROS2OdomPublisher::UpdateAndPublishMessage_Implementation()
{
	check(State == UROS2State::Initialized);
	check(ownerNode != nullptr);
	
	FOdometryData OdomData = Controller->GetOdomData();
    UROS2OdometryMsg* Message = Cast<UROS2OdometryMsg>(TopicMessage);
    Message->Update(OdomData);
    Publish();
}
