// Copyright (C) Rapyuta Robotics

#pragma once
#include <ROS2Node.h>
#include <Msgs/ROS2StringMsg.h>
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ROS2SubscriberNode.generated.h"

UCLASS()
class TURTLEBOT3_API AROS2SubscriberNode : public AROS2Node
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AROS2SubscriberNode();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	// Callback function
	void MsgCallback(const UROS2GenericMsg *Msg);

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	
	
};
