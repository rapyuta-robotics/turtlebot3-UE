// Copyright (C) Rapyuta Robotics

#pragma once
#include <ROS2Node.h>
#include <ROS2Publisher.h>
#include <Msgs/ROS2StringMsg.h>
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "ROS2PublisherNode.generated.h"

UCLASS()
class TURTLEBOT3_API AROS2PublisherNode : public AROS2Node
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AROS2PublisherNode();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	UROS2Publisher *StringPublisher;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FString Message;

	UFUNCTION()
	void MessageUpdate(UROS2GenericMsg *TopicMessage);

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	
	
};
