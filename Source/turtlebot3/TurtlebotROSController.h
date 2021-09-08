// Copyright (C) Rapyuta Robotics

#pragma once

#include "CoreMinimal.h"
#include "AIController.h"

#include <Msgs/ROS2OdometryMsg.h>
#include <Msgs/ROS2TFMsg.h>
#include <Tools/ROS2TFPublisher.h>

#include "TurtlebotROSController.generated.h"

class AROS2Node;
class ASensorLidar;
class UROS2Publisher;
class ARobotVehicle;

/**
 * 
 */
UCLASS(ClassGroup=(Custom),  meta=(BlueprintSpawnableComponent))
class TURTLEBOT3_API ATurtlebotROSController : public AAIController
{
	GENERATED_BODY()

protected:

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	TSubclassOf<ASensorLidar> LidarClass;

	UPROPERTY(Transient)
	AROS2Node *TurtleNode;

	// UPROPERTY(Transient, BlueprintReadWrite)
	// ASensorLidar *TurtleLidar;

	UPROPERTY(Transient, BlueprintReadWrite)
	FVector LidarOffset;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	UROS2TFPublisher *TFPublisher;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	UROS2Publisher *OdomPublisher;

public:

	ATurtlebotROSController(const FObjectInitializer& ObjectInitializer);

	UFUNCTION(BlueprintCallable)
	virtual FROSOdometry GetOdomData() const;

	UPROPERTY(EditAnywhere,BlueprintReadWrite)
	int RobotID = 0;

	// total number of agents (== maxID)
	UPROPERTY(EditAnywhere,BlueprintReadWrite)
	int NAgents = 1;

protected:

	UFUNCTION()
	virtual void MovementCallback(const UROS2GenericMsg *Msg);

	virtual void OnPossess(APawn *InPawn) override;

	virtual void OnUnPossess() override;

	virtual void SetPawn(APawn *InPawn) override;

	virtual void SetupCommandTopicSubscription(ARobotVehicle *InPawn);

	UFUNCTION()
	void OdomMessageUpdate(UROS2GenericMsg *TopicMessage);

protected:

	UPROPERTY()
	ARobotVehicle *Turtlebot;

	UPROPERTY()
	FVector InitialPosition;

	UPROPERTY()
	FRotator InitialOrientation;
};
