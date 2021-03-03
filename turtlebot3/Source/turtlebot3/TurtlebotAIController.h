// Copyright (C) Rapyuta Robotics

#pragma once

#include "CoreMinimal.h"
#include "AIController.h"

#include "TurtlebotAIController.generated.h"

class AROS2Node;
class ASensorLidar;
class ATurtlebotVehicle;

/**
 * 
 */
UCLASS(ClassGroup=(Custom),  meta=(BlueprintSpawnableComponent))
class TURTLEBOT3_API ATurtlebotAIController : public AAIController
{
	GENERATED_BODY()

protected:

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	TSubclassOf<ASensorLidar> LidarClass;

	UPROPERTY(Transient)
	AROS2Node *TurtleNode;

	UPROPERTY(Transient)
	ASensorLidar *TurtleLidar;

public:

	ATurtlebotAIController(const FObjectInitializer& ObjectInitializer);

protected:

	UFUNCTION()
	void MovementCallback(const UROS2GenericMsg *Msg);

	virtual void OnPossess(APawn *InPawn) override;

	virtual void OnUnPossess() override;

	virtual void SetPawn(APawn *InPawn) override;

	virtual void SetupCommandTopicSubscription(ATurtlebotVehicle *InPawn);

protected:

	UPROPERTY()
	ATurtlebotVehicle *Turtlebot;
};
