// Copyright (C) Rapyuta Robotics

#pragma once

#include "CoreMinimal.h"
#include "AIController.h"

#include "TurtlebotAIController.generated.h"

class AROS2Node;
class ATurtlebotVehicle;

/**
 * 
 */
UCLASS()
class TURTLEBOT3_API ATurtlebotAIController : public AAIController
{
	GENERATED_BODY()

public:

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Parameters")
	float Radius = 1.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Parameters")
	float Timeout = 0.1f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Parameters")
	float UpdateRage = 50.0f;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Parameters")
	bool bStickToGround = false;

protected:

	UPROPERTY(Transient)
	AROS2Node *TurtleNode;

public:

	ATurtlebotAIController(const FObjectInitializer& ObjectInitializer);

protected:

	virtual void OnPossess(APawn *InPawn) override;

	virtual void OnUnPossess() override;

	virtual void SetPawn(APawn *InPawn) override;

	virtual void SetupCommandTopicSubscription(ATurtlebotVehicle *InPawn);

protected:

	UPROPERTY()
	ATurtlebotVehicle *Turtlebot;
};
