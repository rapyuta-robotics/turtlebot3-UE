// Copyright (C) Rapyuta Robotics

#pragma once

#include "CoreMinimal.h"
#include "TurtlebotAIController.h"

#include "BurgerAIController.generated.h"

class ATurtlebot3_Burger;

/**
 * This needs to be refactored with a base class from which this and ATurtlebotAIController derive
 */
UCLASS()
class TURTLEBOT3_API ABurgerAIController : public ATurtlebotAIController
{
	GENERATED_BODY()

public:

	ABurgerAIController(const FObjectInitializer& ObjectInitializer);
	
	virtual TArray<FTFData> GetTFData() const override;

	virtual FOdometryData GetOdomData() const override;

protected:

	virtual void MovementCallback(const UROS2GenericMsg *Msg) override;

	virtual void OnPossess(APawn *InPawn) override;

	virtual void SetPawn(APawn *InPawn) override;

	virtual void SetupSubscription(ATurtlebot3_Burger *InPawn);

protected:

	UPROPERTY()
	ATurtlebot3_Burger *Burger;

	UPROPERTY()
	FVector LidarOffset;
};
