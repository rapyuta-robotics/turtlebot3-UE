// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

#include "CoreMinimal.h"
#include "../TurtlebotROSController.h"

#include "BurgerAIController.generated.h"

class ATurtlebot3_Burger;

/**
 * This needs to be refactored with a base class from which this and ATurtlebotROSController derive
 */
UCLASS()
class TURTLEBOT3_API ABurgerAIController : public ATurtlebotROSController
{
	GENERATED_BODY()

public:

	ABurgerAIController(const FObjectInitializer& ObjectInitializer);
	
	virtual TArray<FTFData> GetTFData() const ;

	virtual TArray<FTFData> GetTFStaticData() const ;

	virtual FROSOdometry GetOdomData() const ;

protected:
	UFUNCTION()
	virtual void ClockCallback(const UROS2GenericMsg *Msg);

	virtual void MovementCallback(const UROS2GenericMsg *Msg) override;

	virtual void OnPossess(APawn *InPawn) override;

	virtual void SetPawn(APawn *InPawn) override;

	virtual void SetupSubscription(ATurtlebot3_Burger *InPawn);

protected:

	UPROPERTY()
	ATurtlebot3_Burger *Burger;

	UPROPERTY()
	FVector LinearVelTarget;

	UPROPERTY()
	FVector AngularVelTarget;
};
