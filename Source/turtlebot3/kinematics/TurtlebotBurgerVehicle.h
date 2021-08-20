// Copyright (C) Rapyuta Robotics

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"

#include "TurtlebotBurgerVehicle.generated.h"

class PawnMovementComponent;

/**
 * 
 */
UCLASS()
class TURTLEBOT3_API ATurtlebotBurgerVehicle : public APawn
{
	GENERATED_BODY()

public:

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	UPawnMovementComponent *MoveComponent;

public:

	ATurtlebotBurgerVehicle(const FObjectInitializer& ObjectInitializer);

	virtual void Tick(float DeltaSeconds) override;

	virtual void SetLinearVel(FVector velocity);

	virtual void SetAngularVel(FVector velocity);

protected:

	virtual void BeginPlay() override;

	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
};
