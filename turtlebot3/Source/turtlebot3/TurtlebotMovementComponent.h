// Copyright (C) Rapyuta Robotics

#pragma once

#include "CoreMinimal.h"
#include "WheeledVehicleMovementComponent.h"

#include "TurtlebotMovementComponent.generated.h"

/**
 * 
 */
UCLASS()
class TURTLEBOT3_API UTurtlebotMovementComponent : public UWheeledVehicleMovementComponent
{
	GENERATED_BODY()

private:

	UPROPERTY(Transient)
	FVector DesiredMovement;

	UPROPERTY(Transient)
	FQuat DesiredRotation;

public:

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Velocity)
	FVector AngularVelocity;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Velocity)
	float InverseRadius;

private:

	virtual void UpdateMovement(float DeltaTime);

public:

	virtual void TickComponent(float DeltaTime, enum ELevelTick TickType, FActorComponentTickFunction *ThisTickFunction) override;
};
