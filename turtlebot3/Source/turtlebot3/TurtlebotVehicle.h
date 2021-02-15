// Copyright (C) Rapyuta Robotics

#pragma once

#include "CoreMinimal.h"
#include "WheeledVehicle.h"

#include "TurtlebotVehicle.generated.h"

/**
 * 
 */
UCLASS()
class TURTLEBOT3_API ATurtlebotVehicle : public AWheeledVehicle
{
	GENERATED_BODY()

public:

	ATurtlebotVehicle(const FObjectInitializer& ObjectInitializer);

	virtual void Tick(float DeltaSeconds) override;

	virtual void SetLinearVel(FVector velocity);

	virtual void SetAngularVel(FVector velocity);

protected:

	virtual void BeginPlay() override;

	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
};
