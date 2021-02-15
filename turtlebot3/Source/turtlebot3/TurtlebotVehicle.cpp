// Copyright (C) Rapyuta Robotics


#include "TurtlebotVehicle.h"
#include "TurtlebotMovementComponent.h"

ATurtlebotVehicle::ATurtlebotVehicle(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer.SetDefaultSubobjectClass<UTurtlebotMovementComponent>(VehicleMovementComponentName))
{
}


void ATurtlebotVehicle::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
}


void ATurtlebotVehicle::SetLinearVel(FVector Velocity)
{
	// We're assuming input is in meters, so convert to centimeters.
	GetVehicleMovementComponent()->Velocity = Velocity * 100.0f;
}


void ATurtlebotVehicle::SetAngularVel(FVector Velocity)
{
	UTurtlebotMovementComponent *TurtlebotMovementComponent = Cast<UTurtlebotMovementComponent>(GetVehicleMovementComponent());

	if (TurtlebotMovementComponent)
	{
		TurtlebotMovementComponent->AngularVelocity = Velocity;
	}
}


void ATurtlebotVehicle::BeginPlay()
{
	Super::BeginPlay();
}


void ATurtlebotVehicle::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Super::EndPlay(EndPlayReason);
}
