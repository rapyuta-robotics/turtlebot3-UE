// Copyright (C) Rapyuta Robotics

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "DifferentialDrive_2W.generated.h"

UCLASS()
class TURTLEBOT3_API ADifferentialDrive_2W : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ADifferentialDrive_2W();

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	UPROPERTY(EditAnywhere,BlueprintReadWrite)
	UStaticMeshComponent* Body;
	
	UPROPERTY(EditAnywhere,BlueprintReadWrite)
	UStaticMeshComponent* WheelL;
	
	UPROPERTY(EditAnywhere,BlueprintReadWrite)
	UStaticMeshComponent* WheelR;
	
	UPROPERTY(EditAnywhere,BlueprintReadWrite)
	UStaticMeshComponent* BallCaster;


	UPROPERTY(EditAnywhere,BlueprintReadWrite)
	UPhysicsConstraintComponent* Body_WheelL;

	UPROPERTY(EditAnywhere,BlueprintReadWrite)
	UPhysicsConstraintComponent* Body_WheelR;

	UPROPERTY(EditAnywhere,BlueprintReadWrite)
	UPhysicsConstraintComponent* Body_BallCaster;
	
	UPROPERTY(EditAnywhere,BlueprintReadWrite)
	float VelocityL = 0;
	
	UPROPERTY(EditAnywhere,BlueprintReadWrite)
	float VelocityR = 0;
	
	UPROPERTY(EditAnywhere,BlueprintReadWrite)
	float MaxForce = 10000;
	
	// UPROPERTY(EditAnywhere,BlueprintReadWrite)
	// float StaticFriction = .7;
	
	// UPROPERTY(EditAnywhere,BlueprintReadWrite)
	// float DynamicFriction = .7;
	
	// UPROPERTY(EditAnywhere,BlueprintReadWrite)
	// float Restitution = .1;
	
	UPROPERTY(EditAnywhere,BlueprintReadWrite)
	UMaterial* VehicleMaterial;

	UFUNCTION(BlueprintCallable)
	void SetAngularVelocityTargets(float velL, float velR);	
};
