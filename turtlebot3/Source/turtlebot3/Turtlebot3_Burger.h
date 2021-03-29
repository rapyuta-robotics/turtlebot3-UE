// Copyright (C) Rapyuta Robotics

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "Turtlebot3_Burger.generated.h"

class PawnMovementComponent;

UCLASS()
class TURTLEBOT3_API ATurtlebot3_Burger : public APawn
{
	GENERATED_BODY()
	
public:	
	ATurtlebot3_Burger(const FObjectInitializer& ObjectInitializer);

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	void Init();

	UPROPERTY(EditAnywhere,BlueprintReadWrite)
	UStaticMeshComponent* Base;

	UPROPERTY(EditAnywhere,BlueprintReadWrite)
	UStaticMeshComponent* LidarSensor;
	
	UPROPERTY(EditAnywhere,BlueprintReadWrite)
	UStaticMeshComponent* WheelLeft;
	
	UPROPERTY(EditAnywhere,BlueprintReadWrite)
	UStaticMeshComponent* WheelRight;
	
	UPROPERTY(EditAnywhere,BlueprintReadWrite)
	UStaticMeshComponent* CasterBack;


	UPROPERTY(EditAnywhere,BlueprintReadWrite)
	UPhysicsConstraintComponent* Base_LidarSensor;

	UPROPERTY(EditAnywhere,BlueprintReadWrite)
	UPhysicsConstraintComponent* Base_WheelLeft;

	UPROPERTY(EditAnywhere,BlueprintReadWrite)
	UPhysicsConstraintComponent* Base_WheelRight;

	UPROPERTY(EditAnywhere,BlueprintReadWrite)
	UPhysicsConstraintComponent* Base_CasterBack;
	
	UPROPERTY(EditAnywhere,BlueprintReadWrite)
	float VelocityL = 0;
	
	UPROPERTY(EditAnywhere,BlueprintReadWrite)
	float VelocityR = 0;
	
	UPROPERTY(EditAnywhere,BlueprintReadWrite)
	float MaxForce = 10000;
	
	UPROPERTY(EditAnywhere,BlueprintReadWrite)
	UMaterial* VehicleMaterial;

	UPROPERTY(VisibleAnywhere)
	bool IsInitialized = false;

	UFUNCTION(BlueprintCallable)
	void SetAngularVelocityTargets(float velL, float velR);	

protected:

	UFUNCTION()
	void SetupConstraints();
};
