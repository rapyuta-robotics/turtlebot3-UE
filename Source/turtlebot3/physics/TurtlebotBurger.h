// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

#include "CoreMinimal.h"
#include "Robot/RobotVehicle.h"

#include "Drive/DifferentialDriveComponent.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"

#include "TurtlebotBurger.generated.h"

/**
 * 
 */
UCLASS()
class TURTLEBOT3_API ATurtlebotBurger : public ARobotVehicle
{
	GENERATED_BODY()
	
public:	
	ATurtlebotBurger(const FObjectInitializer& ObjectInitializer);
	ATurtlebotBurger(const FObjectInitializer& ObjectInitializer, const bool& InitMovementComponent);

protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	UFUNCTION(BlueprintCallable)
	virtual void Init();

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
	float MaxForce = 1000;
	
	UPROPERTY(EditAnywhere,BlueprintReadWrite)
	UMaterial* VehicleMaterial;
	
	UPROPERTY(EditAnywhere,BlueprintReadWrite)
	UMaterial* BallMaterial;

	UPROPERTY(VisibleAnywhere)
	bool IsInitialized = false;

protected:

	UFUNCTION()
	void SetupConstraintsAndPhysics();
	
	
};
