// Copyright (C) Rapyuta Robotics

#pragma once

#include "CoreMinimal.h"
#include "AIController.h"

#include <Msgs/ROS2OdometryMsg.h>
#include <Msgs/ROS2TFMsg.h>

#include "TurtlebotAIController.generated.h"

class AROS2Node;
class ASensorLidar;
class UROS2Publisher;
class ATurtlebotVehicle;

/**
 * 
 */
UCLASS(ClassGroup=(Custom),  meta=(BlueprintSpawnableComponent))
class TURTLEBOT3_API ATurtlebotAIController : public AAIController
{
	GENERATED_BODY()

protected:

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	TSubclassOf<ASensorLidar> LidarClass;

	UPROPERTY(Transient)
	AROS2Node *TurtleNode;

	UPROPERTY(Transient, BlueprintReadWrite)
	ASensorLidar *TurtleLidar;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	UROS2Publisher *TFPublisher;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	UROS2Publisher *OdomPublisher;

public:

	ATurtlebotAIController(const FObjectInitializer& ObjectInitializer);

	UFUNCTION(BlueprintCallable)
	TArray<FTFData> GetTFData() const;

	UFUNCTION(BlueprintCallable)
	FOdometryData GetOdomData() const;

protected:

	UFUNCTION()
	void MovementCallback(const UROS2GenericMsg *Msg);

	virtual void OnPossess(APawn *InPawn) override;

	virtual void OnUnPossess() override;

	virtual void SetPawn(APawn *InPawn) override;

	virtual void SetupCommandTopicSubscription(ATurtlebotVehicle *InPawn);

	UFUNCTION()
	void TFMessageUpdate(UROS2GenericMsg *TopicMessage);

	UFUNCTION()
	void OdomMessageUpdate(UROS2GenericMsg *TopicMessage);

protected:

	UPROPERTY()
	ATurtlebotVehicle *Turtlebot;

	UPROPERTY()
	FVector InitialPosition;

	UPROPERTY()
	FRotator InitialOrientation;
};
