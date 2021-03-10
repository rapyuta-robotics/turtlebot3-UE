// Copyright (C) Rapyuta Robotics


#include "TurtlebotAIController.h"

#include "TurtlebotVehicle.h"

#include "ROS2Node.h"
#include "Msgs/ROS2TwistMsg.h"
#include "Msgs/ROS2LaserScanMsg.h"
#include "Sensors/SensorLidar.h"
#include "ROS2LidarPublisher.h"

ATurtlebotAIController::ATurtlebotAIController(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
	LidarClass = ASensorLidar::StaticClass();
}


void ATurtlebotAIController::OnPossess(APawn *InPawn)
{
	Super::OnPossess(InPawn);

	FActorSpawnParameters LidarSpawnParamsNode;
	FName LidarName("TurtleLidar");
	LidarSpawnParamsNode.Name = LidarName;
	TurtleLidar = GetWorld()->SpawnActor<ASensorLidar>(LidarClass, LidarSpawnParamsNode);
	TurtleLidar->SetActorLocation(InPawn->GetActorLocation() + FVector(6,0,17));
	TurtleLidar->AttachToActor(InPawn, FAttachmentTransformRules::KeepWorldTransform);
	
	FActorSpawnParameters SpawnParamsNode;
	FName NodeName("TurtleNode");
	SpawnParamsNode.Name = NodeName;
	TurtleNode = GetWorld()->SpawnActor<AROS2Node>(AROS2Node::StaticClass(), SpawnParamsNode);
	TurtleNode->SetActorLocation(InPawn->GetActorLocation());
	TurtleNode->AttachToActor(InPawn, FAttachmentTransformRules::KeepWorldTransform);
	TurtleNode->Init();
	
	TurtleLidar->InitToNode(TurtleNode);
	TurtleLidar->Run();

	SetupCommandTopicSubscription(Turtlebot);
}


void ATurtlebotAIController::OnUnPossess()
{
	TurtleLidar = nullptr;
	TurtleNode = nullptr;

	Super::OnUnPossess();
}


void ATurtlebotAIController::SetPawn(APawn *InPawn)
{
	Super::SetPawn(InPawn);

	Turtlebot = Cast<ATurtlebotVehicle>(InPawn);
}


void ATurtlebotAIController::SetupCommandTopicSubscription(ATurtlebotVehicle *InPawn)
{
	if (IsValid(InPawn))
	{
		// Subscription with callback to enqueue vehicle spawn info.
		if (ensure(IsValid(TurtleNode)))
		{
			FSubscriptionCallback cb;
			cb.BindDynamic(this, &ATurtlebotAIController::MovementCallback);
			TurtleNode->AddSubscription(TEXT("cmd_vel"), UROS2TwistMsg::StaticClass(), cb);

			TurtleNode->Subscribe();
		}
	}
}


void ATurtlebotAIController::MovementCallback(const UROS2GenericMsg *Msg)
{
	const UROS2TwistMsg *Concrete = Cast<UROS2TwistMsg>(Msg);

	if (IsValid(Concrete))
	{
		FVector linear(Concrete->GetLinearVelocity());
		FVector angular(Concrete->GetAngularVelocity());
		ATurtlebotVehicle *Vehicle = Turtlebot;

		AsyncTask(ENamedThreads::GameThread, [linear, angular, Vehicle]
		{
			if (IsValid(Vehicle))
			{
				Vehicle->SetLinearVel(linear);
				Vehicle->SetAngularVel(angular);
			}
		});
	}
}
