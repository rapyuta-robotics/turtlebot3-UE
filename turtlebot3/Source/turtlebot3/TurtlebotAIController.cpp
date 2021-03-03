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
}


void ATurtlebotAIController::OnPossess(APawn *InPawn)
{
	Super::OnPossess(InPawn);

	FActorSpawnParameters SpawnParamsLidar;
	FName LidarName("TurtleLidar");
	SpawnParamsLidar.Name = LidarName;
	TurtleLidar = GWorld->SpawnActor<ASensorLidar>(ASensorLidar::StaticClass(), SpawnParamsLidar);
	TurtleLidar->SetActorLocation(InPawn->GetActorLocation() + FVector(600,0,1700));
	TurtleLidar->AttachToActor(InPawn, FAttachmentTransformRules::KeepWorldTransform);
	TurtleLidar->nSamplesPerSecond = 1000;
	TurtleLidar->StartAngle = -120;
	TurtleLidar->FOVHorizontal = 240;
	TurtleLidar->Range = 10000;
	
	FActorSpawnParameters SpawnParamsNode;
	FName NodeName("TurtleNode");
	SpawnParamsNode.Name = NodeName;
	TurtleNode = GetWorld()->SpawnActor<AROS2Node>(AROS2Node::StaticClass(), SpawnParamsNode);
	TurtleNode->SetActorLocation(InPawn->GetActorLocation());
	TurtleNode->AttachToActor(InPawn, FAttachmentTransformRules::KeepWorldTransform);
	TurtleNode->Init();
	
	//TurtleNode->AddPublisher(FName("scan"), UROS2LidarPublisher::StaticClass(), 10, UROS2LaserScanMsg::StaticClass());
	UROS2LidarPublisher* LidarPub = NewObject<UROS2LidarPublisher>(this, UROS2LidarPublisher::StaticClass());
	LidarPub->TopicName = FName("scan");
	LidarPub->PublicationFrequencyHz = 10;
	LidarPub->MsgClass = UROS2LaserScanMsg::StaticClass();
	LidarPub->Lidar = TurtleLidar;
	TurtleNode->AddPublisher(LidarPub);

	SetupCommandTopicSubscription(Turtlebot);
}


void ATurtlebotAIController::OnUnPossess()
{
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
