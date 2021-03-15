// Copyright (C) Rapyuta Robotics


#include "TurtlebotAIController.h"

#include "TurtlebotVehicle.h"
#include "TurtlebotMovementComponent.h"

#include <ROS2Node.h>
#include <Msgs/ROS2TwistMsg.h>
#include <Msgs/ROS2LaserScanMsg.h>
#include <Sensors/SensorLidar.h>
#include <ROS2LidarPublisher.h>
#include "ROS2TFPublisher.h"
#include "ROS2OdomPublisher.h"

#include "Kismet/GameplayStatics.h"

ATurtlebotAIController::ATurtlebotAIController(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
	LidarClass = ASensorLidar::StaticClass();
}


void ATurtlebotAIController::OnPossess(APawn *InPawn)
{
	Super::OnPossess(InPawn);

	FActorSpawnParameters LidarSpawnParamsNode;
	// FName LidarName("TurtleLidar");
	// LidarSpawnParamsNode.Name = LidarName;
	TurtleLidar = GetWorld()->SpawnActor<ASensorLidar>(LidarClass, LidarSpawnParamsNode);
	TurtleLidar->SetActorLocation(InPawn->GetActorLocation() + FVector(-3.2,0,17.2));
	TurtleLidar->AttachToActor(InPawn, FAttachmentTransformRules::KeepWorldTransform);
	
	FActorSpawnParameters SpawnParamsNode;
	// FName NodeName("TurtleNode");
	// SpawnParamsNode.Name = NodeName;
	TurtleNode = GetWorld()->SpawnActor<AROS2Node>(AROS2Node::StaticClass(), SpawnParamsNode);
	TurtleNode->SetActorLocation(InPawn->GetActorLocation());
	TurtleNode->AttachToActor(InPawn, FAttachmentTransformRules::KeepWorldTransform);
	TurtleNode->Name = FName("UE4Node_" + FGuid::NewGuid().ToString());
	TurtleNode->Namespace = NAME_None;
	TurtleNode->Init();
	
	TurtleLidar->InitToNode(TurtleNode);
	TurtleLidar->LidarPublisher->Init();
	TurtleLidar->Run();

	TFPublisher = NewObject<UROS2TFPublisher>(this, UROS2TFPublisher::StaticClass());
	TFPublisher->RegisterComponent();
	TFPublisher->TopicName = FName("tf");
	TFPublisher->PublicationFrequencyHz = 60;
	TFPublisher->MsgClass = UROS2TFMsg::StaticClass();
	TFPublisher->Controller = this;
	TurtleNode->AddPublisher(TFPublisher);
	TFPublisher->Init();

	OdomPublisher = NewObject<UROS2OdomPublisher>(this, UROS2OdomPublisher::StaticClass());
	OdomPublisher->RegisterComponent();
	OdomPublisher->TopicName = FName("odom");
	OdomPublisher->PublicationFrequencyHz = 30;
	OdomPublisher->MsgClass = UROS2OdometryMsg::StaticClass();
	OdomPublisher->Controller = this;
	TurtleNode->AddPublisher(OdomPublisher);
	OdomPublisher->Init();

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

// this is complicated, from observing the gazebo example:
// it either publishes odom (1 element) or the 2 wheel states (2 elements)
TArray<FTFData> ATurtlebotAIController::GetTFData() const
{
	TArray<FTFData> retValue;

	// this should be TF_Static
	FTFData Footprint2Link;
	float TimeNow = UGameplayStatics::GetTimeSeconds(GWorld);
	Footprint2Link.sec = (int32_t)TimeNow;
	unsigned long long ns = (unsigned long long)(TimeNow * 1000000000.0f);
	Footprint2Link.nanosec = (uint32_t)(ns - (Footprint2Link.sec * 1000000000ul));

	Footprint2Link.frame_id = FName("base_footprint");
	Footprint2Link.child_frame_id = FName("base_link");

	Footprint2Link.translation = FVector(0,0,0);
	Footprint2Link.rotation = FQuat(0,0,0,1);

	retValue.Add(Footprint2Link);


	// this should be TF_Static
	FTFData Link2Scan;
	Link2Scan.sec = (int32_t)TimeNow;
	Link2Scan.nanosec = (uint32_t)(ns - (Link2Scan.sec * 1000000000ul));

	Link2Scan.frame_id = FName("base_link");
	Link2Scan.child_frame_id = FName("base_scan");

	Link2Scan.translation = FVector(0,0,.17);
	Link2Scan.rotation = FQuat(0,0,0,1);

	retValue.Add(Link2Scan);



	FTFData CurrentValue;
	CurrentValue.sec = (int32_t)TimeNow;
	CurrentValue.nanosec = (uint32_t)(ns - (CurrentValue.sec * 1000000000ul));

	CurrentValue.frame_id = FName("odom");
	CurrentValue.child_frame_id = FName("base_footprint");

	ATurtlebotVehicle *Vehicle = Turtlebot;
	CurrentValue.translation = Vehicle->GetActorLocation() / 100.f;
	CurrentValue.rotation = FQuat(Vehicle->GetActorRotation());

	retValue.Add(CurrentValue);

	return retValue;
}

struct FOdometryData ATurtlebotAIController::GetOdomData() const
{
	FOdometryData retValue;

	float TimeNow = UGameplayStatics::GetTimeSeconds(GWorld);
	retValue.sec = (int32_t)TimeNow;
	unsigned long long ns = (unsigned long long)(TimeNow * 1000000000.0f);
	retValue.nanosec = (uint32_t)(ns - (retValue.sec * 1000000000ul));

	//retValue.frame_id = FName("odom");
	retValue.frame_id = FName("odom");
	retValue.child_frame_id = FName("base_footprint");
	
	ATurtlebotVehicle *Vehicle = Turtlebot;
	UTurtlebotMovementComponent *TurtlebotMovementComponent = Cast<UTurtlebotMovementComponent>(Vehicle->GetMovementComponent());

	retValue.position = Vehicle->GetActorLocation();
	retValue.orientation = FQuat(Vehicle->GetActorRotation());
	retValue.pose_covariance.Init(0,36);
	retValue.pose_covariance[0] = 1;
	retValue.pose_covariance[7] = 1;
	retValue.pose_covariance[14] = 1;
	retValue.pose_covariance[21] = 1;
	retValue.pose_covariance[28] = 1;
	retValue.pose_covariance[35] = 1;

	retValue.linear = Vehicle->GetMovementComponent()->Velocity;
	retValue.angular = TurtlebotMovementComponent->AngularVelocity;
	retValue.twist_covariance.Init(0,36);
	retValue.twist_covariance[0] = 1;
	retValue.twist_covariance[7] = 1;
	retValue.twist_covariance[14] = 1;
	retValue.twist_covariance[21] = 1;
	retValue.twist_covariance[28] = 1;
	retValue.twist_covariance[35] = 1;

	return retValue;
}