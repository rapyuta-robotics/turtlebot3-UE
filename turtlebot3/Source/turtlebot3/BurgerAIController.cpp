// Copyright (C) Rapyuta Robotics


#include "BurgerAIController.h"

#include "Turtlebot3_Burger.h"

#include "ROS2Node.h"
#include "Msgs/ROS2TwistMsg.h"
#include "Msgs/ROS2LaserScanMsg.h"
#include "Sensors/SensorLidar.h"
#include "ROS2LidarPublisher.h"
#include "ROS2TFPublisher.h"
#include "ROS2OdomPublisher.h"

#include "Math/Vector.h"
#include "Kismet/GameplayStatics.h"



ABurgerAIController::ABurgerAIController(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
	LidarClass = ASensorLidar::StaticClass();
}

void ABurgerAIController::OnPossess(APawn *InPawn)
{
	FActorSpawnParameters LidarSpawnParamsNode;
	TurtleLidar = GWorld->SpawnActor<ASensorLidar>(LidarClass, LidarSpawnParamsNode);
	TurtleLidar->SetActorLocation(InPawn->GetActorLocation() + FVector(0,0,15));
	Burger = Cast<ATurtlebot3_Burger>(InPawn);
	TurtleLidar->AttachToComponent(Burger->LidarSensor, FAttachmentTransformRules::KeepWorldTransform);
	
	FActorSpawnParameters SpawnParamsNode;
	TurtleNode = GWorld->SpawnActor<AROS2Node>(AROS2Node::StaticClass(), SpawnParamsNode);
	TurtleNode->SetActorLocation(InPawn->GetActorLocation());
	TurtleNode->AttachToActor(InPawn, FAttachmentTransformRules::KeepWorldTransform);
	TurtleNode->Name = FString("UE4Node_" + FGuid::NewGuid().ToString());
	TurtleNode->Namespace = FString();
	TurtleNode->Init();
	
    TurtleLidar->nSamplesPerScan = 360;
    TurtleLidar->ScanFrequency = 5;
    TurtleLidar->StartAngle = 0;
    TurtleLidar->FOVHorizontal = 360;
    TurtleLidar->MinRange = 12;
    TurtleLidar->MaxRange = 350;
	
	Super::OnPossess(InPawn);

	InitialPosition = Burger->GetActorLocation();
	InitialOrientation = Burger->GetActorRotation();
	InitialOrientation.Yaw += 180;

	SetupSubscription(Burger);
}


void ABurgerAIController::SetPawn(APawn *InPawn)
{
	Super::SetPawn(InPawn);

	Burger = Cast<ATurtlebot3_Burger>(InPawn);
}


void ABurgerAIController::SetupSubscription(ATurtlebot3_Burger *InPawn)
{
	if (IsValid(InPawn))
	{
		// Subscription with callback to enqueue vehicle spawn info.
		if (ensure(IsValid(TurtleNode)))
		{
			FSubscriptionCallback cb;
			cb.BindDynamic(this, &ABurgerAIController::MovementCallback);
			TurtleNode->AddSubscription(TEXT("cmd_vel"), UROS2TwistMsg::StaticClass(), cb);

			TurtleNode->Subscribe();
		}
	}
}


void ABurgerAIController::MovementCallback(const UROS2GenericMsg *Msg)
{
	const UROS2TwistMsg *Concrete = Cast<UROS2TwistMsg>(Msg);

	if (IsValid(Concrete))
	{
		// TODO refactoring will be needed to put units and system of reference conversions in a consistent location
		// 	probably should not stay in msg though
		FVector linear(Concrete->GetLinearVelocity()*100.f);
		FVector angular(Concrete->GetAngularVelocity());

        Burger->SetTargetRotPerSFromVel(linear.X - angular.Z * Burger->WheelSeparationHalf, linear.X + angular.Z * Burger->WheelSeparationHalf);
	}
}

// this is complicated, from observing the gazebo example:
// it either publishes odom (1 element) or the 2 wheel states (2 elements)
TArray<FTFData> ABurgerAIController::GetTFData() const
{
	TArray<FTFData> retValue;

	// this should be TF_Static
	FTFData Footprint2Link;
	float TimeNow = UGameplayStatics::GetTimeSeconds(GWorld);
	Footprint2Link.sec = (int32_t)TimeNow;
	unsigned long long ns = (unsigned long long)(TimeNow * 1000000000.0f);
	Footprint2Link.nanosec = (uint32_t)(ns - (Footprint2Link.sec * 1000000000ul));

	Footprint2Link.frame_id = FString("base_footprint");
	Footprint2Link.child_frame_id = FString("base_link");

	Footprint2Link.translation = FVector(0,0,0);
	Footprint2Link.rotation = FQuat(0,0,0,1);

	retValue.Add(Footprint2Link);


	// this should be TF_Static
	FTFData Link2Scan;
	Link2Scan.sec = (int32_t)TimeNow;
	Link2Scan.nanosec = (uint32_t)(ns - (Link2Scan.sec * 1000000000ul));

	Link2Scan.frame_id = FString("base_link");
	Link2Scan.child_frame_id = FString("base_scan");

	Link2Scan.translation = FVector(0,0,.17);
	Link2Scan.rotation = FQuat(0,0,0,1);

	retValue.Add(Link2Scan);



	FTFData CurrentValue;
	CurrentValue.sec = (int32_t)TimeNow;
	CurrentValue.nanosec = (uint32_t)(ns - (CurrentValue.sec * 1000000000ul));

	CurrentValue.frame_id = FString("odom");
	CurrentValue.child_frame_id = FString("base_footprint");

	CurrentValue.translation = (Burger->GetActorLocation()-InitialPosition) / 100.f;
	CurrentValue.translation.Y = -CurrentValue.translation.Y;
	CurrentValue.rotation = Burger->GetActorRotation().Quaternion() * InitialOrientation.Quaternion().Inverse();
	CurrentValue.rotation.X = -CurrentValue.rotation.X;
	CurrentValue.rotation.Z = -CurrentValue.rotation.Z;

	retValue.Add(CurrentValue);

	return retValue;
}

struct FOdometryData ABurgerAIController::GetOdomData() const
{
	FOdometryData retValue;

	float TimeNow = UGameplayStatics::GetTimeSeconds(GWorld);
	retValue.sec = (int32_t)TimeNow;
	unsigned long long ns = (unsigned long long)(TimeNow * 1000000000.0f);
	retValue.nanosec = (uint32_t)(ns - (retValue.sec * 1000000000ul));

	retValue.frame_id = FString("odom");
	retValue.child_frame_id = FString("base_footprint");
	
	// ATurtlebotVehicle *Vehicle = Turtlebot;
	// UTurtlebotMovementComponent *TurtlebotMovementComponent = Cast<UTurtlebotMovementComponent>(Vehicle->GetMovementComponent());

	retValue.position = (Burger->GetActorLocation()-InitialPosition) / 100.f;
	retValue.position.Y = -retValue.position.Y;
	retValue.orientation = InitialOrientation.Quaternion();
	retValue.orientation.X = -retValue.orientation.X;
	retValue.orientation.Z = -retValue.orientation.Z;
	retValue.pose_covariance.Init(0,36);
	retValue.pose_covariance[0] = 0.00001;
	retValue.pose_covariance[7] = 0.00001;
	retValue.pose_covariance[14] = 1000000000000.0;
	retValue.pose_covariance[21] = 1000000000000.0;
	retValue.pose_covariance[28] = 1000000000000.0;
	retValue.pose_covariance[35] = 0.001;

	// retValue.linear = Vehicle->GetMovementComponent()->Velocity / 100.0f;
	// retValue.angular = FMath::DegreesToRadians(TurtlebotMovementComponent->AngularVelocity);
	retValue.angular.Z = -retValue.angular.Z;
	retValue.twist_covariance.Init(0,36);
	retValue.twist_covariance[0] = 1;
	retValue.twist_covariance[7] = 1;
	retValue.twist_covariance[14] = 1;
	retValue.twist_covariance[21] = 1;
	retValue.twist_covariance[28] = 1;
	retValue.twist_covariance[35] = 1;

	return retValue;
}