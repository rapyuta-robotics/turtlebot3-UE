// Copyright (C) Rapyuta Robotics


#include "BurgerAIController.h"

#include "Turtlebot3_Burger.h"

#include <ROS2Node.h>
#include <Sensors/SensorLidar.h>
#include <ROS2Publisher.h>
#include <Msgs/ROS2TwistMsg.h>
#include <Msgs/ROS2LaserScanMsg.h>
#include <Msgs/ROS2ClockMsg.h>

#include "Math/Vector.h"
#include "Kismet/GameplayStatics.h"



ABurgerAIController::ABurgerAIController(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
}

void ABurgerAIController::OnPossess(APawn *InPawn)
{
	LidarOffset = FVector(0,0,15);

	FActorSpawnParameters LidarSpawnParamsNode;
	TurtleLidar = GWorld->SpawnActor<ASensorLidar>(LidarClass, LidarSpawnParamsNode);
	TurtleLidar->SetActorLocation(InPawn->GetActorLocation() + LidarOffset);
	TurtleLidar->SetActorRotation(FRotator(0,0,0));
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

	TurtleLidar->LidarPublisher->PublicationFrequencyHz = TurtleLidar->ScanFrequency;
	
	Super::OnPossess(InPawn);

	InitialPosition = Burger->GetActorLocation();
	InitialOrientation = Burger->GetActorRotation();
	//InitialOrientation.Yaw += 180;


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
			FSubscriptionCallback cb_clock;
			cb_clock.BindDynamic(this, &ABurgerAIController::ClockCallback);
			TurtleNode->AddSubscription(TEXT("clock"), UROS2ClockMsg::StaticClass(), cb_clock);

			FSubscriptionCallback cb_move;
			cb_move.BindDynamic(this, &ABurgerAIController::MovementCallback);
			TurtleNode->AddSubscription(TEXT("cmd_vel"), UROS2TwistMsg::StaticClass(), cb_move);
		}
	}
}

void ABurgerAIController::ClockCallback(const UROS2GenericMsg *Msg)
{
	const UROS2ClockMsg *Concrete = Cast<UROS2ClockMsg>(Msg);
	Concrete->PrintSubToLog();
}


void ABurgerAIController::MovementCallback(const UROS2GenericMsg *Msg)
{
	const UROS2TwistMsg *Concrete = Cast<UROS2TwistMsg>(Msg);

	if (IsValid(Concrete))
	{
		// TODO refactoring will be needed to put units and system of reference conversions in a consistent location
		// 	probably should not stay in msg though
		LinearVelTarget = Concrete->GetLinearVelocity()*100.f;
		AngularVelTarget = Concrete->GetAngularVelocity();

		//UE_LOG(LogTemp, Warning, TEXT("cmd_vel: %s %s to rot: %f %f"), *LinearVelTarget.ToString(), *AngularVelTarget.ToString(), LinearVelTarget.X - AngularVelTarget.Z * Burger->WheelSeparationHalf, LinearVelTarget.X + AngularVelTarget.Z * Burger->WheelSeparationHalf);

        Burger->SetTargetRotPerSFromVel(LinearVelTarget.X - AngularVelTarget.Z * Burger->WheelSeparationHalf, LinearVelTarget.X + AngularVelTarget.Z * Burger->WheelSeparationHalf);
	}
}

// this is complicated, from observing the gazebo example:
// it either publishes odom (1 element) or the 2 wheel states (2 elements)
TArray<FTFData> ABurgerAIController::GetTFData() const
{
	TArray<FTFData> retValue;

	float TimeNow = UGameplayStatics::GetTimeSeconds(GWorld);
	unsigned long long ns = (unsigned long long)(TimeNow * 1000000000.0f);
	
	FTFData Odom_Base;
	Odom_Base.sec = (int32_t)TimeNow;
	Odom_Base.nanosec = (uint32_t)(ns - (Odom_Base.sec * 1000000000ul));

	Odom_Base.frame_id = FString("odom");
	Odom_Base.child_frame_id = FString("base_footprint");

	Odom_Base.translation = (Burger->GetActorLocation()-InitialPosition) / 100.f;
	Odom_Base.translation.Y = -Odom_Base.translation.Y;
	Odom_Base.rotation = Burger->GetActorRotation().Quaternion() * InitialOrientation.Quaternion().Inverse();
	Odom_Base.rotation.X = -Odom_Base.rotation.X;
	Odom_Base.rotation.Z = -Odom_Base.rotation.Z;

	retValue.Add(Odom_Base);
	
	FTFData Base_WheelLeft;
	Base_WheelLeft.sec = (int32_t)TimeNow;
	Base_WheelLeft.nanosec = (uint32_t)(ns - (Base_WheelLeft.sec * 1000000000ul));

	Base_WheelLeft.frame_id = FString("base_link");
	Base_WheelLeft.child_frame_id = FString("wheel_left_link");

	Base_WheelLeft.translation = (Burger->Base_WheelLeft->GetRelativeLocation()) / 100.f;
	Base_WheelLeft.translation.Y = -Base_WheelLeft.translation.Y;
	Base_WheelLeft.rotation = Burger->Base_WheelLeft->GetRelativeRotation().Quaternion();
	Base_WheelLeft.rotation.X = -Base_WheelLeft.rotation.X;
	Base_WheelLeft.rotation.Z = -Base_WheelLeft.rotation.Z;

	retValue.Add(Base_WheelLeft);
	
	FTFData Base_WheelRight;
	Base_WheelRight.sec = (int32_t)TimeNow;
	Base_WheelRight.nanosec = (uint32_t)(ns - (Base_WheelRight.sec * 1000000000ul));

	Base_WheelRight.frame_id = FString("base_link");
	Base_WheelRight.child_frame_id = FString("wheel_right_link");

	Base_WheelRight.translation = (Burger->Base_WheelRight->GetRelativeLocation()) / 100.f;
	Base_WheelRight.translation.Y = -Base_WheelRight.translation.Y;
	Base_WheelRight.rotation = Burger->Base_WheelRight->GetRelativeRotation().Quaternion();
	Base_WheelRight.rotation.X = -Base_WheelRight.rotation.X;
	Base_WheelRight.rotation.Z = -Base_WheelRight.rotation.Z;

	retValue.Add(Base_WheelRight);

	return retValue;
}

TArray<FTFData> ABurgerAIController::GetTFStaticData() const
{
	TArray<FTFData> retValue;

	float TimeNow = UGameplayStatics::GetTimeSeconds(GWorld);
	unsigned long long ns = (unsigned long long)(TimeNow * 1000000000.0f);

	FTFData Footprint2Link;
	Footprint2Link.sec = (int32_t)TimeNow;
	Footprint2Link.nanosec = (uint32_t)(ns - (Footprint2Link.sec * 1000000000ul));

	Footprint2Link.frame_id = FString("base_footprint");
	Footprint2Link.child_frame_id = FString("base_link");

	Footprint2Link.translation = FVector(0,0,0);
	Footprint2Link.rotation = FQuat(0,0,0,1);

	retValue.Add(Footprint2Link);


	FTFData Link2CasterBackLink;
	Link2CasterBackLink.sec = (int32_t)TimeNow;
	Link2CasterBackLink.nanosec = (uint32_t)(ns - (Link2CasterBackLink.sec * 1000000000ul));

	Link2CasterBackLink.frame_id = FString("base_link");
	Link2CasterBackLink.child_frame_id = FString("caster_back_link");

	Link2CasterBackLink.translation = Burger->Base_CasterBack->GetRelativeLocation() / 100.f;
	Link2CasterBackLink.rotation = Burger->Base_CasterBack->GetRelativeRotation().Quaternion();

	retValue.Add(Link2CasterBackLink);


	FTFData Link2Scan;
	Link2Scan.sec = (int32_t)TimeNow;
	Link2Scan.nanosec = (uint32_t)(ns - (Link2Scan.sec * 1000000000ul));

	Link2Scan.frame_id = FString("base_link");
	Link2Scan.child_frame_id = FString("base_scan");

	Link2Scan.translation = LidarOffset / 100.f;
	Link2Scan.rotation = FQuat(0,0,0,1);

	retValue.Add(Link2Scan);

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
	
	retValue.position = (Burger->GetActorLocation() - InitialPosition) / 100.f;
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

	retValue.linear = LinearVelTarget / 100.f;
	retValue.angular = AngularVelTarget;
	retValue.angular.Z = -retValue.angular.Z;
	retValue.twist_covariance.Init(0,36);
	retValue.twist_covariance[0] = 0.00001;
	retValue.twist_covariance[7] = 0.00001;
	retValue.twist_covariance[14] = 1000000000000.0;
	retValue.twist_covariance[21] = 1000000000000.0;
	retValue.twist_covariance[28] = 1000000000000.0;
	retValue.twist_covariance[35] = 0.001;

	return retValue;
}