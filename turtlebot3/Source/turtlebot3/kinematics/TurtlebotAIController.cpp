// Copyright (C) Rapyuta Robotics


#include "TurtlebotAIController.h"

#include "TurtlebotVehicle.h"
#include "TurtlebotMovementComponent.h"

#include <ROS2Node.h>
#include <ROS2Publisher.h>
#include <Msgs/ROS2TwistMsg.h>
#include <Msgs/ROS2LaserScanMsg.h>
#include <Sensors/SensorLidar.h>

#include "Kismet/GameplayStatics.h"

ATurtlebotAIController::ATurtlebotAIController(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
	LidarClass = ASensorLidar::StaticClass();
}

void ATurtlebotAIController::OnPossess(APawn *InPawn)
{
	Super::OnPossess(InPawn);

	if (TurtleLidar == nullptr)
	{
		LidarOffset = FVector(-3.2,0,17.2);
		FActorSpawnParameters LidarSpawnParamsNode;
		TurtleLidar = GetWorld()->SpawnActor<ASensorLidar>(LidarClass, LidarSpawnParamsNode);
		TurtleLidar->SetActorLocation(InPawn->GetActorLocation() + LidarOffset);
		TurtleLidar->AttachToActor(InPawn, FAttachmentTransformRules::KeepWorldTransform);
	}
	
	if (TurtleNode == nullptr)
	{
		FActorSpawnParameters SpawnParamsNode;
		TurtleNode = GetWorld()->SpawnActor<AROS2Node>(AROS2Node::StaticClass(), SpawnParamsNode);
		TurtleNode->SetActorLocation(InPawn->GetActorLocation());
		TurtleNode->AttachToActor(InPawn, FAttachmentTransformRules::KeepWorldTransform);
		TurtleNode->Name = FString("UE4Node_" + FGuid::NewGuid().ToString());
		TurtleNode->Namespace = FString();
		TurtleNode->Init();
	}
	
	TurtleLidar->InitToNode(TurtleNode);
	TurtleLidar->Run();

	TFPublisher = NewObject<UROS2Publisher>(this, UROS2Publisher::StaticClass());
	TFPublisher->RegisterComponent();
	TFPublisher->TopicName = FString("tf");
	TFPublisher->PublicationFrequencyHz = 50;
	TFPublisher->MsgClass = UROS2TFMsg::StaticClass();
	TFPublisher->UpdateDelegate.BindDynamic(this, &ATurtlebotAIController::TFMessageUpdate);
	TurtleNode->AddPublisher(TFPublisher);
	TFPublisher->Init();

	TFStaticPublisher = NewObject<UROS2Publisher>(this, UROS2Publisher::StaticClass());
	TFStaticPublisher->RegisterComponent();
	TFStaticPublisher->TopicName = FString("tf_static");
	TFStaticPublisher->PublicationFrequencyHz = 10;
	TFStaticPublisher->MsgClass = UROS2TFMsg::StaticClass();
	TFStaticPublisher->UpdateDelegate.BindDynamic(this, &ATurtlebotAIController::TFStaticMessageUpdate);
	TurtleNode->AddPublisher(TFStaticPublisher);
	TFStaticPublisher->Init(true);

	OdomPublisher = NewObject<UROS2Publisher>(this, UROS2Publisher::StaticClass());
	OdomPublisher->RegisterComponent();
	OdomPublisher->TopicName = FString("odom");
	OdomPublisher->PublicationFrequencyHz = 30;
	OdomPublisher->MsgClass = UROS2OdometryMsg::StaticClass();
	OdomPublisher->UpdateDelegate.BindDynamic(this, &ATurtlebotAIController::OdomMessageUpdate);
	TurtleNode->AddPublisher(OdomPublisher);
	OdomPublisher->Init();

	if (Turtlebot != nullptr)
	{
		InitialPosition = Turtlebot->GetActorLocation();
		InitialOrientation = Turtlebot->GetActorRotation();
		InitialOrientation.Yaw += 180;

		SetupCommandTopicSubscription(Turtlebot);
	}
}


void ATurtlebotAIController::OnUnPossess()
{
	TurtleLidar = nullptr;
	TurtleNode = nullptr;
	TFPublisher = nullptr;
	OdomPublisher = nullptr;

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
		}
	}
}


void ATurtlebotAIController::TFMessageUpdate(UROS2GenericMsg *TopicMessage)
{
    UROS2TFMsg *TFMessage = Cast<UROS2TFMsg>(TopicMessage);
    TFMessage->Update(GetTFData());
}

void ATurtlebotAIController::TFStaticMessageUpdate(UROS2GenericMsg *TopicMessage)
{
    UROS2TFMsg *TFMessage = Cast<UROS2TFMsg>(TopicMessage);
    TFMessage->Update(GetTFStaticData());
}


void ATurtlebotAIController::OdomMessageUpdate(UROS2GenericMsg *TopicMessage)
{
    UROS2OdometryMsg *OdomMessage = Cast<UROS2OdometryMsg>(TopicMessage);
    OdomMessage->Update(GetOdomData());
}


void ATurtlebotAIController::MovementCallback(const UROS2GenericMsg *Msg)
{
	const UROS2TwistMsg *Concrete = Cast<UROS2TwistMsg>(Msg);

	if (IsValid(Concrete))
	{
		// TODO refactoring will be needed to put units and system of reference conversions in a consistent location
		// 	probably should not stay in msg though
		FVector linear(Concrete->GetLinearVelocity()*100.f);
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

	float TimeNow = UGameplayStatics::GetTimeSeconds(GWorld);
	unsigned long long ns = (unsigned long long)(TimeNow * 1000000000.0f);

	FTFData CurrentValue;
	CurrentValue.sec = (int32_t)TimeNow;
	CurrentValue.nanosec = (uint32_t)(ns - (CurrentValue.sec * 1000000000ul));

	CurrentValue.frame_id = FString("odom");
	CurrentValue.child_frame_id = FString("base_footprint");

	ATurtlebotVehicle *Vehicle = Turtlebot;
	CurrentValue.translation = (Vehicle->GetActorLocation()-InitialPosition) / 100.f;
	CurrentValue.translation.Y = -CurrentValue.translation.Y;
	//CurrentValue.rotation = InitialOrientation.Quaternion() * Vehicle->GetActorRotation().Quaternion().Inverse();
	CurrentValue.rotation = Vehicle->GetActorRotation().Quaternion() * InitialOrientation.Quaternion().Inverse();
	//CurrentValue.rotation = InitialOrientation.Quaternion();
	CurrentValue.rotation.X = -CurrentValue.rotation.X;
	CurrentValue.rotation.Z = -CurrentValue.rotation.Z;

	retValue.Add(CurrentValue);

	return retValue;
}

TArray<FTFData> ATurtlebotAIController::GetTFStaticData() const
{
	TArray<FTFData> retValue;

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


	FTFData Link2Scan;
	Link2Scan.sec = (int32_t)TimeNow;
	Link2Scan.nanosec = (uint32_t)(ns - (Link2Scan.sec * 1000000000ul));

	Link2Scan.frame_id = FString("base_link");
	Link2Scan.child_frame_id = FString("base_scan");

	Link2Scan.translation = LidarOffset;
	Link2Scan.rotation = FQuat(0,0,0,1);

	retValue.Add(Link2Scan);

	return retValue;
}

struct FOdometryData ATurtlebotAIController::GetOdomData() const
{
	FOdometryData retValue;

	float TimeNow = UGameplayStatics::GetTimeSeconds(GWorld);
	retValue.sec = (int32_t)TimeNow;
	unsigned long long ns = (unsigned long long)(TimeNow * 1000000000.0f);
	retValue.nanosec = (uint32_t)(ns - (retValue.sec * 1000000000ul));

	retValue.frame_id = FString("odom");
	retValue.child_frame_id = FString("base_footprint");
	
	ATurtlebotVehicle *Vehicle = Turtlebot;
	UTurtlebotMovementComponent *TurtlebotMovementComponent = Cast<UTurtlebotMovementComponent>(Vehicle->GetMovementComponent());

	retValue.position = (Vehicle->GetActorLocation()-InitialPosition) / 100.f;
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

	retValue.linear = Vehicle->GetMovementComponent()->Velocity / 100.0f;
	retValue.angular = FMath::DegreesToRadians(TurtlebotMovementComponent->AngularVelocity);
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