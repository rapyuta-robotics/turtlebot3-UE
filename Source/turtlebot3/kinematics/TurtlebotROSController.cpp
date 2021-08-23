// Copyright (C) Rapyuta Robotics


#include "TurtlebotROSController.h"

#include "TurtlebotBurgerVehicle.h"
#include "RobotVehicleMovementComponent.h"
#include "Tools/UEUtilities.h"

#include <ROS2Node.h>
#include <ROS2Publisher.h>
#include <Msgs/ROS2TwistMsg.h>
#include <Msgs/ROS2LaserScanMsg.h>
#include <Sensors/SensorLidar.h>

#include "Kismet/GameplayStatics.h"

ATurtlebotROSController::ATurtlebotROSController(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
	LidarClass = ASensorLidar::StaticClass();
}

void ATurtlebotROSController::OnPossess(APawn *InPawn)
{
	Super::OnPossess(InPawn);

	if (TurtleLidar == nullptr)
	{
		LidarOffset = FVector(-3.2,0,17.2);
		FActorSpawnParameters LidarSpawnParamsNode;
		TurtleLidar = GetWorld()->SpawnActor<ASensorLidar>(LidarClass, LidarSpawnParamsNode);
		TurtleLidar->SetActorLocation(InPawn->GetActorLocation() + LidarOffset);
		TurtleLidar->AttachToActor(InPawn, FAttachmentTransformRules::KeepWorldTransform);
		TurtleLidar->nSamplesPerScan = 360;
		TurtleLidar->ScanFrequency = 5;
		TurtleLidar->StartAngle = 0;
		TurtleLidar->FOVHorizontal = 360;
		TurtleLidar->MinRange = 12;
		TurtleLidar->MaxRange = 350;
		TurtleLidar->ScanFrequency = 30;
		TurtleLidar->LidarPublisher->PublicationFrequencyHz = TurtleLidar->ScanFrequency;
	}
	
	if (TurtleNode == nullptr)
	{
		FActorSpawnParameters SpawnParamsNode;
		TurtleNode = GetWorld()->SpawnActor<AROS2Node>(AROS2Node::StaticClass(), SpawnParamsNode);
		TurtleNode->SetActorLocation(InPawn->GetActorLocation());
		TurtleNode->AttachToActor(InPawn, FAttachmentTransformRules::KeepWorldTransform);
		TurtleNode->Name = TEXT("UE4Node_" + FGuid::NewGuid().ToString());
		TurtleNode->Namespace = FString();
		TurtleNode->Init();
	}
	
	TurtleLidar->InitToNode(TurtleNode);
	TurtleLidar->Run();

	URobotVehicleMovementComponent *RobotVehicleMovementComponent = Cast<URobotVehicleMovementComponent>(InPawn->GetMovementComponent());
	TFPublisher = NewObject<UROS2TFPublisher>(this, UROS2TFPublisher::StaticClass());
	TFPublisher->RegisterComponent();
	TFPublisher->FrameId = RobotVehicleMovementComponent->FrameId = TEXT("odom");
	TFPublisher->ChildFrameId = RobotVehicleMovementComponent->ChildFrameId = TEXT("base_footprint");
	TFPublisher->PublicationFrequencyHz = 50;
	TFPublisher->InitTFPublisher(TurtleNode);

	OdomPublisher = NewObject<UROS2Publisher>(this, UROS2Publisher::StaticClass());
	OdomPublisher->RegisterComponent();
	OdomPublisher->TopicName = TEXT("odom");
	OdomPublisher->PublicationFrequencyHz = 30;
	OdomPublisher->MsgClass = UROS2OdometryMsg::StaticClass();
	OdomPublisher->UpdateDelegate.BindDynamic(this, &ATurtlebotROSController::OdomMessageUpdate);
	TurtleNode->AddPublisher(OdomPublisher);
	OdomPublisher->Init(UROS2QoS::KeepLast);

	if (Turtlebot != nullptr)
	{
		InitialPosition = Turtlebot->GetActorLocation();
		InitialOrientation = Turtlebot->GetActorRotation();
		InitialOrientation.Yaw += 180;

		SetupCommandTopicSubscription(Turtlebot);
	}
}


void ATurtlebotROSController::OnUnPossess()
{
	TurtleLidar = nullptr;
	TurtleNode = nullptr;
	TFPublisher = nullptr;
	OdomPublisher = nullptr;

	Super::OnUnPossess();
}


void ATurtlebotROSController::SetPawn(APawn *InPawn)
{
	Super::SetPawn(InPawn);

	Turtlebot = Cast<ATurtlebotBurgerVehicle>(InPawn);
}


void ATurtlebotROSController::SetupCommandTopicSubscription(ATurtlebotBurgerVehicle *InPawn)
{
	if (IsValid(InPawn))
	{
		// Subscription with callback to enqueue vehicle spawn info.
		if (ensure(IsValid(TurtleNode)))
		{
			FSubscriptionCallback cb;
			cb.BindDynamic(this, &ATurtlebotROSController::MovementCallback);
			TurtleNode->AddSubscription(TEXT("cmd_vel"), UROS2TwistMsg::StaticClass(), cb);
		}
	}
}

void ATurtlebotROSController::OdomMessageUpdate(UROS2GenericMsg *TopicMessage)
{
    UROS2OdometryMsg *OdomMessage = Cast<UROS2OdometryMsg>(TopicMessage);
    OdomMessage->Update(GetOdomData());
}


void ATurtlebotROSController::MovementCallback(const UROS2GenericMsg *Msg)
{
	const UROS2TwistMsg *Concrete = Cast<UROS2TwistMsg>(Msg);

	if (IsValid(Concrete))
	{
		// TODO refactoring will be needed to put units and system of reference conversions in a consistent location
		// 	probably should not stay in msg though
		FVector linear(ConversionUtils::VectorROSToUE(Concrete->GetLinearVelocity()));
		FVector angular(ConversionUtils::RotationROSToUE(Concrete->GetAngularVelocity()));
		ATurtlebotBurgerVehicle *Vehicle = Turtlebot;

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

struct FOdometryData ATurtlebotROSController::GetOdomData() const
{
	ATurtlebotBurgerVehicle *Vehicle = Turtlebot;
	URobotVehicleMovementComponent *RobotVehicleMovementComponent = Cast<URobotVehicleMovementComponent>(Vehicle->GetMovementComponent());
	TFPublisher->TF = RobotVehicleMovementComponent->GetOdomTF();
	
	FOdometryData res = RobotVehicleMovementComponent->OdomData;
	res.position = ConversionUtils::VectorUEToROS(res.position);
	res.orientation = ConversionUtils::QuatUEToROS(res.orientation);
	res.linear = ConversionUtils::VectorUEToROS(res.linear);
	res.angular = ConversionUtils::VectorUEToROS(res.angular);
	return res;
}