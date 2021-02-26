// Copyright (C) Rapyuta Robotics


#include "TurtlebotAIController.h"

#include "TurtlebotVehicle.h"

#include "ROS2Node.h"
#include "Msgs/ROS2TwistMsg.h"


ATurtlebotAIController::ATurtlebotAIController(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
}


void ATurtlebotAIController::OnPossess(APawn *InPawn)
{
	Super::OnPossess(InPawn);
	
	FActorSpawnParameters SpawnParams;
	FName Name("TurtleNode");
	SpawnParams.Name = Name;
	TurtleNode = GetWorld()->SpawnActor<AROS2Node>(AROS2Node::StaticClass(), SpawnParams);
	TurtleNode->SetActorLocation(InPawn->GetActorLocation());
	TurtleNode->AttachToActor(InPawn, FAttachmentTransformRules::KeepWorldTransform);
	TurtleNode->Init();

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
        /*
		// Create a std::function callback object
		std::function<void(TSharedPtr<FROSBaseMsg>)> SubscribeCallback = [VehiclePawn](TSharedPtr<FROSBaseMsg> msg) -> void
		{
			auto Concrete = StaticCastSharedPtr<ROSMessages::geometry_msgs::Twist>(msg);

			if (Concrete.IsValid())
			{
				FVector linear(Concrete->linear.x, Concrete->linear.y, Concrete->linear.z);
				FVector angular(Concrete->angular.x, Concrete->angular.y, Concrete->angular.z);

				AsyncTask(ENamedThreads::GameThread, [linear, angular, VehiclePawn]
				{
					if (IsValid(VehiclePawn))
					{
						VehiclePawn->SetLinearVel(linear);
						VehiclePawn->SetAngularVel(angular);
					}
				});
			}
		};
        */

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
	UE_LOG(LogTemp, Log, TEXT("MovementCallback"));
}
