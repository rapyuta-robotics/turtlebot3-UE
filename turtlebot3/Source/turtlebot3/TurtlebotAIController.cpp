// Copyright (C) Rapyuta Robotics


#include "TurtlebotAIController.h"

#include "TurtlebotVehicle.h"

#include "ROS2Node.h"
#include "ROS2TwistMsg.h"


ATurtlebotAIController::ATurtlebotAIController(const FObjectInitializer& ObjectInitializer) : Super(ObjectInitializer)
{
}


void ATurtlebotAIController::OnPossess(APawn *InPawn)
{
	Super::OnPossess(InPawn);

	TurtleNode = NewObject<AROS2Node>(AROS2Node::StaticClass());
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
		if (ensure(TurtleNode))
		{
			TurtleNode->Subscribe(TEXT("cmd_vel"), UROS2TwistMsg::StaticClass());
		}
	}
}
