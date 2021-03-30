// Copyright (C) Rapyuta Robotics


#include "Turtlebot3_Burger.h"
#include "TurtlebotMovementComponent.h"
#include "BurgerAIController.h"


// Sets default values
ATurtlebot3_Burger::ATurtlebot3_Burger(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	AIControllerClass = ABurgerAIController::StaticClass();
	
	// Meshes
	if (VehicleMaterial == nullptr)
	{
		static ConstructorHelpers::FObjectFinder<UMaterial> RobotMaterial(TEXT("Material'/Game/Blueprints/RobotMat.RobotMat'"));
		VehicleMaterial = RobotMaterial.Object;
	}
    static ConstructorHelpers::FObjectFinder<UStaticMesh> BaseMesh(TEXT("'/Game/Burger/burger_base.burger_base'"));
    static ConstructorHelpers::FObjectFinder<UStaticMesh> LidarMesh(TEXT("'/Game/Burger/burger_lds.burger_lds'"));
    static ConstructorHelpers::FObjectFinder<UStaticMesh> WheelLMesh(TEXT("'/Game/Burger/burger_left_wheel.burger_left_wheel'"));
    static ConstructorHelpers::FObjectFinder<UStaticMesh> WheelRMesh(TEXT("'/Game/Burger/burger_right_wheel.burger_right_wheel'"));
    static ConstructorHelpers::FObjectFinder<UStaticMesh> CasterMesh(TEXT("'/Game/Burger/burger_caster_back.burger_caster_back'"));

    Base = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Base"));
	LidarSensor = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("LidarSensor"));
    WheelLeft = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("WheelLeft"));
    WheelRight = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("WheelRight"));
    CasterBack = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("CasterBack"));

	RootComponent = Base;

    Base->SetupAttachment(RootComponent);
    LidarSensor->SetupAttachment(RootComponent);
    WheelLeft->SetupAttachment(RootComponent);
    WheelRight->SetupAttachment(RootComponent);
    CasterBack->SetupAttachment(RootComponent);

	Base->SetStaticMesh(BaseMesh.Object);
	LidarSensor->SetStaticMesh(LidarMesh.Object);
	WheelLeft->SetStaticMesh(WheelLMesh.Object);
	WheelRight->SetStaticMesh(WheelRMesh.Object);
    CasterBack->SetStaticMesh(CasterMesh.Object);

	Base->SetMaterial(0, VehicleMaterial);
	LidarSensor->SetMaterial(0, VehicleMaterial);
	WheelLeft->SetMaterial(0, VehicleMaterial);
	WheelRight->SetMaterial(0, VehicleMaterial);
	CasterBack->SetMaterial(0, VehicleMaterial);

	Base->SetSimulatePhysics(true);
	LidarSensor->SetSimulatePhysics(true);
	WheelLeft->SetSimulatePhysics(true);
	WheelRight->SetSimulatePhysics(true);
	CasterBack->SetSimulatePhysics(true);


	// Constraints
	Base_LidarSensor = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("Base_LidarSensor"));
    Base_LidarSensor->SetupAttachment(RootComponent);
	
	Base_WheelLeft = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("Base_WheelLeft"));
    Base_WheelLeft->SetupAttachment(RootComponent);
	
	Base_WheelRight = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("Base_WheelRight"));
    Base_WheelRight->SetupAttachment(RootComponent);
	
	Base_CasterBack = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("Base_CasterBack"));
    Base_CasterBack->SetupAttachment(RootComponent);

	IsInitialized = true;

	SetupConstraintsAndPhysics();
}

void ATurtlebot3_Burger::Init()
{
	if (!IsInitialized)
	{
		AIControllerClass = ABurgerAIController::StaticClass();
		
		// Meshes
		if (VehicleMaterial == nullptr)
		{
			UMaterial* RobotMaterial = Cast<UMaterial>(StaticLoadObject(UMaterial::StaticClass(), NULL, TEXT("Material'/Game/Blueprints/RobotMat.RobotMat'")));
			VehicleMaterial = RobotMaterial;
		}
		UStaticMesh* BaseMesh = Cast<UStaticMesh>(StaticLoadObject(UStaticMesh::StaticClass(), NULL, TEXT("'/Game/Burger/burger_base.burger_base'")));
		UStaticMesh* LidarMesh = Cast<UStaticMesh>(StaticLoadObject(UStaticMesh::StaticClass(), NULL, TEXT("'/Game/Burger/burger_lds.burger_lds'")));
		UStaticMesh* WheelLMesh = Cast<UStaticMesh>(StaticLoadObject(UStaticMesh::StaticClass(), NULL, TEXT("'/Game/Burger/burger_left_wheel.burger_left_wheel'")));
		UStaticMesh* WheelRMesh = Cast<UStaticMesh>(StaticLoadObject(UStaticMesh::StaticClass(), NULL, TEXT("'/Game/Burger/burger_right_wheel.burger_right_wheel'")));
		UStaticMesh* CasterMesh = Cast<UStaticMesh>(StaticLoadObject(UStaticMesh::StaticClass(), NULL, TEXT("'/Game/Burger/burger_caster_back.burger_caster_back'")));

		Base = NewObject<UStaticMeshComponent>(this, UStaticMeshComponent::StaticClass(), FName("Base"));
		LidarSensor = NewObject<UStaticMeshComponent>(this, UStaticMeshComponent::StaticClass(), FName("LidarSensor"));
		WheelLeft = NewObject<UStaticMeshComponent>(this, UStaticMeshComponent::StaticClass(), FName("WheelLeft"));
		WheelRight = NewObject<UStaticMeshComponent>(this, UStaticMeshComponent::StaticClass(), FName("WheelRight"));
		CasterBack = NewObject<UStaticMeshComponent>(this, UStaticMeshComponent::StaticClass(), FName("CasterBack"));

		RootComponent = Base;

		Base->SetupAttachment(RootComponent);
		LidarSensor->SetupAttachment(RootComponent);
		WheelLeft->SetupAttachment(RootComponent);
		WheelRight->SetupAttachment(RootComponent);
		CasterBack->SetupAttachment(RootComponent);

		Base->SetStaticMesh(BaseMesh);
		LidarSensor->SetStaticMesh(LidarMesh);
		WheelLeft->SetStaticMesh(WheelLMesh);
		WheelRight->SetStaticMesh(WheelRMesh);
		CasterBack->SetStaticMesh(CasterMesh);


		// Constraints
		Base_LidarSensor = NewObject<UPhysicsConstraintComponent>(this, UPhysicsConstraintComponent::StaticClass(), FName("Base_LidarSensor"));
		Base_LidarSensor->SetupAttachment(RootComponent);
		
		Base_WheelLeft = NewObject<UPhysicsConstraintComponent>(this, UPhysicsConstraintComponent::StaticClass(), FName("Base_WheelLeft"));
		Base_WheelLeft->SetupAttachment(RootComponent);
		
		Base_WheelRight = NewObject<UPhysicsConstraintComponent>(this, UPhysicsConstraintComponent::StaticClass(), FName("Base_WheelRight"));
		Base_WheelRight->SetupAttachment(RootComponent);
		
		Base_CasterBack = NewObject<UPhysicsConstraintComponent>(this, UPhysicsConstraintComponent::StaticClass(), FName("Base_CasterBack"));
		Base_CasterBack->SetupAttachment(RootComponent);
		
		IsInitialized = true;

		SetupConstraintsAndPhysics();
	}
}

void ATurtlebot3_Burger::SetupConstraintsAndPhysics()
{
	if (IsInitialized)
	{
		Base->SetMaterial(0, VehicleMaterial);
		LidarSensor->SetMaterial(0, VehicleMaterial);
		WheelLeft->SetMaterial(0, VehicleMaterial);
		WheelRight->SetMaterial(0, VehicleMaterial);
		CasterBack->SetMaterial(0, VehicleMaterial);

		Base->SetSimulatePhysics(true);
		LidarSensor->SetSimulatePhysics(true);
		WheelLeft->SetSimulatePhysics(true);
		WheelRight->SetSimulatePhysics(true);
		CasterBack->SetSimulatePhysics(true);

		Base_LidarSensor->ComponentName2.ComponentName = TEXT("Base");
		Base_LidarSensor->ComponentName1.ComponentName = TEXT("LidarSensor");
		Base_LidarSensor->SetDisableCollision(true);
		Base_LidarSensor->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0);
		Base_LidarSensor->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 0);
		Base_LidarSensor->SetAngularTwistLimit(EAngularConstraintMotion::ACM_Locked, 0);
		Base_LidarSensor->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0);
		Base_LidarSensor->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0);
		Base_LidarSensor->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0);

		WheelSeparationHalf = 8;

		Base_WheelLeft->ComponentName2.ComponentName = TEXT("Base");
		Base_WheelLeft->ComponentName1.ComponentName = TEXT("WheelLeft");
		Base_WheelLeft->SetDisableCollision(true);
		Base_WheelLeft->SetRelativeLocation(FVector(-8,0,2.3));
		Base_WheelLeft->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
		Base_WheelLeft->SetAngularDriveParams(MaxForce, MaxForce, MaxForce);
		Base_WheelLeft->SetAngularVelocityDriveTwistAndSwing(true,false);
		Base_WheelLeft->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0);
		Base_WheelLeft->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 0);
		Base_WheelLeft->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0);
		Base_WheelLeft->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0);
		Base_WheelLeft->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0);

		Base_WheelRight->ComponentName2.ComponentName = TEXT("Base");
		Base_WheelRight->ComponentName1.ComponentName = TEXT("WheelRight");
		Base_WheelRight->SetDisableCollision(true);
		Base_WheelRight->SetRelativeLocation(FVector(8,0,2.3));
		Base_WheelRight->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
		Base_WheelRight->SetAngularDriveParams(MaxForce, MaxForce, MaxForce);
		Base_WheelRight->SetAngularVelocityDriveTwistAndSwing(true,false);
		Base_WheelRight->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0);
		Base_WheelRight->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 0);
		Base_WheelRight->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0);
		Base_WheelRight->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0);
		Base_WheelRight->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0);

		Base_CasterBack->ComponentName1.ComponentName = TEXT("Base");
		Base_CasterBack->ComponentName2.ComponentName = TEXT("CasterBack");
		Base_CasterBack->SetDisableCollision(true);
		Base_CasterBack->SetRelativeLocation(FVector(0,7.9,1));
		Base_CasterBack->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0);
		Base_CasterBack->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0);
		Base_CasterBack->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0);
	}
	else
	{
		UE_LOG(LogTemp, Error, TEXT("Turtlebot not initialized - can't setup constraints!"));
	}
}

void ATurtlebot3_Burger::SetAngularVelocityTargets(float velL, float velR)
{
	Base_WheelLeft->SetAngularVelocityTarget(FVector(velL, 0, 0));
	Base_WheelRight->SetAngularVelocityTarget(FVector(velR, 0, 0));
	Base_WheelLeft->SetAngularDriveParams(MaxForce, MaxForce, MaxForce);
	Base_WheelRight->SetAngularDriveParams(MaxForce, MaxForce, MaxForce);
}

void ATurtlebot3_Burger::SetTargetRotPerSFromVel(float velL, float velR)
{
	float WheelPerimeter = 6.6*3.1416;
	Base_WheelLeft->SetAngularVelocityTarget(FVector(velL/WheelPerimeter, 0, 0));
	Base_WheelRight->SetAngularVelocityTarget(FVector(velR/WheelPerimeter, 0, 0));
	Base_WheelLeft->SetAngularDriveParams(MaxForce, MaxForce, MaxForce);
	Base_WheelRight->SetAngularDriveParams(MaxForce, MaxForce, MaxForce);
}

// Called when the game starts or when spawned
void ATurtlebot3_Burger::BeginPlay()
{
	Super::BeginPlay();

	Base->SetMaterial(0, VehicleMaterial);
	LidarSensor->SetMaterial(0, VehicleMaterial);
	WheelLeft->SetMaterial(0, VehicleMaterial);
	WheelRight->SetMaterial(0, VehicleMaterial);
	CasterBack->SetMaterial(0, VehicleMaterial);

	SetAngularVelocityTargets(VelocityL,VelocityR);
}

void ATurtlebot3_Burger::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Super::EndPlay(EndPlayReason);
}

// Called every frame
void ATurtlebot3_Burger::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

