// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.


#include "TurtlebotBurger.h"

DEFINE_LOG_CATEGORY(LogTurtlebotBurger);

// Sets default values
ATurtlebotBurger::ATurtlebotBurger(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
	
    Init();
	MoveComponent = CreateDefaultSubobject<UDifferentialDriveComponent>(TEXT("MoveComponent"));
	UDifferentialDriveComponent* DifferentialDriveComponent = Cast<UDifferentialDriveComponent>(MoveComponent);
	DifferentialDriveComponent->SetWheels(Base_WheelLeft, Base_WheelRight);
}

void ATurtlebotBurger::Init()
{
	if (!IsInitialized)
	{
		// Meshes
        if (VehicleMaterial == nullptr)
        {
            static ConstructorHelpers::FObjectFinder<UMaterial> RobotMaterial(TEXT("Material'/Game/Blueprints/M_RobotMat.M_RobotMat'"));
            VehicleMaterial = RobotMaterial.Object;
            static ConstructorHelpers::FObjectFinder<UMaterial> CasterBallMaterial(TEXT("Material'/Game/Blueprints/M_CasterBallMat.M_CasterBallMat'"));
            BallMaterial = CasterBallMaterial.Object;
        }
        static ConstructorHelpers::FObjectFinder<UStaticMesh> BaseMesh(TEXT("'/Game/Models/Burger/BurgerBase.BurgerBase'"));
        static ConstructorHelpers::FObjectFinder<UStaticMesh> LidarMesh(TEXT("'/Game/Models/Common/Lds.Lds'"));
        static ConstructorHelpers::FObjectFinder<UStaticMesh> WheelLMesh(TEXT("'/Game/Models/Common/LeftWheel.LeftWheel'"));
        static ConstructorHelpers::FObjectFinder<UStaticMesh> WheelRMesh(TEXT("'/Game/Models/Common/RightWheel.RightWheel'"));
        static ConstructorHelpers::FObjectFinder<UStaticMesh> CasterMesh(TEXT("'/Game/Models/Common/BallCaster.BallCaster'"));
        
        Base = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Base"));
        LidarSensor = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("LidarSensor"));
        WheelLeft = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("WheelLeft"));
        WheelRight = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("WheelRight"));
        CasterBack = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("CasterBack"));

        RootComponent = Base;

        LidarSensor->SetupAttachment(Base);
        WheelLeft->SetupAttachment(Base);
        WheelRight->SetupAttachment(Base);
        CasterBack->SetupAttachment(Base);

        Base->SetStaticMesh(BaseMesh.Object);
        LidarSensor->SetStaticMesh(LidarMesh.Object);
        WheelLeft->SetStaticMesh(WheelLMesh.Object);
        WheelRight->SetStaticMesh(WheelRMesh.Object);
        CasterBack->SetStaticMesh(CasterMesh.Object);


        // Constraints
        Base_LidarSensor = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("Base_LidarSensor"));
        Base_LidarSensor->SetupAttachment(LidarSensor);
        
        Base_WheelLeft = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("Base_WheelLeft"));
        Base_WheelLeft->SetupAttachment(WheelLeft);
        
        Base_WheelRight = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("Base_WheelRight"));
        Base_WheelRight->SetupAttachment(WheelRight);
        
        Base_CasterBack = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("Base_CasterBack"));
        Base_CasterBack->SetupAttachment(CasterBack);

        IsInitialized = true;

        SetupConstraintsAndPhysics();
	}
}

void ATurtlebotBurger::SetupConstraintsAndPhysics()
{
	if (IsInitialized)
	{
		Base->SetMaterial(0, VehicleMaterial);
		LidarSensor->SetMaterial(0, VehicleMaterial);
		WheelLeft->SetMaterial(0, VehicleMaterial);
		WheelRight->SetMaterial(0, VehicleMaterial);
		CasterBack->SetMaterial(0, BallMaterial);

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

		Base_WheelLeft->ComponentName2.ComponentName = TEXT("Base");
		Base_WheelLeft->ComponentName1.ComponentName = TEXT("WheelLeft");
		Base_WheelLeft->SetDisableCollision(true);
		Base_WheelLeft->SetRelativeLocation(FVector(0,-8,2.3));
		Base_WheelLeft->SetRelativeRotation(FRotator(0,90,0));
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
		Base_WheelRight->SetRelativeLocation(FVector(0,8,2.3));
		Base_WheelRight->SetRelativeRotation(FRotator(0,90,0));
		Base_WheelRight->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
		Base_WheelRight->SetAngularDriveParams(MaxForce, MaxForce, MaxForce);
		Base_WheelRight->SetAngularVelocityDriveTwistAndSwing(true,false);
		Base_WheelRight->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0);
		Base_WheelRight->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 0);
		Base_WheelRight->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0);
		Base_WheelRight->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0);
		Base_WheelRight->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0);

		Base_CasterBack->ComponentName2.ComponentName = TEXT("Base");
		Base_CasterBack->ComponentName1.ComponentName = TEXT("CasterBack");
		Base_CasterBack->SetDisableCollision(true);
		Base_CasterBack->SetRelativeLocation(FVector(-8,0,-.5));
		Base_CasterBack->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0);
		Base_CasterBack->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0);
		Base_CasterBack->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0);
	}
	else
	{
		UE_LOG(LogTurtlebotBurger, Error, TEXT("Turtlebot not initialized - can't setup constraints!"));
	}
}

// Called when the game starts or when spawned
void ATurtlebotBurger::BeginPlay()
{
	Super::BeginPlay();

    Base->SetMaterial(0, VehicleMaterial);
	LidarSensor->SetMaterial(0, VehicleMaterial);
	WheelLeft->SetMaterial(0, VehicleMaterial);
	WheelRight->SetMaterial(0, VehicleMaterial);
	CasterBack->SetMaterial(0, VehicleMaterial);
    
    UDifferentialDriveComponent* DifferentialDriveComponent = Cast<UDifferentialDriveComponent>(MoveComponent);
    DifferentialDriveComponent->SetPerimeter();
}

void ATurtlebotBurger::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Super::EndPlay(EndPlayReason);
}

// Called every frame
void ATurtlebotBurger::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}


