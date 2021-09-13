// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "DifferentialDrive_2W.h"


// Sets default values
ADifferentialDrive_2W::ADifferentialDrive_2W()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	RootComponent = CreateDefaultSubobject<USceneComponent>("Root");
	
	// Meshes
	if (VehicleMaterial == nullptr)
	{
		static ConstructorHelpers::FObjectFinder<UMaterial> RobotMaterial(TEXT("Material'/Game/Blueprints/RobotMat.RobotMat'"));
		VehicleMaterial = RobotMaterial.Object;
	}
    static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeMesh(TEXT("'/Engine/BasicShapes/Cube.Cube'"));
    static ConstructorHelpers::FObjectFinder<UStaticMesh> CylinderMesh(TEXT("'/Engine/BasicShapes/Cylinder.Cylinder'"));
    static ConstructorHelpers::FObjectFinder<UStaticMesh> SphereMesh(TEXT("'/Engine/BasicShapes/Sphere.Sphere'"));

    Body = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Body"));
    WheelL = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("WheelL"));
    WheelR = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("WheelR"));
    BallCaster = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("BallCaster"));

    Body->SetupAttachment(RootComponent);
    WheelL->SetupAttachment(RootComponent);
    WheelR->SetupAttachment(RootComponent);
    BallCaster->SetupAttachment(RootComponent);

	Body->SetStaticMesh(CubeMesh.Object);
	WheelL->SetStaticMesh(CylinderMesh.Object);
	WheelR->SetStaticMesh(CylinderMesh.Object);
    BallCaster->SetStaticMesh(SphereMesh.Object);

	Body->SetMaterial(0, VehicleMaterial);
	WheelL->SetMaterial(0, VehicleMaterial);
	WheelR->SetMaterial(0, VehicleMaterial);
	BallCaster->SetMaterial(0, VehicleMaterial);

	Body->SetWorldScale3D(FVector(1.f, 1.2f, 0.7f));
	Body->SetRelativeLocation(FVector(0, 40, 40));

	WheelL->SetWorldScale3D(FVector(1.f, 1.f, 0.1f));
	WheelL->SetRelativeLocation(FVector( 60, 0, 50));
	WheelL->SetRelativeRotation(FRotator(90,0,0));

	WheelR->SetWorldScale3D(FVector(1.f, 1.f, 0.1f));
	WheelR->SetRelativeLocation(FVector(-60, 0, 50));
	WheelR->SetRelativeRotation(FRotator(90,0,0));

	BallCaster->SetWorldScale3D(FVector(.5f, .5f, .5f));
	BallCaster->SetRelativeLocation(FVector(0, 70, 25));

	Body->SetSimulatePhysics(true);
	WheelL->SetSimulatePhysics(true);
	WheelR->SetSimulatePhysics(true);
	BallCaster->SetSimulatePhysics(true);


	// Constraints
	Body_WheelL = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("Body_WheelL"));
    Body_WheelL->SetupAttachment(RootComponent);
	Body_WheelL->ComponentName2.ComponentName = TEXT("Body");
	Body_WheelL->ComponentName1.ComponentName = TEXT("WheelL");
	Body_WheelL->SetDisableCollision(true);
	Body_WheelL->SetRelativeLocation(FVector( 60, 0, 50));
	Body_WheelL->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
	Body_WheelL->SetAngularDriveParams(MaxForce, MaxForce, MaxForce);
	Body_WheelL->SetAngularVelocityDriveTwistAndSwing(true,false);
	Body_WheelL->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0);
	Body_WheelL->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 0);
	Body_WheelL->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0);
	Body_WheelL->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0);
	Body_WheelL->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0);
	
	Body_WheelR = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("Body_WheelR"));
    Body_WheelR->SetupAttachment(RootComponent);
	Body_WheelR->ComponentName2.ComponentName = TEXT("Body");
	Body_WheelR->ComponentName1.ComponentName = TEXT("WheelR");
	Body_WheelR->SetDisableCollision(true);
	Body_WheelR->SetRelativeLocation(FVector(-60, 0, 50));
	Body_WheelR->SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
	Body_WheelR->SetAngularDriveParams(MaxForce, MaxForce, MaxForce);
	Body_WheelR->SetAngularVelocityDriveTwistAndSwing(true,false);
	Body_WheelR->SetAngularSwing1Limit(EAngularConstraintMotion::ACM_Locked, 0);
	Body_WheelR->SetAngularSwing2Limit(EAngularConstraintMotion::ACM_Locked, 0);
	Body_WheelR->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0);
	Body_WheelR->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0);
	Body_WheelR->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0);
	
	Body_BallCaster = CreateDefaultSubobject<UPhysicsConstraintComponent>(TEXT("Body_BallCaster"));
    Body_BallCaster->SetupAttachment(RootComponent);
	Body_BallCaster->ComponentName1.ComponentName = TEXT("Body");
	Body_BallCaster->ComponentName2.ComponentName = TEXT("BallCaster");
	Body_BallCaster->SetDisableCollision(true);
	Body_BallCaster->SetRelativeLocation(FVector(0, 70, 25));
	Body_BallCaster->SetLinearXLimit(ELinearConstraintMotion::LCM_Locked, 0);
	Body_BallCaster->SetLinearYLimit(ELinearConstraintMotion::LCM_Locked, 0);
	Body_BallCaster->SetLinearZLimit(ELinearConstraintMotion::LCM_Locked, 0);
}

void ADifferentialDrive_2W::SetAngularVelocityTargets(float velL, float velR)
{
	Body_WheelL->SetAngularVelocityTarget(FVector(velL, 0, 0));
	Body_WheelR->SetAngularVelocityTarget(FVector(velR, 0, 0));
	Body_WheelL->SetAngularDriveParams(MaxForce, MaxForce, MaxForce);
	Body_WheelR->SetAngularDriveParams(MaxForce, MaxForce, MaxForce);
}

// Called when the game starts or when spawned
void ADifferentialDrive_2W::BeginPlay()
{
	Super::BeginPlay();

	Body->SetMaterial(0, VehicleMaterial);
	WheelL->SetMaterial(0, VehicleMaterial);
	WheelR->SetMaterial(0, VehicleMaterial);
	BallCaster->SetMaterial(0, VehicleMaterial);

	SetAngularVelocityTargets(VelocityL,VelocityR);
}

// Called every frame
void ADifferentialDrive_2W::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

