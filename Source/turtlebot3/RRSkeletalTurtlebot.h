// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.
#pragma once

// UE
#include "CoreMinimal.h"

// rclUE
#include "logUtilities.h"

// RapyutaRobotImporter
#include "Robot/RRDifferentialDriveRobot.h"

#include "RRSkeletalTurtlebot.generated.h"

class URRTurtlebotROS2Interface;

UCLASS()
class TURTLEBOT3_API ARRSkeletalTurtlebot : public ARRDifferentialDriveRobot
{
    GENERATED_BODY()

public:
    ARRSkeletalTurtlebot();
    ARRSkeletalTurtlebot(const FObjectInitializer& ObjectInitializer);

    UPROPERTY()
    TObjectPtr<URRTurtlebotROS2Interface> TurtlebotROS2Interface = nullptr;

protected:
    /**
     * @brief Called by ctor only to setup default components, configurable by child BPs
     */
    void SetupDefaultTurtlebot();
    virtual void PreInitializeComponents() override;
    virtual void InitROSBasedProperties();
};
