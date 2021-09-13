// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"

#include <Runtime/Core/Public/Misc/FileHelper.h>

#include "FileIOUtils.generated.h"

/**
 *
 */
UCLASS()
class TURTLEBOT3_API UFileIOUtils : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()

    UFUNCTION(BlueprintCallable, Category = "File I/O")
    static bool LoadFileToString(const FString& InFileName, FString& OutFileContent);
};
