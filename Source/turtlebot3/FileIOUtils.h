// Copyright (C) Rapyuta Robotics

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

    // UFUNCTION(BlueprintCallable, Category="File I/O")
    // static TArray<FString> LoadFileToStringArray(const FString& InFileName);

    // UFUNCTION(BlueprintCallable, Category = "File I/O")
    // static bool FileSaveString(const FString& InStringToSave, const FString& InFileName);
};
