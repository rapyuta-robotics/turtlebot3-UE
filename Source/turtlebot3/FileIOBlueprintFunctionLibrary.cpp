// Copyright (C) Rapyuta Robotics


#include "FileIOBlueprintFunctionLibrary.h"
#include <Runtime/Core/Public/Misc/Paths.h>
#include <Runtime/Core/Public/HAL/PlatformFilemanager.h>


FString UFileIOBlueprintFunctionLibrary::LoadFileToString(FString Filename)
{
    FString Directory = FPaths::ProjectContentDir();
    FString Result;
    IPlatformFile& File = FPlatformFileManager::Get().GetPlatformFile();

    if (File.CreateDirectory(*Directory))
    {
        FString TargetFile = Directory + "/" + Filename;
        FFileHelper::LoadFileToString(Result, *TargetFile);
    }

    return Result;
}

// TArray<FString> UFileIOBlueprintFunctionLibrary::LoadFileToStringArray(FString Filename)
// {
//     FString Directory = FPaths::ProjectContentDir();
//     FString Result;
//     IPlatformFile& File = FPlatformFileManager::Get().GetPlatformFile();

//     if (File.CreateDirectory(*Directory))
//     {
//         FString TargetFile = Directory + "/" + Filename;
//         FFileHelper::LoadFileToStringArray(Result, *TargetFile);
//     }

//     return Result;
// }

