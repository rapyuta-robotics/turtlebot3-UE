// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.
#include "FileIOUtils.h"

// UE
#include "HAL/PlatformFileManager.h"
#include "Misc/Paths.h"

// Turtlebot3_UE
#include "Turtlebot3.h"

bool UFileIOUtils::LoadFileToString(const FString& InFileName, FString& OutFileContent)
{
    FString ContentDirectory = FPaths::ProjectContentDir();
    IPlatformFile& File = FPlatformFileManager::Get().GetPlatformFile();

    if (File.CreateDirectory(*ContentDirectory))
    {
        FString TargetFileName = ContentDirectory / InFileName;
        return FFileHelper::LoadFileToString(OutFileContent, *TargetFileName);
    }
    else
    {
        UE_LOG(LogTurtlebot3, Error, TEXT("Dir does not exist or failed being created: [%s]"), *ContentDirectory);
        return false;
    }
}
