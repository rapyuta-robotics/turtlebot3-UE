#!/bin/sh

PROJECT_PATH="/home/yu/UnrealProject"
PROJECT_NAME="MyProject"

EDITOR_COMMAND="./ue"

(exec "$EDITOR_COMMAND" "$PROJECT_PATH/$PROJECT_NAME/$PROJECT_NAME.uproject")
