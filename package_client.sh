#!/bin/sh

GENERATOR_COMMAND="../UnrealEngine/Engine/Build/BatchFiles/RunUAT.sh"

if [ "${1}" ]; then
	if [ "${1}" = "--default" ]; then
		ARCHIVE_DIR_REF=""
	else
		ARCHIVE_DIR_REF="-archivedirectory=$1"
	fi
else
	if [ -z "${ARCHIVE_DIR}" ]; then
		echo "Packaging the client requires an output archive directory!"
		exit 1
	fi
	ARCHIVE_DIR_REF="-archivedirectory=${ARCHIVE_DIR}"
fi

(exec "$GENERATOR_COMMAND" BuildCookRun -project="${PWD}/turtlebot3.uproject" -nop4 -build -cook -compressed -stage -platform=Linux -clientconfig=Development -pak -archive ${ARCHIVE_DIR_REF} -utf8output)

