#!/usr/bin/env bash

DOXYGEN_DOWNLOAD_REMOTE=https://github.com/doxygen/doxygen/releases/download/Release_1_9_7/doxygen-1.9.7.linux.bin.tar.gz
DOXYGEN_TAR_UNPACK_LOCAL=./docs/doxygen/tools/doxygen-1.9.7
DOXYGEN_PATH="$DOXYGEN_TAR_UNPACK_LOCAL/bin/doxygen"

export PROJECT_VERSION=$(git describe --tags)

if ! $(which plantuml >&/dev/null); then
    echo -e "\e[32mplantuml not present, installing...\e[0m"
    echo -e "\e[32mIf prompted, please enter your password to continue the installation\e[0m"
    echo
    sudo apt install plantuml -y
fi

mkdir -p docs/generated-images
# copy images that are not PlantUML
echo -e "\e[32mCopying non-PlantUML images...\e[0m"
find src \( -name *.png -or -name *.jpg -or -name *.bmp \) -exec /bin/bash -c 'echo "Copying '{}' to docs/generated-images/$(basename "{}")"; cp "{}" "docs/generated-images/$(basename "{}")";' \;
# Generate PlantUML
echo -e "\e[32mGenerating PlantUML (*.puml) images...\e[0m"
PUML_FILES=$(find * -name *.puml)
java -jar -Djava.awt.headless=true -Djava.net.useSystemProxies=true /usr/share/plantuml/plantuml.jar -o $(realpath docs/generated-images) $PUML_FILES
# export twice to get mkdown to look nice
java -jar -Djava.awt.headless=true -Djava.net.useSystemProxies=true /usr/share/plantuml/plantuml.jar $PUML_FILES

if ! $(which curl >&/dev/null); then
    echo -e "\e[32mcurl not present, installing...\e[0m"
    echo -e "\e[32mIf prompted, please enter your password to continue the installation\e[0m"
    echo
    sudo apt update
    sudo apt install curl -y
fi

if ! test -e $DOXYGEN_PATH; then
    echo -e "\e[32mInstalling doxygen from '$DOXYGEN_DOWNLOAD_REMOTE' to '$DOXYGEN_TAR_UNPACK_LOCAL'...\e[0m"
    echo
    mkdir -p $DOXYGEN_TAR_UNPACK_LOCAL
    if ! $(curl -L $DOXYGEN_DOWNLOAD_REMOTE | tar -xz -C $DOXYGEN_TAR_UNPACK_LOCAL --strip-components 1); then
        echo -e "\e[31mDownload failed, exiting...\e[0m"
        exit 1
    fi
    echo -e "\e[32mDownload complete!\e[0m"
    echo
fi

echo -e "\e[32mTools present, generating documentation...\e[0m"
echo
$DOXYGEN_PATH
