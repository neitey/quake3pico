#!/bin/bash

if [ -z "$ANDROID_SDK_ROOT" ]; then
  ANDROID_SDK_ROOT=~/Android/Sdk
fi
if [ -z "$ANDROID_TOOLS_VERSION" ]; then
  ANDROID_TOOLS_VERSION=29.0.2
fi
if [ -z "$KEYSTORE" ]; then
  KEYSTORE=~/.android/release.keystore
fi

SCRIPTDIR=$(readlink -f $(dirname $0))

APP_NAME=$(cat $SCRIPTDIR/app/src/main/res/values/strings.xml | grep "app_name" | cut -d">" -f2 | cut -d"<" -f1 )
APP_VERSION=$(cat $SCRIPTDIR/app/src/main/AndroidManifest.xml | grep "versionName" | cut -d\" -f2 | cut -d\" -f1 )
APP_PACKAGE=$(cat $SCRIPTDIR/app/src/main/AndroidManifest.xml | grep "package" | cut -d\" -f2 | cut -d\" -f1 )

if [ "$1" == "release" ] || [ "$2" == "release" ] || [ "$3" == "release" ] || [ "$4" == "release" ]; then
  TARGET=release
  GRADLE_BUILD_TYPE=:app:assembleRelease
  APK_NAME=app-release-unsigned.apk
  APK_LOCATION="$SCRIPTDIR/app/build/outputs/apk/release"
else
  TARGET=release
  GRADLE_BUILD_TYPE=:app:assembleDebug
  APK_NAME=app-debug.apk
  APK_LOCATION="$SCRIPTDIR/app/build/outputs/apk/debug"
fi

if [ "$1" == "clean" ] || [ "$2" == "clean" ] || [ "$3" == "clean" ] || [ "$4" == "clean" ]; then
	rm -rf $SCRIPTDIR/../build
	rm -rf $SCRIPTDIR/build
	rm -rf $SCRIPTDIR/app/build
	rm -rf $SCRIPTDIR/app/src/main/jniLibs
fi

cd $SCRIPTDIR/app/src/main/pakQ3Q
rm -f $SCRIPTDIR/app/src/main/assets/pakQ3Q.pk3
zip -r $SCRIPTDIR/app/src/main/assets/pakQ3Q.pk3 .

echo "#define Q3QVERSION  \"$APP_VERSION"\" > $SCRIPTDIR/app/src/main/cpp/code/vr/vr_version.h

cd $SCRIPTDIR/..

make -j $(getconf _NPROCESSORS_ONLN) $TARGET
if [ $? -ne 0 ]; then
	echo "Failed to build ioq3"
	exit 1
fi

cd $SCRIPTDIR

./gradlew "$GRADLE_BUILD_TYPE"

if [ $? -ne 0 ]; then
	echo "Failed to build android project"
	exit 1
fi

UNSIGNED_APK="$APK_LOCATION/$APK_NAME"
APK_NAME="$APP_NAME-debug-$APP_VERSION.apk"
mv "$UNSIGNED_APK" "$APK_LOCATION/$APK_NAME"

if [ "$1" == "install" ] || [ "$2" == "install" ] || [ "$3" == "install" ] || [ "$4" == "install" ]; then
  adb install -r "$APK_LOCATION/$APK_NAME"
  if [ $? -ne 0 ]; then
    adb uninstall $APP_PACKAGE
    adb install "$APK_LOCATION/$APK_NAME"
    if [ $? -ne 0 ]; then
      echo "Failed to install apk."
      exit 1
    fi
  fi
fi

if [ "$1" == "start" ] || [ "$2" == "start" ] || [ "$3" == "start" ] || [ "$4" == "start" ]; then
  adb logcat -c
  adb shell am start -n $APP_PACKAGE/.MainActivity
  if [ $? -ne 0 ]; then
    echo "Failed to start application."
    exit 1
  fi
  adb logcat *:S Quake3:V SDL:V DEBUG:V
fi
