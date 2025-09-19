# mvhal

implement custom solution for the Vehicle HAL of AOSP automotive.

Connection to SOME/IP using CommonAPI


### Building

It is mandatory to get boost, vsomeip, commonApi library in order to compil this software.


### Test

Vsomeip needs to have the following directory before launch a test server : 
mkdir /data/vendor/vsomeip


Then in a root adb shell, you can launch the vhal using the following command :
VSOMEIP_APPLICATION_NAME=client-sample \
VSOMEIP_CONFIGURATION=/vendor/etc/vsomeip.json \
COMMONAPI_CONFIG=/vendor/etc/commonapi.ini \
TREBLE_TESTING_OVERRIDE=true \
/vendor/bin/hw/vendor.marelli.hardware.automotive.vehicle@1.0-service &



For launching the test server on a the same target, 
VSOMEIP_APPLICATION_NAME=service-sample \
VSOMEIP_CONFIGURATION=/vendor/etc/vsomeip.json \
COMMONAPI_CONFIG=/vendor/etc/commonapi.ini \
test-someip-service &
