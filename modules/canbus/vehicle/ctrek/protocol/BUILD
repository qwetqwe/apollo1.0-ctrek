
package(default_visibility = ["//visibility:public"])

cc_library(
    name = "canbus_ctrek_protocol",
    srcs = [
        "brake_214.cc",
        "brake_321.cc",
        "gear_217.cc",
        "gear_324.cc",
        "steering_215.cc",
        "steering_322.cc",
        "throttle_216.cc",
        "throttle_323.cc",
        "turnsignal_218.cc",
	    "vehiclespd_4A0.cc",
        "light_218.cc",
        "acc_500.cc",
        "gyro_501.cc",
        "heading_502.cc",
        "height_503.cc",
        "lonlat_504.cc",
        "status_506.cc",
        "vel_505.cc"
    ],
    hdrs = [
        "brake_214.h",
        "brake_321.h",
        "gear_217.h",
        "gear_324.h",
        "steering_215.h",
        "steering_322.h",
        "throttle_323.h",
        "throttle_216.h",
        "turnsignal_218.h",
	    "vehiclespd_4A0.h",
        "light_218.h",
        "acc_500.h",
        "gyro_501.h",
        "heading_502.h",
        "height_503.h",
        "lonlat_504.h",
        "status_506.h",
        "vel_505.h"
    ],
    deps = [
       "//modules/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/canbus/vehicle:message_manager_base",
    ],
)
cc_test(
    name = "brake_214_test",
    size = "small",
    srcs = [
        "brake_214_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/ctrek/protocol:canbus_ctrek_protocol",
        "@gtest//:main",
    ],
)
cc_test(
    name = "brake_321_test",
    size = "small",
    srcs = [
        "brake_321_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/ctrek/protocol:canbus_ctrek_protocol",
        "@gtest//:main",
    ],
)
cc_test(
    name = "gear_217_test",
    size = "small",
    srcs = [
        "gear_217_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/ctrek/protocol:canbus_ctrek_protocol",
        "@gtest//:main",
    ],
)
cc_test(
    name = "gear_324_test",
    size = "small",
    srcs = [
        "gear_324_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/ctrek/protocol:canbus_ctrek_protocol",
        "@gtest//:main",
    ],
)
cc_test(
    name = "steering_215_test",
    size = "small",
    srcs = [
        "steering_215_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/ctrek/protocol:canbus_ctrek_protocol",
        "@gtest//:main",
    ],
)
cc_test(
    name = "steering_322_test",
    size = "small",
    srcs = [
        "steering_322_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/ctrek/protocol:canbus_ctrek_protocol",
        "@gtest//:main",
    ],
)
cc_test(
    name = "throttle_216_test",
    size = "small",
    srcs = [
        "throttle_216_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/ctrek/protocol:canbus_ctrek_protocol",
        "@gtest//:main",
    ],
)
cc_test(
    name = "vehiclespd_4A0_test",
    size = "small",
    srcs = [
        "vehiclespd_4A0_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/ctrek/protocol:canbus_ctrek_protocol",
        "@gtest//:main",
    ],
)
cc_test(
    name = "turnsignal_218_test",
    size = "small",
    srcs = [
        "turnsignal_218_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/ctrek/protocol:canbus_ctrek_protocol",
        "@gtest//:main",
    ],
)
