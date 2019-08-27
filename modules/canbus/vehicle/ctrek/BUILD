package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ctrek_message_manager",
    srcs = [
        "ctrek_message_manager.cc",
    ],
    hdrs = [
        "ctrek_message_manager.h",
    ],
    deps = [
        "//modules/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/canbus/vehicle:message_manager_base",
        "//modules/canbus/vehicle/ctrek/protocol:canbus_ctrek_protocol",
    ],
)

cc_library(
    name = "ctrek_controller",
    srcs = [
        "ctrek_controller.cc",
    ],
    hdrs = [
        "ctrek_controller.h",
    ],
    deps = [
        ":ctrek_message_manager",
        "//modules/canbus/can_comm:can_sender",
        "//modules/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/canbus/vehicle:message_manager_base",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/ctrek/protocol:canbus_ctrek_protocol",
    ],
)

#cc_test(
#    name = "ctrek_controller_test",
#    size = "small",
#    srcs = [
#        "ctrek_controller_test.cc",
#    ],
#    data = ["//modules/canbus:canbus_testdata"],
#    deps = [
#        "//modules/canbus/vehicle/ctrek:ctrek_controller",
#        "@gtest//:main",
#    ],
#)

