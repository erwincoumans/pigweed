# Copyright 2020 The Pigweed Authors
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

load(
    "//pw_build:pigweed.bzl",
    "pw_cc_library",
)

package(default_visibility = ["//visibility:public"])

licenses(["notice"])  # Apache License 2.0

pw_cc_library(
    name = "pw_i2c_baremetal_stm32f429",
    srcs = [
        "i2c_baremetal.cc",
        "demo_app_main.cc"
    ],
    hdrs = [
        "public/pw_i2c_baremetal_stm32f429/i2c_baremetal.h",
    ],
    includes = ["public"],
    deps = [
        "//pw_preprocessor",
        "//pw_span",
        "//pw_status",
        "//pw_bytes",
        "//pw_i2c",
    ],
)