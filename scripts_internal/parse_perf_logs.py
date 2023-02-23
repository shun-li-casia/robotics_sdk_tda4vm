#!/usr/bin/python3

#  Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#    Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
#    Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the
#    distribution.
#
#    Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import argparse

"""
Log files are stored as blow:

$BASE_DIR_PATH
├── perf_logs_ros1_bag
│   ├── app_estop.md
│   ├── app_objdet.md
│   ├── app_sde.md
│   ├── app_semseg.md
│   ├── app_visloc.md
│   ├── estop
│   │   ├── Log0.md
│   │   ├── Log10.md
│   │   ├── Log11.md
│   │   ├── Log12.md
│   │   ├── Log1.md
│   │   ├── Log2.md
│   │   ├── Log3.md
│   │   ├── Log4.md
│   │   ├── Log5.md
│   │   ├── Log6.md
│   │   ├── Log7.md
│   │   ├── Log8.md
│   │   └── Log9.md
│   ├── objdet
│   │   ├── Log0.md
│   │   ├── Log1.md
│   │   ├── Log2.md
│   │   └── Log3.md
...
├── perf_logs_ros2_bag
│   ├── app_estop.md
│   ├── app_objdet.md
│   ├── app_sde.md
│   ├── app_semseg.md
│   ├── app_visloc.md
│   ├── estop
│   │   ├── Log0.md
│   │   ├── Log10.md
│   │   ├── Log11.md
...
"""

import re
import os
import sys

metrics = [
    "FPS",
    "Inter_frame_interval",
    "Preprocessing",
    "DlInfer-node",
    "mpu1_0",
    "READ BW",
    "WRITE BW",
    "TOTAL BW",
    "c7x_1",
    "c6x_1",
    "c6x_2",
    "mcu2_0",
    "mcu2_1",
    "MSC0",
    "MSC1",
    "VISS",
    "NF",
    "LDC",
    "SDE",
    "DOF",
]

metrics_desc = {
    "FPS": "FPS",
    "Inter_frame_interval": "Total time (ms)",
    "Preprocessing": "Preproc time (ms)",
    "DlInfer-node": "Inference time (ms)",
    "mpu1_0": "A72 Load (%)",
    "mcu2_0": "MCU2_0 Load (%)",
    "mcu2_1": "MCU2_1 Load (%)",
    "c7x_1": "C71 Load (%)",
    "c6x_1": "C66_1 Load (%)",
    "c6x_2": "C66_2 Load (%)",
    "MSC0": "MSC_0 (%)",
    "MSC1": "MSC_1 (%)",
    "VISS": "VISS (%)",
    "NF": "NF (%)",
    "LDC": "LDC (%)",
    "SDE": "SDE (%)",
    "DOF": "DOF (%)",
    "READ BW": "DDR Read BW (MB/s)",
    "WRITE BW": "DDR Write BW (MB/s)",
    "TOTAL BW": "DDR Total BW (MB/s)",
}

app_desc = {
    "semseg": "ti_vision_cnn (semseg)",
    "objdet": "ti_vision_cnn (objdet)",
    "sde": "ti_sde",
    "sdepcl": "ti_sde (w/ pcl)",
    "estop": "ti_estop",
    "objdet_range": "ti_objdet_range",
    "visloc": "ti_vl",
}

apps_ordered = [
    "sde",
    "sdepcl",
    "semseg",
    "objdet",
    "estop",
    "objdet_range",
    "visloc",
]

def parse_log_file(test_suit, folder_path):

    # app stats file
    app_stats_path = os.path.join(
        folder_path,
        f"app_{test_suit}.md"
    )

    # log file to use (Log*.md)
    logs_dir_path = os.path.join(
        folder_path,
        f"{test_suit}"
    )

    max = 0
    for root, dirs, files in os.walk(logs_dir_path, topdown=False):
        for name in files:
            index = int(re.match("Log([0-9]*)\.md", name).groups(0)[0])
            if (index > max):
                max = index

    # init
    data = {}
    for key in metrics[:4]:
        data[key] = "NA"
    for key in metrics[4:]:
        data[key] = "0"

    # parse from app_stats_file
    with open(app_stats_path) as stats_file:
        for line in stats_file.readlines():
            for key in metrics[:4]:
                match = re.match(" *\| *" + key + " *\: *([0-9]*\.?[ *]?[0-9]*)", line)
                if (match):
                    data[key] = match.groups(0)[0].replace(" ", "")
                    break;

    # parse from log file
    with open(os.path.join(logs_dir_path, f"Log{max-1}.md")) as log_file:
        for line in log_file.readlines():
            for key in metrics[4:]:
                match = re.match(" *" + key + " *\| *([0-9]*\.?[ *]?[0-9]*)", line)
                if (match):
                    data[key] = match.groups(0)[0].replace(" ", "")
                    break;

    # FPS
    # if not data["Inter_frame_interval"]=='0':
    fps = 1000./float(data["Inter_frame_interval"])
    data["FPS"] = f"{fps:.2f}"

    # generate the table row
    table_row =  "%s" % app_desc[test_suit]
    try:
        for i in metrics:
            table_row += "| %s" % data[i]
        table_row += "\n"
        return table_row
    except KeyError:
        print("[PERF] ERROR: Failed to parse performance information from the log")
        exit(2)
        return None

def form_table(test_suits, folder_path):
    table_rows = ""
    table_header = "Demo "
    for metric in metrics:
        table_header += f"| {metrics_desc[metric]}"
    table_header += "\n"
    table_rows += table_header

    table_header = "----"
    for metric in metrics:
        table_header += "|-----"
    table_header += "\n"
    table_rows += table_header

    for test_suit in test_suits:
        print(f"    Processing {test_suit}...")
        table_row = parse_log_file(test_suit, folder_path)
        table_rows += table_row

    return table_rows

def save_to_file(table_rows, out_file):
    if os.path.exists(out_file):
        os.system(f"rm {out_file}")
    with open(out_file, 'a') as f:
        f.write(table_rows)
    print(f"    {out_file} generated.")

def run_main(dirName, folder_name):
    # list all demos under "folder_name"
    folder_path = os.path.join(dirName, folder_name)
    test_suits_ = []
    for f in os.listdir(folder_path):
        if os.path.isdir(os.path.join(folder_path,f)):
            test_suits_.append(f)
    # order test_suits as much following apps_ordered
    test_suits = [x for y in apps_ordered for x in test_suits_ if y == x]

    # main routine
    table_rows = form_table(test_suits, folder_path)
    out_file = os.path.join(dirName, folder_name+".md")
    save_to_file(table_rows, out_file)

def ParseCmdLineArgs():
    # Define the parser rules
    parser = argparse.ArgumentParser(description='PERFORMANCE LOG PARSER', add_help=False)

    required    = parser.add_argument_group('required arguments')

    required.add_argument('-d', '--data_dir',
                          help='Root directory for the log files.',
                          required=True)

    # Invoke the parser
    args = parser.parse_args()

    return args

# Main routine
if __name__ == "__main__":
    # Parse command line arguments
    args      = ParseCmdLineArgs()
    dirName   = args.data_dir

    # run through following folders under 'dirName'
    folder_names = [
        "perf_logs_ros1_bag",
        "perf_logs_ros1_mono",
        "perf_logs_ros1_zed",
        "perf_logs_ros2_bag",
        "perf_logs_ros2_mono",
        "perf_logs_ros2_zed",
    ]

    for folder_name in folder_names:
        print(f"Processing {folder_name}...")
        run_main(dirName, folder_name)
