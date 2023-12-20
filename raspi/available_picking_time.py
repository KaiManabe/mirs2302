import sys
import json

if len(sys.argv) > 1:
    picking_place = sys.argv[1]
else:
    picking_place = ""
    
time_list = ["15:00-15:10",
             "15:10-15:20",
             "15:20-15:30",
             "16:00-16:10",
             "16:10-16:20",
             "16:20-16:30",
             ]

if picking_place == "D科棟":
    time_list.remove(time_list[0])
if picking_place == "S科棟":
    time_list.remove(time_list[1])
if picking_place == "C科棟":
    time_list.remove(time_list[2])


for t in time_list:
    print(t, end = ",")
# print(json.dumps(time_list))