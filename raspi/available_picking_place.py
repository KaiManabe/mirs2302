import sys
import json

if len(sys.argv) > 1:
    picking_time = sys.argv[1]
else:
    picking_time = ""
    
place_list = ["D科棟",
             "S科棟",
             "C科棟",
             ]

if picking_time == "15:00-15:10":
    place_list.remove(place_list[0])
if picking_time == "15:10-15:20":
    place_list.remove(place_list[1])
if picking_time == "15:20-15:30":
    place_list.remove(place_list[2])


for p in place_list:
    print(p, end = ",")
# print(json.dumps(place_list))