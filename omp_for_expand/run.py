from subprocess import Popen, PIPE, STDOUT
import re

court_detection = "./detect"
args = ["-i", "41.png", "-f", "white"]

court_detection_proc = Popen([court_detection]+args, stdout=PIPE)
result = court_detection_proc.communicate()

print("--------------")
result_list = re.split("Coord:|\n| \n", result[0].decode())
print(eval("["+re.subn(", | ", ",", result_list[5])[0]+"]"))
coord = "["+re.subn(", | ", ",", result_list[7])[0]+"]"

segment_proc = Popen(["python3", "inout_seg.py", "-i", "0.png", "-c", coord], stdout=PIPE)
result = segment_proc.communicate()
