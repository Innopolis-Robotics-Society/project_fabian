import onnx
from collections import Counter
m = onnx.load("//Users/anashamrouni/Documents/InnopolisUniversity/Bachelor/PMLDL/project/project_fabian/f_gesture_recognition/models/stgcn_ntu60.onnx")
ops = [n.op_type for n in m.graph.node]
print(Counter(ops))
