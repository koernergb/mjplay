"""Generate the tiny deterministic ONNX model used by the UR5e demo policy.

Requires the optional `onnx` Python package. The generated model has no learned
robotics intelligence: it maps phase features to six smooth joint offsets and
exists to verify the complete browser policy execution path.
"""

from pathlib import Path

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper


OUTPUT = Path(__file__).parents[1] / "src/policies/ur5e-wave/policy.onnx"

# Input: [sin(phase), cos(phase), amplitude]
# Output: normalized offsets for the six UR5e actuators.
weights = np.array(
    [
        [0.20, -0.45, 0.55, 0.15, -0.30, 0.45],
        [0.35, 0.20, -0.15, 0.40, 0.25, -0.35],
        [0.00, 0.00, 0.00, 0.00, 0.00, 0.00],
    ],
    dtype=np.float32,
)
bias = np.zeros(6, dtype=np.float32)

graph = helper.make_graph(
    [
        helper.make_node("MatMul", ["observation", "weights"], ["linear"]),
        helper.make_node("Add", ["linear", "bias"], ["action"]),
    ],
    "mjplay_ur5e_wave",
    [helper.make_tensor_value_info("observation", TensorProto.FLOAT, [1, 3])],
    [helper.make_tensor_value_info("action", TensorProto.FLOAT, [1, 6])],
    [numpy_helper.from_array(weights, "weights"), numpy_helper.from_array(bias, "bias")],
)
model = helper.make_model(
    graph,
    producer_name="mjplay",
    opset_imports=[helper.make_opsetid("", 13)],
    ir_version=8,
)
onnx.checker.check_model(model)
OUTPUT.parent.mkdir(parents=True, exist_ok=True)
onnx.save(model, OUTPUT)
print(f"wrote {OUTPUT} ({OUTPUT.stat().st_size} bytes)")
