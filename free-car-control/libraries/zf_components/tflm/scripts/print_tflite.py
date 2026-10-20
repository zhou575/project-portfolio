import flatbuffers
import tensorflow.lite as tflite
from tensorflow.lite.python import schema_py_generated as schema_fb
import argparse

# 枚举类型映射（适用于旧版本）
tensor_type_map = {
    schema_fb.TensorType.FLOAT32: "FLOAT32",
    schema_fb.TensorType.FLOAT16: "FLOAT16",
    schema_fb.TensorType.INT32: "INT32",
    schema_fb.TensorType.UINT8: "UINT8",
    schema_fb.TensorType.INT64: "INT64",
    schema_fb.TensorType.STRING: "STRING",
    schema_fb.TensorType.BOOL: "BOOL",
    schema_fb.TensorType.INT16: "INT16",
    schema_fb.TensorType.COMPLEX64: "COMPLEX64",
    schema_fb.TensorType.INT8: "INT8",
    schema_fb.TensorType.FLOAT64: "FLOAT64",
    schema_fb.TensorType.COMPLEX128: "COMPLEX128",
    schema_fb.TensorType.UINT64: "UINT64",
    schema_fb.TensorType.RESOURCE: "RESOURCE",
    schema_fb.TensorType.VARIANT: "VARIANT",
    schema_fb.TensorType.UINT32: "UINT32",
    schema_fb.TensorType.UINT16: "UINT16",
    schema_fb.TensorType.INT4: "INT4",
    schema_fb.TensorType.BFLOAT16: "BFLOAT16",
}

# 枚举操作符映射（适用于旧版本）
buildin_op_type_map = {
    schema_fb.BuiltinOperator.ADD: "ADD",
    schema_fb.BuiltinOperator.AVERAGE_POOL_2D: "AVERAGE_POOL_2D",
    schema_fb.BuiltinOperator.CONCATENATION: "CONCATENATION",
    schema_fb.BuiltinOperator.CONV_2D: "CONV_2D",
    schema_fb.BuiltinOperator.DEPTHWISE_CONV_2D: "DEPTHWISE_CONV_2D",
    schema_fb.BuiltinOperator.DEPTH_TO_SPACE: "DEPTH_TO_SPACE",
    schema_fb.BuiltinOperator.DEQUANTIZE: "DEQUANTIZE",
    schema_fb.BuiltinOperator.EMBEDDING_LOOKUP: "EMBEDDING_LOOKUP",
    schema_fb.BuiltinOperator.FLOOR: "FLOOR",
    schema_fb.BuiltinOperator.FULLY_CONNECTED: "FULLY_CONNECTED",
    schema_fb.BuiltinOperator.HASHTABLE_LOOKUP: "HASHTABLE_LOOKUP",
    schema_fb.BuiltinOperator.L2_NORMALIZATION: "L2_NORMALIZATION",
    schema_fb.BuiltinOperator.L2_POOL_2D: "L2_POOL_2D",
    schema_fb.BuiltinOperator.LOCAL_RESPONSE_NORMALIZATION: "LOCAL_RESPONSE_NORMALIZATION",
    schema_fb.BuiltinOperator.LOGISTIC: "LOGISTIC",
    schema_fb.BuiltinOperator.LSH_PROJECTION: "LSH_PROJECTION",
    schema_fb.BuiltinOperator.LSTM: "LSTM",
    schema_fb.BuiltinOperator.MAX_POOL_2D: "MAX_POOL_2D",
    schema_fb.BuiltinOperator.MUL: "MUL",
    schema_fb.BuiltinOperator.RELU: "RELU",
    schema_fb.BuiltinOperator.RELU_N1_TO_1: "RELU_N1_TO_1",
    schema_fb.BuiltinOperator.RELU6: "RELU6",
    schema_fb.BuiltinOperator.RESHAPE: "RESHAPE",
    schema_fb.BuiltinOperator.RESIZE_BILINEAR: "RESIZE_BILINEAR",
    schema_fb.BuiltinOperator.RNN: "RNN",
    schema_fb.BuiltinOperator.SOFTMAX: "SOFTMAX",
    schema_fb.BuiltinOperator.SPACE_TO_DEPTH: "SPACE_TO_DEPTH",
    schema_fb.BuiltinOperator.SVDF: "SVDF",
    schema_fb.BuiltinOperator.TANH: "TANH",
    schema_fb.BuiltinOperator.CONCAT_EMBEDDINGS: "CONCAT_EMBEDDINGS",
    schema_fb.BuiltinOperator.SKIP_GRAM: "SKIP_GRAM",
    schema_fb.BuiltinOperator.CALL: "CALL",
    schema_fb.BuiltinOperator.CUSTOM: "CUSTOM",
    schema_fb.BuiltinOperator.EMBEDDING_LOOKUP_SPARSE: "EMBEDDING_LOOKUP_SPARSE",
    schema_fb.BuiltinOperator.PAD: "PAD",
    schema_fb.BuiltinOperator.UNIDIRECTIONAL_SEQUENCE_RNN: "UNIDIRECTIONAL_SEQUENCE_RNN",
    schema_fb.BuiltinOperator.GATHER: "GATHER",
    schema_fb.BuiltinOperator.BATCH_TO_SPACE_ND: "BATCH_TO_SPACE_ND",
    schema_fb.BuiltinOperator.SPACE_TO_BATCH_ND: "SPACE_TO_BATCH_ND",
    schema_fb.BuiltinOperator.TRANSPOSE: "TRANSPOSE",
    schema_fb.BuiltinOperator.MEAN: "MEAN",
    schema_fb.BuiltinOperator.SUB: "SUB",
    schema_fb.BuiltinOperator.DIV: "DIV",
    schema_fb.BuiltinOperator.SQUEEZE: "SQUEEZE",
    schema_fb.BuiltinOperator.UNIDIRECTIONAL_SEQUENCE_LSTM: "UNIDIRECTIONAL_SEQUENCE_LSTM",
    schema_fb.BuiltinOperator.STRIDED_SLICE: "STRIDED_SLICE",
    schema_fb.BuiltinOperator.BIDIRECTIONAL_SEQUENCE_RNN: "BIDIRECTIONAL_SEQUENCE_RNN",
    schema_fb.BuiltinOperator.EXP: "EXP",
    schema_fb.BuiltinOperator.TOPK_V2: "TOPK_V2",
    schema_fb.BuiltinOperator.SPLIT: "SPLIT",
    schema_fb.BuiltinOperator.LOG_SOFTMAX: "LOG_SOFTMAX",
    schema_fb.BuiltinOperator.DELEGATE: "DELEGATE",
    schema_fb.BuiltinOperator.BIDIRECTIONAL_SEQUENCE_LSTM: "BIDIRECTIONAL_SEQUENCE_LSTM",
    schema_fb.BuiltinOperator.CAST: "CAST",
    schema_fb.BuiltinOperator.PRELU: "PRELU",
    schema_fb.BuiltinOperator.MAXIMUM: "MAXIMUM",
    schema_fb.BuiltinOperator.ARG_MAX: "ARG_MAX",
    schema_fb.BuiltinOperator.MINIMUM: "MINIMUM",
    schema_fb.BuiltinOperator.LESS: "LESS",
    schema_fb.BuiltinOperator.NEG: "NEG",
    schema_fb.BuiltinOperator.PADV2: "PADV2",
    schema_fb.BuiltinOperator.GREATER: "GREATER",
    schema_fb.BuiltinOperator.GREATER_EQUAL: "GREATER_EQUAL",
    schema_fb.BuiltinOperator.LESS_EQUAL: "LESS_EQUAL",
    schema_fb.BuiltinOperator.SELECT: "SELECT",
    schema_fb.BuiltinOperator.SLICE: "SLICE",
    schema_fb.BuiltinOperator.SIN: "SIN",
    schema_fb.BuiltinOperator.TRANSPOSE_CONV: "TRANSPOSE_CONV",
    schema_fb.BuiltinOperator.SPARSE_TO_DENSE: "SPARSE_TO_DENSE",
    schema_fb.BuiltinOperator.TILE: "TILE",
    schema_fb.BuiltinOperator.EXPAND_DIMS: "EXPAND_DIMS",
    schema_fb.BuiltinOperator.EQUAL: "EQUAL",
    schema_fb.BuiltinOperator.NOT_EQUAL: "NOT_EQUAL",
    schema_fb.BuiltinOperator.LOG: "LOG",
    schema_fb.BuiltinOperator.SUM: "SUM",
    schema_fb.BuiltinOperator.SQRT: "SQRT",
    schema_fb.BuiltinOperator.RSQRT: "RSQRT",
    schema_fb.BuiltinOperator.SHAPE: "SHAPE",
    schema_fb.BuiltinOperator.POW: "POW",
    schema_fb.BuiltinOperator.ARG_MIN: "ARG_MIN",
    schema_fb.BuiltinOperator.FAKE_QUANT: "FAKE_QUANT",
    schema_fb.BuiltinOperator.REDUCE_PROD: "REDUCE_PROD",
    schema_fb.BuiltinOperator.REDUCE_MAX: "REDUCE_MAX",
    schema_fb.BuiltinOperator.PACK: "PACK",
    schema_fb.BuiltinOperator.LOGICAL_OR: "LOGICAL_OR",
    schema_fb.BuiltinOperator.ONE_HOT: "ONE_HOT",
    schema_fb.BuiltinOperator.LOGICAL_AND: "LOGICAL_AND",
    schema_fb.BuiltinOperator.LOGICAL_NOT: "LOGICAL_NOT",
    schema_fb.BuiltinOperator.UNPACK: "UNPACK",
    schema_fb.BuiltinOperator.REDUCE_MIN: "REDUCE_MIN",
    schema_fb.BuiltinOperator.FLOOR_DIV: "FLOOR_DIV",
    schema_fb.BuiltinOperator.REDUCE_ANY: "REDUCE_ANY",
    schema_fb.BuiltinOperator.SQUARE: "SQUARE",
    schema_fb.BuiltinOperator.ZEROS_LIKE: "ZEROS_LIKE",
    schema_fb.BuiltinOperator.FILL: "FILL",
    schema_fb.BuiltinOperator.FLOOR_MOD: "FLOOR_MOD",
    schema_fb.BuiltinOperator.RANGE: "RANGE",
    schema_fb.BuiltinOperator.RESIZE_NEAREST_NEIGHBOR: "RESIZE_NEAREST_NEIGHBOR",
    schema_fb.BuiltinOperator.LEAKY_RELU: "LEAKY_RELU",
    schema_fb.BuiltinOperator.SQUARED_DIFFERENCE: "SQUARED_DIFFERENCE",
    schema_fb.BuiltinOperator.MIRROR_PAD: "MIRROR_PAD",
    schema_fb.BuiltinOperator.ABS: "ABS",
    schema_fb.BuiltinOperator.SPLIT_V: "SPLIT_V",
    schema_fb.BuiltinOperator.UNIQUE: "UNIQUE",
    schema_fb.BuiltinOperator.CEIL: "CEIL",
    schema_fb.BuiltinOperator.REVERSE_V2: "REVERSE_V2",
    schema_fb.BuiltinOperator.ADD_N: "ADD_N",
    schema_fb.BuiltinOperator.GATHER_ND: "GATHER_ND",
    schema_fb.BuiltinOperator.COS: "COS",
    schema_fb.BuiltinOperator.WHERE: "WHERE",
    schema_fb.BuiltinOperator.RANK: "RANK",
    schema_fb.BuiltinOperator.ELU: "ELU",
    schema_fb.BuiltinOperator.REVERSE_SEQUENCE: "REVERSE_SEQUENCE",
    schema_fb.BuiltinOperator.MATRIX_DIAG: "MATRIX_DIAG",
    schema_fb.BuiltinOperator.QUANTIZE: "QUANTIZE",
    schema_fb.BuiltinOperator.MATRIX_SET_DIAG: "MATRIX_SET_DIAG",
    schema_fb.BuiltinOperator.ROUND: "ROUND",
    schema_fb.BuiltinOperator.HARD_SWISH: "HARD_SWISH",
    schema_fb.BuiltinOperator.IF: "IF",
    schema_fb.BuiltinOperator.WHILE: "WHILE",
    schema_fb.BuiltinOperator.NON_MAX_SUPPRESSION_V4: "NON_MAX_SUPPRESSION_V4",
    schema_fb.BuiltinOperator.NON_MAX_SUPPRESSION_V5: "NON_MAX_SUPPRESSION_V5",
    schema_fb.BuiltinOperator.SCATTER_ND: "SCATTER_ND",
    schema_fb.BuiltinOperator.SELECT_V2: "SELECT_V2",
    schema_fb.BuiltinOperator.DENSIFY: "DENSIFY",
    schema_fb.BuiltinOperator.SEGMENT_SUM: "SEGMENT_SUM",
    schema_fb.BuiltinOperator.BATCH_MATMUL: "BATCH_MATMUL",
    schema_fb.BuiltinOperator.PLACEHOLDER_FOR_GREATER_OP_CODES: "PLACEHOLDER_FOR_GREATER_OP_CODES",
    schema_fb.BuiltinOperator.CUMSUM: "CUMSUM",
    schema_fb.BuiltinOperator.CALL_ONCE: "CALL_ONCE",
    schema_fb.BuiltinOperator.BROADCAST_TO: "BROADCAST_TO",
    schema_fb.BuiltinOperator.RFFT2D: "RFFT2D",
    schema_fb.BuiltinOperator.CONV_3D: "CONV_3D",
    schema_fb.BuiltinOperator.IMAG: "IMAG",
    schema_fb.BuiltinOperator.REAL: "REAL",
    schema_fb.BuiltinOperator.COMPLEX_ABS: "COMPLEX_ABS",
    schema_fb.BuiltinOperator.HASHTABLE: "HASHTABLE",
    schema_fb.BuiltinOperator.HASHTABLE_FIND: "HASHTABLE_FIND",
    schema_fb.BuiltinOperator.HASHTABLE_IMPORT: "HASHTABLE_IMPORT",
    schema_fb.BuiltinOperator.HASHTABLE_SIZE: "HASHTABLE_SIZE",
    schema_fb.BuiltinOperator.REDUCE_ALL: "REDUCE_ALL",
    schema_fb.BuiltinOperator.CONV_3D_TRANSPOSE: "CONV_3D_TRANSPOSE",
    schema_fb.BuiltinOperator.VAR_HANDLE: "VAR_HANDLE",
    schema_fb.BuiltinOperator.READ_VARIABLE: "READ_VARIABLE",
    schema_fb.BuiltinOperator.ASSIGN_VARIABLE: "ASSIGN_VARIABLE",
    schema_fb.BuiltinOperator.BROADCAST_ARGS: "BROADCAST_ARGS",
    schema_fb.BuiltinOperator.RANDOM_STANDARD_NORMAL: "RANDOM_STANDARD_NORMAL",
    schema_fb.BuiltinOperator.BUCKETIZE: "BUCKETIZE",
    schema_fb.BuiltinOperator.RANDOM_UNIFORM: "RANDOM_UNIFORM",
    schema_fb.BuiltinOperator.MULTINOMIAL: "MULTINOMIAL",
    schema_fb.BuiltinOperator.GELU: "GELU",
    schema_fb.BuiltinOperator.DYNAMIC_UPDATE_SLICE: "DYNAMIC_UPDATE_SLICE",
    schema_fb.BuiltinOperator.RELU_0_TO_1: "RELU_0_TO_1",
    schema_fb.BuiltinOperator.UNSORTED_SEGMENT_PROD: "UNSORTED_SEGMENT_PROD",
    schema_fb.BuiltinOperator.UNSORTED_SEGMENT_MAX: "UNSORTED_SEGMENT_MAX",
    schema_fb.BuiltinOperator.UNSORTED_SEGMENT_SUM: "UNSORTED_SEGMENT_SUM",
    schema_fb.BuiltinOperator.ATAN2: "ATAN2",
    schema_fb.BuiltinOperator.UNSORTED_SEGMENT_MIN: "UNSORTED_SEGMENT_MIN",
    schema_fb.BuiltinOperator.SIGN: "SIGN",
    schema_fb.BuiltinOperator.BITCAST: "BITCAST",
    schema_fb.BuiltinOperator.BITWISE_XOR: "BITWISE_XOR",
    schema_fb.BuiltinOperator.RIGHT_SHIFT: "RIGHT_SHIFT",
    schema_fb.BuiltinOperator.STABLEHLO_LOGISTIC: "STABLEHLO_LOGISTIC",
    schema_fb.BuiltinOperator.STABLEHLO_ADD: "STABLEHLO_ADD",
    schema_fb.BuiltinOperator.STABLEHLO_DIVIDE: "STABLEHLO_DIVIDE",
    schema_fb.BuiltinOperator.STABLEHLO_MULTIPLY: "STABLEHLO_MULTIPLY",
    schema_fb.BuiltinOperator.STABLEHLO_MAXIMUM: "STABLEHLO_MAXIMUM",
    schema_fb.BuiltinOperator.STABLEHLO_RESHAPE: "STABLEHLO_RESHAPE",
    schema_fb.BuiltinOperator.STABLEHLO_CLAMP: "STABLEHLO_CLAMP",
    schema_fb.BuiltinOperator.STABLEHLO_CONCATENATE: "STABLEHLO_CONCATENATE",
    schema_fb.BuiltinOperator.STABLEHLO_BROADCAST_IN_DIM: "STABLEHLO_BROADCAST_IN_DIM",
    schema_fb.BuiltinOperator.STABLEHLO_CONVOLUTION: "STABLEHLO_CONVOLUTION",
    schema_fb.BuiltinOperator.STABLEHLO_SLICE: "STABLEHLO_SLICE",
    schema_fb.BuiltinOperator.STABLEHLO_CUSTOM_CALL: "STABLEHLO_CUSTOM_CALL",
    schema_fb.BuiltinOperator.STABLEHLO_REDUCE: "STABLEHLO_REDUCE",
    schema_fb.BuiltinOperator.STABLEHLO_ABS: "STABLEHLO_ABS",
    schema_fb.BuiltinOperator.STABLEHLO_AND: "STABLEHLO_AND",
    schema_fb.BuiltinOperator.STABLEHLO_COSINE: "STABLEHLO_COSINE",
    schema_fb.BuiltinOperator.STABLEHLO_EXPONENTIAL: "STABLEHLO_EXPONENTIAL",
    schema_fb.BuiltinOperator.STABLEHLO_FLOOR: "STABLEHLO_FLOOR",
    schema_fb.BuiltinOperator.STABLEHLO_LOG: "STABLEHLO_LOG",
    schema_fb.BuiltinOperator.STABLEHLO_MINIMUM: "STABLEHLO_MINIMUM",
    schema_fb.BuiltinOperator.STABLEHLO_NEGATE: "STABLEHLO_NEGATE",
    schema_fb.BuiltinOperator.STABLEHLO_OR: "STABLEHLO_OR",
    schema_fb.BuiltinOperator.STABLEHLO_POWER: "STABLEHLO_POWER",
    schema_fb.BuiltinOperator.STABLEHLO_REMAINDER: "STABLEHLO_REMAINDER",
    schema_fb.BuiltinOperator.STABLEHLO_RSQRT: "STABLEHLO_RSQRT",
    schema_fb.BuiltinOperator.STABLEHLO_SELECT: "STABLEHLO_SELECT",
    schema_fb.BuiltinOperator.STABLEHLO_SUBTRACT: "STABLEHLO_SUBTRACT",
    schema_fb.BuiltinOperator.STABLEHLO_TANH: "STABLEHLO_TANH",
    schema_fb.BuiltinOperator.STABLEHLO_SCATTER: "STABLEHLO_SCATTER",
    schema_fb.BuiltinOperator.STABLEHLO_COMPARE: "STABLEHLO_COMPARE",
    schema_fb.BuiltinOperator.STABLEHLO_CONVERT: "STABLEHLO_CONVERT",
    schema_fb.BuiltinOperator.STABLEHLO_DYNAMIC_SLICE: "STABLEHLO_DYNAMIC_SLICE",
    schema_fb.BuiltinOperator.STABLEHLO_DYNAMIC_UPDATE_SLICE: "STABLEHLO_DYNAMIC_UPDATE_SLICE",
    schema_fb.BuiltinOperator.STABLEHLO_PAD: "STABLEHLO_PAD",
    schema_fb.BuiltinOperator.STABLEHLO_IOTA: "STABLEHLO_IOTA",
    schema_fb.BuiltinOperator.STABLEHLO_DOT_GENERAL: "STABLEHLO_DOT_GENERAL",
    schema_fb.BuiltinOperator.STABLEHLO_REDUCE_WINDOW: "STABLEHLO_REDUCE_WINDOW",
    schema_fb.BuiltinOperator.STABLEHLO_SORT: "STABLEHLO_SORT",
    schema_fb.BuiltinOperator.STABLEHLO_WHILE: "STABLEHLO_WHILE",
    schema_fb.BuiltinOperator.STABLEHLO_GATHER: "STABLEHLO_GATHER",
    schema_fb.BuiltinOperator.STABLEHLO_TRANSPOSE: "STABLEHLO_TRANSPOSE",
    schema_fb.BuiltinOperator.DILATE: "DILATE",
    schema_fb.BuiltinOperator.STABLEHLO_RNG_BIT_GENERATOR: "STABLEHLO_RNG_BIT_GENERATOR",
    schema_fb.BuiltinOperator.REDUCE_WINDOW: "REDUCE_WINDOW",
    schema_fb.BuiltinOperator.STABLEHLO_COMPOSITE: "STABLEHLO_COMPOSITE",
    schema_fb.BuiltinOperator.STABLEHLO_SHIFT_LEFT: "STABLEHLO_SHIFT_LEFT",
    schema_fb.BuiltinOperator.STABLEHLO_CBRT: "STABLEHLO_CBRT",
    schema_fb.BuiltinOperator.STABLEHLO_CASE: "STABLEHLO_CASE",
}

def get_tensor_type_name(tensor):
    try:
        # 新版本 API
        tensor_type = schema_fb.TensorType.Name(tensor.type)
    except AttributeError:
        # 旧版本 API
        tensor_type = tensor_type_map.get(tensor.type, f"未知类型({tensor.type})")
    return tensor_type

def get_buildin_op_type_name(builtin_code):
    try:
        # 新版本 API
        op_name=schema_fb.BuiltinOperator.Name(builtin_code)
    except AttributeError:
        # 旧版本 API
        op_name = buildin_op_type_map.get(builtin_code, f"未知类型({builtin_code})")
    return op_name
    
def print_model_structure(model_path):
    # 读取模型文件
    with open(model_path, 'rb') as f:
        model_data = f.read()
    
    # 解析FlatBuffer
    model_obj = schema_fb.Model.GetRootAsModel(model_data, 0)
    model = schema_fb.ModelT.InitFromObj(model_obj)
    
    # 获取子图
    subgraph = model.subgraphs[0]
    
    # 处理子图名称（兼容bytes和str类型）
    subgraph_name = subgraph.name
    if isinstance(subgraph_name, bytes):
        subgraph_name = subgraph_name.decode('utf-8')
        
    print("\n=== 子图详情 ===")
    print(f"子图名称: {subgraph_name}")
    print(f"输入数量: {len(subgraph.inputs)}")  # 修改点：使用len(subgraph.inputs)
    print(f"输出数量: {len(subgraph.outputs)}")  # 修改点：使用len(subgraph.outputs)
    print(f"张量数量: {len(subgraph.tensors)}")  # 修改点：使用len(subgraph.tensors)
    print(f"操作符数量: {len(subgraph.operators)}")  # 修改点：使用len(subgraph.operators)
    print("-" * 80)
    
    # 打印输入张量
    print("=== 输入张量 ===")
    for input_idx in subgraph.inputs:  # 修改点：直接遍历subgraph.inputs
        tensor = subgraph.tensors[input_idx]
        tensor_type = get_tensor_type_name(tensor)
        shape = tensor.shape
        
        print(f"输入:")
        print(f"  名称: {tensor.name}")
        print(f"  索引: {input_idx}")
        print(f"  类型: {tensor_type}({tensor.type})")
        print(f"  形状: {shape}")
        print()
    
    # 打印输出张量
    print("=== 输出张量 ===")
    for output_idx in subgraph.outputs:  # 修改点：直接遍历subgraph.outputs
        tensor = subgraph.tensors[output_idx]
        tensor_type = get_tensor_type_name(tensor)
        shape = tensor.shape
        
        print(f"输出:")
        print(f"  名称: {tensor.name}")
        print(f"  索引: {output_idx}")
        print(f"  类型: {tensor_type}({tensor.type})")
        print(f"  形状: {shape}")
        print()

    print("-" * 40)

    print("=== 模型中的张量: ===")
    for i, tensor in enumerate(subgraph.tensors):  # 修改点：直接遍历tensors列表
        tensor_type = get_tensor_type_name(tensor)
        shape = tensor.shape        
        print(f"张量 {i}:")
        print(f"  名称: {tensor.name}")
        print(f"  索引: {i}")
        print(f"  类型: {tensor_type}({tensor.type})")
        print(f"  形状: {shape}")
        print()
 
    print("-" * 40)
 
    # 打印操作符
    print("=== 操作符 ===")
    for i, op in enumerate(subgraph.operators):  # 修改点：直接遍历subgraph.operators
        op_code = model.operatorCodes[op.opcodeIndex]
        
        # 获取操作符名称
        builtin_code = op_code.builtinCode
        if builtin_code != schema_fb.BuiltinOperator.CUSTOM:
            op_name = get_buildin_op_type_name(builtin_code)
        else:
            op_name = f"自定义操作: {op_code.customCode}"
        
        # 获取输入和输出张量
        inputs = op.inputs  # 修改点：直接使用op.inputs
        outputs = op.outputs  # 修改点：直接使用op.outputs
        
        print(f"操作符 {i}: {op_name}({builtin_code})")
        print(f"  输入张量索引: {inputs}")
        print(f"  输出张量索引: {outputs}")
        print()

# 使用示例
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--tflite_path',
        type=str,
        default='model.tflite',
        help='Path to TFLite file to use for testing.')
    FLAGS, unparsed = parser.parse_known_args()
    print_model_structure(FLAGS.tflite_path)