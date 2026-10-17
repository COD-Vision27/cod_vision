# OPENVINO

项目涉及关于openvino相关库函数/类型

1. `ov::Core`

   入口对象，管理可用设备，加载插件，读取模型等

   下面是读取模型的示例

   ```cpp
   ov::Core core_
   // 读取模型
   auto model = core_.read_model("model.xml", "model.bin");
   // 或者 ONNX
   auto model = core_.read_model("model.onnx");
   ```

   这两种格式都是openvino可读取的模型，xml+bin被称作IR模型

   编译模型到指定设备

   ```cpp
   ov::CompiledModel compiled_model_ = core_.compile_model("model.xml", "CPU");
   ```


2. `ov::preprocess::PrePostProcessor`

   给模型配置预处理和后处理,调控一些模型的设置，比如输入大小等

   ```cpp
   ov::preprocess::PrePostProcessor ppp(model);
   ```

   创建一个这样的预/后处理器ppp,用于处理model

   下面是项目设置模型图像输入的设置

   ```cpp
   ov::preprocess::PrePostProcessor ppp(model);
   // 单拿出输入设置
   auto & input = ppp.input();
   input.tensor()
     .set_element_type(ov::element::u8)
     .set_shape({1, 640, 640, 3})
     .set_layout("NHWC")
     .set_color_format(ov::preprocess::ColorFormat::BGR);
   //Runtime 会自动在预处理阶段插入布局转换 NHWC → NCHW
   input.model().set_layout("NCHW");
   input.preprocess()
     .convert_element_type(ov::element::f32)//把 U8 转成 FP32
     .convert_color(ov::preprocess::ColorFormat::RGB)//BGR 转成 RGB
     .scale(255.0);//数值从 0–255 缩放到 0–1
   model = ppp.build();//包含预处理节点的新model
   //编译到设备，并设置性能模式为LATENCY
   compiled_model_ = core_.compile_model(
       model, device_,
       ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
   ```

   | 配置         | 值                 | 含义                               |
   | ------------ | ------------------ | ---------------------------------- |
   | element_type | `u8`               | 你传入的数据是 `uint8`，范围 0–255 |
   | shape        | `{1, 640, 640, 3}` | 批次 1，高 640，宽 640，通道 3     |
   | layout       | `"NHWC"`           | 内存排布是 N=1, H=640, W=640, C=3  |
   | color_format | `BGR`              | 通道顺序是 BGR（OpenCV 默认）      |

3. `ov::CompiledModel`

   表示已经被编译到某个设备上的模型

   提供输入输出信息,创建推理请求

   ```cpp
   // 创建推理请求
   ov::InferRequest request = compiled_model_.create_infer_request();
   // 获取输入输出信息
   auto inputs = compiled_model_.inputs();
   auto outputs = compiled_model_.outputs();
   // 设置输入
   ov::Tensor input_tensor(ov::element::u8, {1, 640, 640, 3}, image.data);
   // 填充 input_tensor 数据
   request.set_input_tensor(input_tensor);
   // 执行推理
   request.infer();
   // 获取输出
   auto output = request.get_output_tensor();
   ```
