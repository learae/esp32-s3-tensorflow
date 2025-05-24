项目使用的是微雪esp32-s3-touch-lcd-2,带8m外部的spiram，16m的flash，摄像头使用的是ov5640，更多详细信息可以去微雪的官网查看。

芯片运行基于esp-idf，运行tensorflow lite使用的是官方的依赖ESP-TFLITE-MICRO，目前网上的教程比较少，我会介绍一下关键的步骤和大致流程。

# tensorflow部分
这里使用的是vgg16经典模型，详细代码见[tensorflow-lite.ipynb](https://github.com/learae/esp32-s3-tensorflow/blob/main/tensorflow-lite.ipynb)

//应用量化感知训练

	quantize_model = tfmot.quantization.keras.quantize_model
	
	base_model = create_vgg16_base_model()
	
	qat_model = quantize_model(base_model)

如果需要量化为int8模型，这一段很重要，int8模型的体积只有float32的四分之一，更适合内存不大嵌入式设备，但是准确率也会下降。

量化后的tflite可以在https://netron.app/ 里查看输入输出的数据格式和用到的算子

这里需要主要使用的算子必须是tensorflow lite支持的算子，如果使用的flex等tensorflow lite不支持的算子，在esp-tflite-micro中会无法识别，如果要使用需要在esp-tflite-micro中手动实现和添加声明。

如何就是将生成的tflite模型在linux下用xxd转换为c数组,具体操作如下

	xxd -i model.tflite > model.cc

如果model.cc数组比较大，建议把它定义入外部SPIRAM中，否则esp32的sram可能不够

至此，tflite模型已经准备就绪，接下来介绍在esp-idf中的关键步骤

# esp-idf部分

添加esp-tflite-micro依赖，具体操作如下

	idf.py add-dependency “espressif/esp-tflite-micro^1.3.1”

https://components.espressif.com/components/espressif/esp-tflite-micro/versions/1.3.1?language=en 在乐鑫官网也有具体操作


定义tflite模型，解释器，张量输入指针，张量输出指针，张量竞技场

namespace {

        const tflite::Model* model = nullptr;
		
	tflite::MicroInterpreter* interpreter = nullptr;
	
	TfLiteTensor* input = nullptr;
	
	TfLiteTensor* output = nullptr;
	
	#if CONFIG_NN_OPTIMIZED
	
	constexpr int scratchBufSize = 200 * 1024;
	
	#else
	
	constexpr int scratchBufSize = 0;
	
	#endif
	
	constexpr int kTensorArenaSize = 200 * 1024 + scratchBufSize;
	
	static uint8_t *tensor_arena;
	
  }

步骤一：将tflite转换后的c数组传入model

	model = tflite::GetModel(model_tflite);

步骤二：为tensor_arena分配内存，内存大小最少为单张图片大小加算子大小，同样建议定义在SPIRAM里面

	tensor_arena = (uint8_t *) heap_caps_malloc(kTensorArenaSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

步骤三：注册算子，所有用到的算子都需要注册，可以在https://netron.app/ 里查看所有用到的算子

	static tflite::MicroMutableOpResolver <10> micro_op_resolver;

	    micro_op_resolver.AddAveragePool2D();
	
			.......

步骤四：定义解释器

	static tflite::MicroInterpreter static_interpreter(
	
	        model, micro_op_resolver, tensor_arena, kTensorArenaSize);
					
	    interpreter = &static_interpreter;

步骤五：目的是根据模型的计算图结构，确定所有中间张量的形状和数量，为输入、输出和所有中间计算所需的张量分配内存

	TfLiteStatus allocate_status = interpreter->AllocateTensors();

步骤六：定义输入，并将数据传入输入，这里我定义了两套数据获取函数，分别应对量化模型为float32和int8，确保模型可以正确运行

	input = interpreter->input(0);//0用于检查输入数据是否为空，如果输入数据的长度确定，也可以数据长度

//以上包括步骤一到五一般在初始化中完成，在这里传入的数据必须和tensorflow里训练时的一致。

	if (input->type == kTfLiteFloat32) {
	
	        img_get_float(480, 320, (float *)input->data.f);
					
	    } else if (input->type == kTfLiteInt8) {
			
	        img_get_int(480, 320, (int8_t*)input->data.int8);

步骤七：开始推理，推理完成后获得输出结果

	TfLiteStatus invoke_status = interpreter->Invoke();
	
	if (invoke_status != kTfLiteOk) {
	
	      ESP_LOGI("tensorflow","Invoke failed");
				
	      return;
				
	    }
	
	output = interpreter->output(0);//与input(0)同理

以上就是tensorflow模型在esp32上运行的大概流程，更具体流程的可以参考官方示例，希望本篇对您有帮助，可以帮你排掉一些我踩过的坑










