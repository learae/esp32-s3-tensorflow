#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <esp_heap_caps.h>
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "../bsp/ov5640.h"
#include "model.h"

#include "tensorflow/lite/c/common.h"




namespace {
    const tflite::Model* model = nullptr;
    tflite::MicroInterpreter* interpreter = nullptr;
    TfLiteTensor* input = nullptr;
  
  // In order to use optimized tensorflow lite kernels, a signed int8_t quantized
  // model is preferred over the legacy unsigned model format. This means that
  // throughout this project, input images must be converted from unisgned to
  // signed format. The easiest and quickest way to convert from unsigned to
  // signed 8-bit integers is to subtract 128 from the unsigned value to get a
  // signed value.
  
  #if CONFIG_NN_OPTIMIZED
  constexpr int scratchBufSize = 200 * 1024;
  #else
  constexpr int scratchBufSize = 0;
  #endif
  // An area of memory to use for input, output, and intermediate arrays.
  // Keeping allocation on bit larger size to accomodate future needs.
  constexpr int kTensorArenaSize = 200 * 1024 + scratchBufSize;
  static uint8_t *tensor_arena;//[kTensorArenaSize]; // Maybe we should move this to external
  }  // namespace

static bool g_tflite_ready = false;  

void setup() {
    // Map the model into a usable data structure. This doesn't involve any
    // copying or parsing, it's a very lightweight operation.
    ESP_LOGI("tensorflow","Model size");
    model = tflite::GetModel(model_tflite);
    if (model->version() != TFLITE_SCHEMA_VERSION) {
      ESP_LOGI("tensorflow","Model provided is schema version not equal to supported "      );
      return;
    }
  
    if (tensor_arena == NULL) {
      tensor_arena = (uint8_t *) heap_caps_malloc(kTensorArenaSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    }
    if (tensor_arena == NULL) {
      ESP_LOGI("tensorflow","Couldn't allocate memory of %d bytes\n", kTensorArenaSize);
      return;
    }

    // Pull in only the operation implementations we need.
    // This relies on a complete list of all the ops needed by this graph.
    // An easier approach is to just use the AllOpsResolver, but this will
    // incur some penalty in code space for op implementations that are not
    // needed by this graph.
    //
    // tflite::AllOpsResolver resolver;
    // NOLINTNEXTLINE(runtime-global-variables)
    static tflite::MicroMutableOpResolver <12> micro_op_resolver;
    micro_op_resolver.AddAveragePool2D();
    micro_op_resolver.AddConv2D();
    micro_op_resolver.AddMaxPool2D();
    micro_op_resolver.AddReshape();
    micro_op_resolver.AddSoftmax();
    micro_op_resolver.AddDequantize(); 
    micro_op_resolver.AddMean();  // 对应 GlobalAveragePooling2D
    micro_op_resolver.AddCast();
    micro_op_resolver.AddAdd();
    micro_op_resolver.AddRelu();
    micro_op_resolver.AddFullyConnected();
    micro_op_resolver.AddQuantize();
    // Build an interpreter to run the model with.
    // NOLINTNEXTLINE(runtime-global-variables)
    static tflite::MicroInterpreter static_interpreter(
        model, micro_op_resolver, tensor_arena, kTensorArenaSize);
    interpreter = &static_interpreter;
  
    // Allocate memory from the tensor_arena for the model's tensors.
    TfLiteStatus allocate_status = interpreter->AllocateTensors();
    if (allocate_status != kTfLiteOk) {
        ESP_LOGI("tensorflow","AllocateTensors() failed");
      g_tflite_ready = false;
      return;
    }
  
    // Get information about the memory area to use for the model's input.
    input = interpreter->input(0);
    g_tflite_ready = true;
    ESP_LOGI("tensorflow", "input dims: %d %d %d %d", 
    input->dims->data[0], input->dims->data[1], input->dims->data[2], input->dims->data[3]);
ESP_LOGI("tensorflow", "input type: %d, input bytes: %d", input->type, input->bytes);
  }


void loop() {
    if (g_tflite_ready != true)
    {
      return;
    }
    if (input == nullptr || input->data.raw == nullptr) {
      ESP_LOGI("tensorflow","Input tensor is not properly allocated");
      return;
  }
    // Get the input image from the camera
    
    ESP_LOGI("tensorflow","img_get");
    if (input->type == kTfLiteFloat32) {
        img_get_float(480, 320, (float *)input->data.f);
        // Print the first 10 values of input data
    } else if (input->type == kTfLiteInt8) {
        img_get_int(480, 320, (int8_t*)input->data.int8);
    } else {
        ESP_LOGE("tensorflow", "Unsupported input tensor type: %d", input->type);
        return;
    }
    ESP_LOGI("tensorflow","img_get done");
    // Run the model on the input data
    TfLiteStatus invoke_status = interpreter->Invoke();
        if (invoke_status != kTfLiteOk) {
      ESP_LOGI("tensorflow","Invoke failed");
      return;
    }
  
    // Get the output tensor
    ESP_LOGI("tensorflow","Output tensor");
    TfLiteTensor* output = interpreter->output(0);
  
    // Print the output values
    /*for (int i = 0; i < output->dims->data[1]; ++i) {
      ESP_LOGI("tensorflow","%d: %f\n", i, output->data.f[i]);
    }*/
    
    if(output->data.f[1]>0.5)
    {
      ESP_LOGI("tensorflow","香蕉");
    }
    else if(output->data.f[0]>0.5)
    {
      ESP_LOGI("tensorflow","苹果");
    }
    else if(output->data.f[2]>0.5)
    {
      ESP_LOGI("tensorflow","橘子");
    }
    else{
      ESP_LOGI("tensorflow","其他");
    }
  }

extern "C" {
  void tensorflow_task(void *param)
  {
      ESP_LOGI("tensorflow","Starting TensorFlow Lite Micro task...");
      setup();
      while(1)
      {
          ESP_LOGI("tensorflow","Running TensorFlow Lite Micro model...");
          loop();
          vTaskDelay(600/portTICK_PERIOD_MS);
      }
  }
}