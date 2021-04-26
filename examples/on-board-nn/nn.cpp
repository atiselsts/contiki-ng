extern "C" {
#include "contiki.h"

#include "exported-nn.c"
}

// This constant represents the range of x values our model was trained on,
// which is from 0 to (2 * Pi). We approximate Pi to avoid requiring additional
// libraries.
const float kXrange = 2.f * 3.14159265359f;

// This constant determines the number of inferences to perform across the range
// of x values defined above. Since each inference takes time, the higher this
// number, the more time it will take to run through the entire range. The value
// of this constant can be tuned so that one full cycle takes a desired amount
// of time. Since different devices take different amounts of time to perform
// inference, this value should be defined per-device.
const int kInferencesPerCycle = 1000;


#include "tensorflow/lite/micro/all_ops_resolver.h"
// #include "tensorflow/lite/micro/examples/hello_world/constants.h"
// #include "tensorflow/lite/micro/examples/hello_world/model.h"
// #include "tensorflow/lite/micro/examples/hello_world/output_handler.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

void operator delete(void*) {
  /* no-op */
}


// Globals, used for compatibility with Arduino-style sketches.
namespace {
tflite::ErrorReporter* error_reporter = nullptr;
const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input = nullptr;
TfLiteTensor* output = nullptr;
int inference_count = 0;

// Create an area of memory to use for input, output, and intermediate arrays.
// Minimum arena size, at the time of writing. After allocating tensors
// you can retrieve this value by invoking interpreter.arena_used_bytes().
const int kModelArenaSize = 10000;//754;
// Extra headroom for model + alignment + future interpreter changes.
const int kExtraArenaSize = 10000;//554 + 16 + 100;
const int kTensorArenaSize = kModelArenaSize + kExtraArenaSize;
uint8_t tensor_arena[kTensorArenaSize];
}  // namespace

// The name of this function is important for Arduino compatibility.
void setup() {
//  printf("nn: setup\n");

  // printf("sizeof(TfLiteTensor)=%u %u sizeof(void*)=%u\n",
  //     sizeof(TfLiteTensor), alignof(TfLiteTensor), sizeof(input->dims));

  // Set up logging. Google style is to avoid globals or statics because of
  // lifetime uncertainty, but since this has a trivial destructor it's okay.
  // NOLINTNEXTLINE(runtime-global-variables)
  static tflite::MicroErrorReporter micro_error_reporter;
  error_reporter = &micro_error_reporter;

  // Map the model into a usable data structure. This doesn't involve any
  // copying or parsing, it's a very lightweight operation.
//  model = tflite::GetModel(g_model);
//  model = tflite::GetModel(feature_nn_tflite);
  model = tflite::GetModel(cnn_quant_int_tflite);
//  model = tflite::GetModel(test_cnn);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    TF_LITE_REPORT_ERROR(error_reporter,
                         "Model provided is schema version %d not equal "
                         "to supported version %d.",
                         model->version(), TFLITE_SCHEMA_VERSION);
    return;
  }

  // This pulls in all the operation implementations we need.
  // NOLINTNEXTLINE(runtime-global-variables)
  static tflite::AllOpsResolver resolver;

  // Build an interpreter to run the model with.
  static tflite::MicroInterpreter static_interpreter(
      model, resolver, tensor_arena, kTensorArenaSize, error_reporter);
  interpreter = &static_interpreter;

//  printf("alloc tensors\n");
  // Allocate memory from the tensor_arena for the model's tensors.
  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    printf("AllocateTensors() failed\n");
    TF_LITE_REPORT_ERROR(error_reporter, "AllocateTensors() failed");
    return;
  }

  // Obtain pointers to the model's input and output tensors.
  input = interpreter->input(0);
  output = interpreter->output(0);

  // TF_LITE_REPORT_ERROR(error_reporter,
  //     "interpreter.arena_used_bytes()=%u\n", interpreter->arena_used_bytes());
  // TF_LITE_REPORT_ERROR(error_reporter,
  //     "input=%x\n", (uint32_t)input);
  // TF_LITE_REPORT_ERROR(error_reporter,
  //     "dims=%x\n", (unsigned)input->dims);
  // int stack;
  // TF_LITE_REPORT_ERROR(error_reporter,
  //     "stack=%x tensor_arena=%x\n", (uint32_t)&stack, (uint32_t)&tensor_arena);

  //   const flatbuffers::Vector<int32_t> *shape() const {
  //   return GetPointer<const flatbuffers::Vector<int32_t> *>(VT_SHAPE);
  // }


  // TF_LITE_REPORT_ERROR(error_reporter,
  //     "input->dims->size=%u output->dims->size=%u\n",
  //     (uint32_t)input->dims->size,
  //     (uint32_t)output->dims->size);

  // TF_LITE_REPORT_ERROR(error_reporter,
  //     "input->dims->size=%u\n",
  //     (uint32_t)input->dims->size);


//  TF_LITE_REPORT_ERROR(error_reporter,
//      "input=%u dims=%u dims->size=%u\n", (uint32_t)input, (uint32_t)input->dims, input->dims->size);

//  if (/*(input->dims->size != 4) || (input->dims->data[0] != 1) ||
//      (input->dims->data[1] != 128) ||
//      (input->dims->data[2] != kChannelNumber) || */


// Keep track of how many inferences we have performed.
  inference_count = 0;
}

// The name of this function is important for Arduino compatibility.
int loop()
{
  // Calculate an x value to feed into the model. We compare the current
  // inference_count to the number of inferences per cycle to determine
  // our position within the range of possible x values the model was
  // trained on, and use this to calculate a value.
  // float position = static_cast<float>(inference_count) /
  //                  static_cast<float>(kInferencesPerCycle);
  // float x = position * kXrange;

  // // Quantize the input from floating-point to integer
  // int8_t x_quantized = x / input->params.scale + input->params.zero_point;
  // // Place the quantized input in the model's input tensor
  // input->data.int8[0] = x_quantized;

  // // Run inference, and report any error
  // TfLiteStatus invoke_status = interpreter->Invoke();
  // if (invoke_status != kTfLiteOk) {
  //   TF_LITE_REPORT_ERROR(error_reporter, "Invoke failed on x: %f\n",
  //                        static_cast<double>(x));
  //   return;
  // }

  // // Obtain the quantized output from model's output tensor
  // int8_t y_quantized = output->data.int8[0];
  // // Dequantize the output from integer to floating-point
  // float y = (y_quantized - output->params.zero_point) * output->params.scale;

  // // Output the results.
  // TF_LITE_REPORT_ERROR(error_reporter, "x=%f y=%f\n", x, y);

  // // Increment the inference_counter, and reset it if we have reached
  // // the total number per cycle
  // inference_count += 1;
  // if (inference_count >= kInferencesPerCycle) inference_count = 0;

  // const float *f = data[inference_count * 200];
  // int i;
  // for (i = 0; i < NUM_FEATURES; ++i) {
  //   input->data.f[i] = f[i];
  // }

  int i;
  for (i = 0; i < 3 * 256; ++i) {
    input->data.int8[i] = 1;
  }


  // Run inference, and report any error
  TfLiteStatus invoke_status = interpreter->Invoke();
  if (invoke_status != kTfLiteOk) {
    TF_LITE_REPORT_ERROR(error_reporter, "Invoke failed\n");
    return -1;
  }

//  TF_LITE_REPORT_ERROR(error_reporter, "output\n");
  int best_class = 0;
  for (i = 1; i < NUM_CLASSES; ++i) {
//    TF_LITE_REPORT_ERROR(error_reporter, "  out[%d]=%f\n", i,  output->data.f[i]);
//    TF_LITE_REPORT_ERROR(error_reporter, "  out[%d]=%d\n", i,  output->data.int8[i]);
    if (output->data.int8[i] > output->data.int8[best_class]) {
      best_class = i;
    }
//    result += output->data.int8[i];
  }

//  inference_count++;
  return best_class;
}


extern "C" int nn_setup(void)
{
  setup();
  return 0;
}

extern "C" int nn_classify(void)
{
  return loop();
}
