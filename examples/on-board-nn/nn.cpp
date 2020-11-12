extern "C" {
#include "contiki.h"

#include "exported-nn.c"
}

#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"
#include "tensorflow/lite/micro/all_ops_resolver.h"


tflite::ErrorReporter* error_reporter = nullptr;
const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* model_input = nullptr;
int input_length;

// Create an area of memory to use for input, output, and intermediate arrays.
// The size of this will depend on the model you're using, and may need to be
// determined by experimentation.
constexpr int tensor_arena_size = 60 * 1024;
uint8_t tensor_arena[tensor_arena_size];

void operator delete(void*) {
  /* no-op */
}

int nn_classify_single(const float features[])
{
  model_input->data.f[0] = 0;

  // Run inference, and report any error.
  TfLiteStatus invoke_status = interpreter->Invoke();
  if (invoke_status != kTfLiteOk) {
    TF_LITE_REPORT_ERROR(error_reporter, "Invoke failed on index\n");
    return 0;
  }

  float *f = interpreter->output(0)->data.f;

  TF_LITE_REPORT_ERROR(error_reporter, "Output 0: %f\n",
                       f[0]);

  return 0;
}

extern "C" void nn_setup(void)
{
  // Set up logging. Google style is to avoid globals or statics because of
  // lifetime uncertainty, but since this has a trivial destructor it's okay.
  static tflite::MicroErrorReporter micro_error_reporter;  // NOLINT
  error_reporter = &micro_error_reporter;

  // Map the model into a usable data structure. This doesn't involve any
  // copying or parsing, it's a very lightweight operation.
  model = tflite::GetModel(feature_nn_tflite);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    TF_LITE_REPORT_ERROR(error_reporter,
                         "Model provided is schema version %d not equal "
                         "to supported version %d.",
                         model->version(), TFLITE_SCHEMA_VERSION);
    return;
  }

  tflite::AllOpsResolver resolver;

  static tflite::MicroInterpreter static_interpreter(model, resolver, tensor_arena,
                                                     tensor_arena_size, error_reporter);

  interpreter = &static_interpreter;

  interpreter->AllocateTensors();

  model_input = interpreter->input(0);

  TF_LITE_REPORT_ERROR(error_reporter,
      "dims->size=%u\n", model_input->dims);

  if (/*(model_input->dims->size != 4) || (model_input->dims->data[0] != 1) ||
      (model_input->dims->data[1] != 128) ||
      (model_input->dims->data[2] != kChannelNumber) || */
      (model_input->type != kTfLiteFloat32)) {
    TF_LITE_REPORT_ERROR(error_reporter,
                         "Bad input tensor parameters in model");
    return;
  }

  input_length = model_input->bytes / sizeof(float);

  TF_LITE_REPORT_ERROR(error_reporter,
      "Hello world from TF!\n");
}

extern "C" int nn_classify(void)
{
    int i;
    int dummy = 0;
    for (i = 0; i < NUM_DATA; ++i) {
        dummy += nn_classify_single(data[i]);
    }
    printf("classified from C++!\n");
    return dummy;
}

