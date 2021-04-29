#ifndef NN_H
#define NN_H

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

int nn_setup(void);
int nn_classify(void);

#ifdef __cplusplus
}  // extern "C"
#endif  // __cplusplus

extern const int8_t raw_data[256 * 3];

#endif
