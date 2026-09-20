#include <iostream>
#include <fstream>
#include <vector>
#include <cstring>
#include <cuda_runtime_api.h>
#include <NvInfer.h>

class Logger : public nvinfer1::ILogger {
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING) {
            std::cout << "[TRT] " << msg << std::endl;
        }
    }
} gLogger;

struct TrtContext {
    nvinfer1::IRuntime* runtime{nullptr};
    nvinfer1::ICudaEngine* engine{nullptr};
    nvinfer1::IExecutionContext* context{nullptr};
    cudaStream_t stream{nullptr};

    void* d_input{nullptr};
    void* d_out1{nullptr};
    void* d_out2{nullptr};
    void* d_out3{nullptr};
    void* d_out4{nullptr};

    void* bindings[5]{nullptr};
    int bind_in{-1}, bind_out1{-1}, bind_out2{-1}, bind_out3{-1}, bind_out4{-1};
};

extern "C" {

void* trt_create(const char* engine_path) {
    std::ifstream file(engine_path, std::ios::binary);
    if (!file.good()) {
        std::cerr << "Failed to open engine file: " << engine_path << std::endl;
        return nullptr;
    }
    file.seekg(0, std::ios::end);
    size_t size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<char> model_data(size);
    file.read(model_data.data(), size);
    file.close();

    TrtContext* ctx = new TrtContext();
    ctx->runtime = nvinfer1::createInferRuntime(gLogger);
    if (!ctx->runtime) {
        delete ctx;
        return nullptr;
    }

    ctx->engine = ctx->runtime->deserializeCudaEngine(model_data.data(), size);
    if (!ctx->engine) {
        delete ctx;
        return nullptr;
    }

    ctx->context = ctx->engine->createExecutionContext();
    if (!ctx->context) {
        delete ctx;
        return nullptr;
    }

    cudaStreamCreate(&ctx->stream);

    // Find bindings by name
    int nb_bindings = ctx->engine->getNbBindings();
    for (int i = 0; i < nb_bindings; ++i) {
        const char* name = ctx->engine->getBindingName(i);
        if (strcmp(name, "images") == 0) ctx->bind_in = i;
        else if (strstr(name, "Concat_1")) ctx->bind_out1 = i;
        else if (strstr(name, "Concat_2")) ctx->bind_out2 = i;
        else if (strstr(name, "Concat_3")) ctx->bind_out3 = i;
        else if (strstr(name, "Concat_6")) ctx->bind_out4 = i;
    }

    // Fallback index mapping if names differed
    if (ctx->bind_in < 0) ctx->bind_in = 0;
    if (ctx->bind_out1 < 0) ctx->bind_out1 = 1;
    if (ctx->bind_out2 < 0) ctx->bind_out2 = 2;
    if (ctx->bind_out3 < 0) ctx->bind_out3 = 3;
    if (ctx->bind_out4 < 0) ctx->bind_out4 = 4;

    // Allocate GPU buffers
    // input: 1x3x640x640 = 1228800 floats
    cudaMalloc(&ctx->d_input, 1 * 3 * 640 * 640 * sizeof(float));
    // out1: 1x65x80x80 = 416000 floats
    cudaMalloc(&ctx->d_out1, 1 * 65 * 80 * 80 * sizeof(float));
    // out2: 1x65x40x40 = 104000 floats
    cudaMalloc(&ctx->d_out2, 1 * 65 * 40 * 40 * sizeof(float));
    // out3: 1x65x20x20 = 26000 floats
    cudaMalloc(&ctx->d_out3, 1 * 65 * 20 * 20 * sizeof(float));
    // out4: 1x17x3x8400 = 428400 floats
    cudaMalloc(&ctx->d_out4, 1 * 17 * 3 * 8400 * sizeof(float));

    ctx->bindings[ctx->bind_in] = ctx->d_input;
    ctx->bindings[ctx->bind_out1] = ctx->d_out1;
    ctx->bindings[ctx->bind_out2] = ctx->d_out2;
    ctx->bindings[ctx->bind_out3] = ctx->d_out3;
    ctx->bindings[ctx->bind_out4] = ctx->d_out4;

    return ctx;
}

int trt_infer(void* handle, const float* input, float* out1, float* out2, float* out3, float* out4) {
    if (!handle) return -1;
    TrtContext* ctx = static_cast<TrtContext*>(handle);

    // H2D
    cudaMemcpyAsync(ctx->d_input, input, 1 * 3 * 640 * 640 * sizeof(float), cudaMemcpyHostToDevice, ctx->stream);

    // Enqueue
    ctx->context->enqueueV2(ctx->bindings, ctx->stream, nullptr);

    // D2H
    cudaMemcpyAsync(out1, ctx->d_out1, 1 * 65 * 80 * 80 * sizeof(float), cudaMemcpyDeviceToHost, ctx->stream);
    cudaMemcpyAsync(out2, ctx->d_out2, 1 * 65 * 40 * 40 * sizeof(float), cudaMemcpyDeviceToHost, ctx->stream);
    cudaMemcpyAsync(out3, ctx->d_out3, 1 * 65 * 20 * 20 * sizeof(float), cudaMemcpyDeviceToHost, ctx->stream);
    cudaMemcpyAsync(out4, ctx->d_out4, 1 * 17 * 3 * 8400 * sizeof(float), cudaMemcpyDeviceToHost, ctx->stream);

    cudaStreamSynchronize(ctx->stream);
    return 0;
}

void trt_destroy(void* handle) {
    if (!handle) return;
    TrtContext* ctx = static_cast<TrtContext*>(handle);
    if (ctx->d_input) cudaFree(ctx->d_input);
    if (ctx->d_out1) cudaFree(ctx->d_out1);
    if (ctx->d_out2) cudaFree(ctx->d_out2);
    if (ctx->d_out3) cudaFree(ctx->d_out3);
    if (ctx->d_out4) cudaFree(ctx->d_out4);
    if (ctx->stream) cudaStreamDestroy(ctx->stream);
    if (ctx->context) delete ctx->context;
    if (ctx->engine) delete ctx->engine;
    if (ctx->runtime) delete ctx->runtime;
    delete ctx;
}

}
