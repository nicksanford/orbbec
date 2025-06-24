// Copyright 2025 Viam Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>

#include <tensorflow/lite/c/c_api.h>
#include <tensorflow/lite/interpreter_builder.h>
#include <tensorflow/lite/kernels/register.h>

#include <viam/sdk/common/instance.hpp>
#include <viam/sdk/common/proto_value.hpp>
#include <viam/sdk/components/component.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/module/service.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/resource/reconfigurable.hpp>
#include <viam/sdk/resource/stoppable.hpp>
#include <viam/sdk/rpc/server.hpp>
#include <viam/sdk/services/mlmodel.hpp>

namespace {

namespace vsdk = ::viam::sdk;

class Orbbec : public vsdk::Camera, public vsdk::Reconfigurable {

public:
  explicit Orbbec(vsdk::Dependencies dependencies, public vsdk::Stoppable,
                  vsdk::ResourceConfig configuration)
      : Camera(configuration.name()),
        state_(configure_(std::move(dependencies), std::move(configuration))) {}

  ~MLModelServiceTFLite() final {
    // All invocations arrive via gRPC, so we know we are idle
    // here. It should be safe to tear down all state
    // automatically without needing to wait for anything more to
    // drain.
  }

  void reconfigure(const vsdk::Dependencies &dependencies,
                   const vsdk::ResourceConfig &configuration) final {

    const std::lock_guard<std::mutex> lock(state_lock_);
    state_.reset();
    state_ = configure_(dependencies, configuration);
  }

  vsdk::Camera::raw_image get_image(std::string mime_type,
                                    const vsdk::ProtoStruct &extra) {
    if (debug_enabled) {
      VIAM_SDK_LOG(info) << "[get_image] start";
    }
    try {
      std::chrono::time_point<std::chrono::high_resolution_clock> start;
      if (debug_enabled) {
        start = std::chrono::high_resolution_clock::now();
      }

      rs2::frame latestColorFrame;
      std::shared_ptr<std::vector<uint16_t>> latestDepthFrame;
      {
        std::lock_guard<std::mutex> lock(this->latest_frames_.mutex);
        latestColorFrame = this->latest_frames_.colorFrame;
        latestDepthFrame = this->latest_frames_.depthFrame;
      }
      std::unique_ptr<vsdk::Camera::raw_image> response;
      if (this->props_.mainSensor.compare("color") == 0) {
        if (this->device_->disableColor) {
          throw std::invalid_argument("color disabled");
        }
        if (mime_type.compare("image/png") == 0 ||
            mime_type.compare("image/png+lazy") == 0) {
          response = encodeColorPNGToResponse(
              (const void *)latestColorFrame.get_data(),
              this->props_.color.width, this->props_.color.height);
        } else if (mime_type.compare("image/vnd.viam.rgba") == 0) {
          response = encodeColorRAWToResponse(
              (const unsigned char *)latestColorFrame.get_data(),
              this->props_.color.width, this->props_.color.height);
        } else {
          response = encodeJPEGToResponse(
              (const unsigned char *)latestColorFrame.get_data(),
              this->props_.color.width, this->props_.color.height);
        }
      } else if (this->props_.mainSensor.compare("depth") == 0) {
        if (this->device_->disableDepth) {
          throw std::invalid_argument("depth disabled");
        }
        if (mime_type.compare("image/vnd.viam.dep") == 0) {
          response = encodeDepthRAWToResponse(
              (const unsigned char *)latestDepthFrame->data(),
              this->props_.depth.width, this->props_.depth.height,
              this->props_.littleEndianDepth);
        } else {
          response = encodeDepthPNGToResponse(
              (const unsigned char *)latestDepthFrame->data(),
              this->props_.depth.width, this->props_.depth.height);
        }
      }

      if (debug_enabled) {
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration =
            std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        VIAM_SDK_LOG(info) << "[get_image]  total:           "
                           << duration.count() << "ms\n";
      }

      if (debug_enabled) {
        VIAM_SDK_LOG(info) << "[get_image] end";
      }
      return std::move(*response);
    } catch (const std::exception &e) {
      VIAM_SDK_LOG(error) << "[get_image] failed to get image: " << e.what();
      throw;
    }
  }

  vsdk::Camera::properties get_properties() {
    if (debug_enabled) {
      VIAM_SDK_LOG(info) << "[get_properties] start";
    }
    try {
      auto fillResp = [](vsdk::Camera::properties *p, CameraProperties props) {
        p->supports_pcd = true;
        p->intrinsic_parameters.width_px = props.width;
        p->intrinsic_parameters.height_px = props.height;
        p->intrinsic_parameters.focal_x_px = props.fx;
        p->intrinsic_parameters.focal_y_px = props.fy;
        p->intrinsic_parameters.center_x_px = props.ppx;
        p->intrinsic_parameters.center_y_px = props.ppy;
        p->distortion_parameters.model = props.distortionModel;
        for (int i = 0; i < std::size(props.distortionParameters); i++) {
          p->distortion_parameters.parameters.push_back(
              props.distortionParameters[i]);
        }
      };

      vsdk::Camera::properties response{};
      if (this->props_.mainSensor.compare("color") == 0) {
        fillResp(&response, this->props_.color);
      } else if (this->props_.mainSensor.compare("depth") == 0) {
        fillResp(&response, this->props_.depth);
      }
      if (debug_enabled) {
        VIAM_SDK_LOG(info) << "[get_properties] end";
      }
      return response;
    } catch (const std::exception &e) {
      VIAM_SDK_LOG(error) << "[get_properties] failed to get properties: "
                          << e.what();
      throw;
    }
  }

  vsdk::Camera::image_collection get_images() {
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
    if (debug_enabled) {
      start = std::chrono::high_resolution_clock::now();
    }
    vsdk::Camera::image_collection response;

    rs2::frame latestColorFrame;
    std::shared_ptr<std::vector<uint16_t>> latestDepthFrame;
    std::chrono::milliseconds latestTimestamp;
    {
      std::lock_guard<std::mutex> lock(this->latest_frames_.mutex);
      latestColorFrame = this->latest_frames_.colorFrame;
      latestDepthFrame = this->latest_frames_.depthFrame;
      latestTimestamp = this->latest_frames_.timestamp;
    }

    for (const auto &sensor : this->props_.sensors) {
      if (sensor == "color") {
        std::unique_ptr<vsdk::Camera::raw_image> color_response;
        color_response = encodeJPEGToResponse(
            (const unsigned char *)latestColorFrame.get_data(),
            this->props_.color.width, this->props_.color.height);
        response.images.emplace_back(std::move(*color_response));
      } else if (sensor == "depth") {
        std::unique_ptr<vsdk::Camera::raw_image> depth_response;
        depth_response = encodeDepthRAWToResponse(
            (const unsigned char *)latestDepthFrame->data(),
            this->props_.depth.width, this->props_.depth.height,
            this->props_.littleEndianDepth);
        response.images.emplace_back(std::move(*depth_response));
      }
    }

    response.metadata.captured_at = vsdk::time_pt{
        std::chrono::duration_cast<std::chrono::nanoseconds>(latestTimestamp)};

    if (debug_enabled) {
      auto stop = std::chrono::high_resolution_clock::now();
      auto duration =
          std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
      VIAM_SDK_LOG(info) << "[get_images]  total:           "
                         << duration.count() << "ms\n";
    }

    return response;
  }

  vsdk::ProtoStruct do_command(const vsdk::ProtoStruct &command) {
    VIAM_SDK_LOG(error) << "do_command not implemented";
    return vsdk::ProtoStruct{};
  }

  vsdk::Camera::point_cloud get_point_cloud(std::string mime_type,
                                            const vsdk::ProtoStruct &extra) {
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
    if (debug_enabled) {
      start = std::chrono::high_resolution_clock::now();
    }

    rs2::frame latestColorFrame;
    rs2::frame latestDepthFrame;
    rs2::pointcloud pc;
    rs2::points points;
    std::vector<unsigned char> pcdBytes;
    {
      std::lock_guard<std::mutex> lock(this->latest_frames_.mutex);
      latestColorFrame = this->latest_frames_.colorFrame;
      latestDepthFrame = this->latest_frames_.rsDepthFrame;
    }

    if (latestColorFrame) {
      pc.map_to(latestColorFrame);
    }
    if (!latestDepthFrame) {
      VIAM_SDK_LOG(error)
          << "cannot get point cloud as there is no depth frame";
      return vsdk::Camera::point_cloud{};
    }
    points = pc.calculate(latestDepthFrame);
    pcdBytes = rsPointsToPCDBytes(points, latestColorFrame);

    if (debug_enabled) {
      auto stop = std::chrono::high_resolution_clock::now();
      auto duration =
          std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
      VIAM_SDK_LOG(info) << "[get_point_cloud]  total:           "
                         << duration.count() << "ms\n";
    }
    return vsdk::Camera::point_cloud{mime_type, pcdBytes};
  }

  std::vector<vsdk::GeometryConfig>
  get_geometries(const vsdk::ProtoStruct &extra) {
    VIAM_SDK_LOG(error) << "get_geometries not implemented";
    return std::vector<vsdk::GeometryConfig>{};
  }

private:
  struct state_;

  static std::unique_ptr<struct state_>
  configure_(vsdk::Dependencies dependencies,
             vsdk::ResourceConfig configuration) {

    auto state = std::make_unique<struct state_>(std::move(dependencies),
                                                 std::move(configuration));

    // // Now we can begin parsing and validating the provided `configuration`.
    // // Pull the model path out of the configuration.
    // const auto &attributes = state->configuration.attributes();
    // auto model_path = attributes.find("model_path");
    // if (model_path == attributes.end()) {
    //   std::ostringstream buffer;
    //   buffer << service_name
    //          << ": Required parameter `model_path` not found in
    //          configuration";
    //   throw std::invalid_argument(buffer.str());
    // }
    // const auto *const model_path_string =
    // model_path->second.get<std::string>(); if (!model_path_string ||
    // model_path_string->empty()) {
    //   std::ostringstream buffer;
    //   buffer << service_name
    //          << ": Required non-empty string parameter `model_path` is either
    //          "
    //             "not a string "
    //             "or is an empty string";
    //   throw std::invalid_argument(buffer.str());
    // }

    // std::string label_path_string = ""; // default value for label_path
    // auto label_path = attributes.find("label_path");
    // if (label_path != attributes.end()) {
    //   const auto *const lp_string = label_path->second.get<std::string>();
    //   if (!lp_string) {
    //     std::ostringstream buffer;
    //     buffer << service_name
    //            << ": string parameter `label_path` is not a string ";
    //     throw std::invalid_argument(buffer.str());
    //   }
    //   label_path_string = *lp_string;
    // }
    // state->label_path = std::move(label_path_string);

    // // Configuration parsing / extraction is complete. Move on to
    // // building the actual model with the provided information.

    // // Try to load the provided `model_path`. The TFLite API
    // // declares that if you use `TfLiteModelCreateFromFile` that
    // // the file must remain unaltered during execution, but
    // // reconfiguration might cause it to change on disk while
    // // inference is in progress. Instead we read the file into a
    // // buffer which we can use with `TfLiteModelCreate`. That
    // // still requires that the buffer be kept valid, but that's
    // // more easily done.
    // const std::ifstream in(*model_path_string, std::ios::in |
    // std::ios::binary); if (!in) {
    //   std::ostringstream buffer;
    //   buffer << service_name << ": Failed to open file for `model_path` "
    //          << *model_path_string;
    //   throw std::invalid_argument(buffer.str());
    // }
    // std::ostringstream model_path_contents_stream;
    // model_path_contents_stream << in.rdbuf();
    // state->model_data = std::move(model_path_contents_stream.str());

    // state->model = tflite::impl::FlatBufferModel::BuildFromBuffer(
    //     &state->model_data[0],
    //     std::distance(cbegin(state->model_data), cend(state->model_data)),
    //     state.get());

    // if (!state->model) {
    //   std::ostringstream buffer;
    //   buffer << service_name << ": Failed to load model from file `"
    //          << model_path_string << "`: " << state->interpreter_error_data;
    //   throw std::invalid_argument(buffer.str());
    // }

    // // Create an InterpreterBuilder so we can set the number of threads.
    // tflite::ops::builtin::BuiltinOpResolver resolver;
    // tflite::impl::InterpreterBuilder builder(*state->model, resolver);

    // // If present, extract and validate the number of threads to
    // // use in the interpreter and create an interpreter options
    // // object to carry that information.
    // auto num_threads = attributes.find("num_threads");
    // if (num_threads != attributes.end()) {
    //   const auto *num_threads_double = num_threads->second.get<double>();
    //   if (!num_threads_double || !std::isnormal(*num_threads_double) ||
    //       (*num_threads_double < 0) ||
    //       (*num_threads_double >= std::numeric_limits<int>::max()) ||
    //       (std::trunc(*num_threads_double) != *num_threads_double)) {
    //     std::ostringstream buffer;
    //     buffer << service_name
    //            << ": Value for field `num_threads` is not a positive integer:
    //            "
    //            << *num_threads_double;
    //     throw std::invalid_argument(buffer.str());
    //   }
    //   if (builder.SetNumThreads(static_cast<int>(*num_threads_double)) !=
    //       kTfLiteOk) {
    //     std::ostringstream buffer;
    //     buffer << service_name
    //            << ": Failed to set number of threads in interpreter builder:
    //            "
    //            << state->interpreter_error_data;
    //     throw std::invalid_argument(buffer.str());
    //   }
    // }

    // if (builder(&state->interpreter) != kTfLiteOk) {
    //   std::ostringstream buffer;
    //   buffer << service_name << ": Failed to create tflite interpreter: "
    //          << state->interpreter_error_data;
    //   throw std::runtime_error(buffer.str());
    // }

    // // Have the interpreter allocate tensors for the model
    // if (state->interpreter->AllocateTensors() != kTfLiteOk) {
    //   std::ostringstream buffer;
    //   buffer << service_name
    //          << ": Failed to allocate tensors for tflite interpreter: "
    //          << state->interpreter_error_data;
    //   throw std::runtime_error(buffer.str());
    // }

    // // Walk the input tensors now that they have been allocated
    // // and extract information about tensor names, types, and
    // // dimensions. Apply any tensor renamings per our
    // // configuration. Stash the relevant data in our `metadata`
    // // fields.
    // const auto input_tensor_indices = state->interpreter->inputs();
    // for (auto input_tensor_index : input_tensor_indices) {
    //   const auto *const tensor =
    //   state->interpreter->tensor(input_tensor_index);

    //   auto ndims = TfLiteTensorNumDims(tensor);
    //   if (ndims == -1) {
    //     std::ostringstream buffer;
    //     buffer << service_name
    //            << ": Unable to determine input tensor shape at configuration
    //            "
    //               "time, "
    //               "inference not possible";
    //     throw std::runtime_error(buffer.str());
    //   }

    //   MLModelService::tensor_info input_info;
    //   const auto *name = TfLiteTensorName(tensor);
    //   input_info.name = name;
    //   input_info.data_type =
    //       service_data_type_from_tflite_data_type_(TfLiteTensorType(tensor));
    //   for (decltype(ndims) j = 0; j != ndims; ++j) {
    //     input_info.shape.push_back(TfLiteTensorDim(tensor, j));
    //   }
    //   state->input_tensor_indices_by_name[input_info.name] =
    //   input_tensor_index;
    //   state->metadata.inputs.emplace_back(std::move(input_info));
    // }

    // // NOTE: The tflite C API docs state that information about
    // // output tensors may not be available until after one round
    // // of inference. We do a best effort inference on all zero
    // // inputs to try to account for this.
    // for (auto input_tensor_index : input_tensor_indices) {
    //   auto *const tensor = state->interpreter->tensor(input_tensor_index);
    //   const auto tensor_size = TfLiteTensorByteSize(tensor);
    //   const std::vector<unsigned char> zero_buffer(tensor_size, 0);
    //   TfLiteTensorCopyFromBuffer(tensor, &zero_buffer[0], tensor_size);
    // }

    // if (state->interpreter->Invoke() != TfLiteStatus::kTfLiteOk) {
    //   // TODO: After C++ SDK 0.11.0 is released, use the new logging API.
    //   std::cout << "WARNING: Inference with all zero input tensors failed: "
    //                "returned output tensor metadata may be unreliable"
    //             << std::endl;
    // }

    // // Now that we have hopefully done one round of inference, dig out the
    // // actual metadata that we will return to clients.
    // const auto output_tensor_indices = state->interpreter->outputs();
    // for (auto output_tensor_index : output_tensor_indices) {
    //   const auto *const tensor =
    //       state->interpreter->tensor(output_tensor_index);

    //   auto ndims = TfLiteTensorNumDims(tensor);
    //   if (ndims == -1) {
    //     std::ostringstream buffer;
    //     buffer << service_name
    //            << ": Unable to determine output tensor shape at configuration
    //            "
    //               "time, "
    //               "inference not possible";
    //     throw std::runtime_error(buffer.str());
    //   }

    //   MLModelService::tensor_info output_info;
    //   const auto *name = TfLiteTensorName(tensor);
    //   output_info.name = name;
    //   output_info.data_type =
    //       service_data_type_from_tflite_data_type_(TfLiteTensorType(tensor));
    //   for (decltype(ndims) j = 0; j != ndims; ++j) {
    //     output_info.shape.push_back(TfLiteTensorDim(tensor, j));
    //   }
    //   if (state->label_path != "") {
    //     output_info.extra.insert({"labels", state->label_path});
    //   }
    //   state->output_tensor_indices_by_name[output_info.name] =
    //       output_tensor_index;
    //   state->metadata.outputs.emplace_back(std::move(output_info));
    // }

    return state;
  }

  static std::unique_ptr<struct state_>

      // All of the meaningful internal state of the service is held in
      // a separate state object to help ensure clean replacement of our
      // internals during reconfiguration.
      struct state_ final : public tflite::ErrorReporter {
    explicit state_(vsdk::Dependencies dependencies,
                    vsdk::ResourceConfig configuration)
        : dependencies(std::move(dependencies)),
          configuration(std::move(configuration)) {}

    // The dependencies and configuration we were given at
    // construction / reconfiguration.
    vsdk::Dependencies dependencies;
    vsdk::ResourceConfig configuration;
  };

  // Accesss to the module state is serialized. All configuration
  // state is held in the `state` type to make it easier to destroy
  // the current state and replace it with a new one.
  std::mutex state_lock_;

  // In C++17, this could be `std::optional`.
  std::unique_ptr<struct state_> state_;
};

int serve(const std::string &socket_path) try {
  // Every Viam C++ SDK program must have one and only one Instance object which
  // is created before any other C++ SDK objects and stays alive until all Viam
  // C++ SDK objects are destroyed.
  vsdk::Instance inst;

  // Create a new model registration for the service.
  auto module_registration = std::make_shared<vsdk::ModelRegistration>(
      // Identify that this resource offers the MLModelService API
      vsdk::API::get<vsdk::Camera>(),

      // Declare a model triple for this service.
      vsdk::Model{"viam", "orbbec", "astra2"},

      // Define the factory for instances of the resource.
      [](vsdk::Dependencies deps, vsdk::ResourceConfig config) {
        return std::make_shared<Orbbec>(std::move(deps), std::move(config));
      });

  // Register the newly created registration with the Registry.
  vsdk::Registry::get().register_model(module_registration);

  // Construct the module service and tell it where to place the socket path.
  auto module_service = std::make_shared<vsdk::ModuleService>(socket_path);

  // Add the server as providing the API and model declared in the
  // registration.
  module_service->add_model_from_registry(module_registration->api(),
                                          module_registration->model());

  // Start the module service.
  module_service->serve();

  return EXIT_SUCCESS;
} catch (const std::exception &ex) {
  std::cout << "ERROR: A std::exception was thrown from `serve`: " << ex.what()
            << std::endl;
  return EXIT_FAILURE;
} catch (...) {
  std::cout << "ERROR: An unknown exception was thrown from `serve`"
            << std::endl;
  return EXIT_FAILURE;
}

} // namespace

int main(int argc, char *argv[]) {
  const std::string usage = "usage: orbbec /path/to/unix/socket";

  if (argc < 2) {
    std::cout << "ERROR: insufficient arguments\n";
    std::cout << usage << "\n";
    return EXIT_FAILURE;
  }

  return serve(argv[1]);
}
