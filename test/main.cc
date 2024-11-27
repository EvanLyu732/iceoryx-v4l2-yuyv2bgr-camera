#include <cstdio>

#include "ShmImgBuf.h"
#include "iceoryx_posh/popo/subscriber.hpp"

//! [include subscriber]
#include <chrono>
#include <cstdlib>
#include <iceoryx_posh/popo/subscriber_options.hpp>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <ostream>

#include "iceoryx_hoofs/posix_wrapper/signal_watcher.hpp"
#include "iceoryx_posh/popo/listener.hpp"
#include "iceoryx_posh/runtime/posh_runtime.hpp"

static std::mutex m;

void showImg(void* img_buf, std::string title_name) {
  // std::lock_guard<std::mutex> lg(m);
  // std::cout << "title_name: " << title_name << std::endl;
  cv::Mat bgr_img(1080, 1920, CV_8UC3, img_buf);
  cv::resize(bgr_img, bgr_img, cv::Size(640, 480));
  cv::imshow(title_name, bgr_img);
  cv::waitKey(1);
}

struct ImgSubscriber {
  std::string camera_name;
  static void onSampleReceivedCallback(
      iox::popo::Subscriber<BgrImgBuf>* subscriber,
      ImgSubscriber* camera_subscriber) {
    static auto sub_cam_name = camera_subscriber->camera_name;
    subscriber->take()
        .and_then([&](auto& sample) {
          showImg((void*)(sample->buf.c_str()), sub_cam_name);
          auto finish_time =
              std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::steady_clock::now().time_since_epoch())
                  .count();
          std::cout << "[" << sub_cam_name << "] "
                    << "cost time from sending to showing image: "
                    << finish_time - sample->send_ts << std::endl;
        })
        .or_else([](auto& result) {
          if (result != iox::popo::ChunkReceiveResult::NO_CHUNK_AVAILABLE) {
            std::cout << "Error receiving chunk." << std::endl;
          }
        });
  }
};

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "No args given, Usage E.g: ./dma_recevier camera_0"
              << std::endl;
    return (EXIT_FAILURE);
  }

  std::string sub_camera_name = argv[1];
  std::cout << "subscribe to camera: " << sub_camera_name << std::endl;

  char APP_NAME[50] = "camera-subscriber";
  /* App Name Tag: "camera-publisher-" + @CAM_NAME@ */
  std::strcat(APP_NAME, sub_camera_name.c_str());

  std::cout << "Create iceoryx app: " << APP_NAME << std::endl;
  iox::runtime::PoshRuntime::initRuntime(APP_NAME);

  iox::popo::SubscriberOptions subscriberoptions;
  subscriberoptions.queueCapacity = 0;
  subscriberoptions.historyRequest = 0;
  subscriberoptions.requiresPublisherHistorySupport = false;
  subscriberoptions.nodeName = APP_NAME;
  // subscriberoptions.nodeName = "sub_node";
  subscriberoptions.queueFullPolicy =
      iox::popo::QueueFullPolicy::DISCARD_OLDEST_DATA;

  iox::cxx::string<50> channel_name;
  channel_name.unsafe_assign(sub_camera_name.c_str());

  iox::popo::Subscriber<BgrImgBuf> subscriber(
      {"camera", channel_name, "channel"});

  std::cout << "subscribe to channel: " << channel_name << std::endl;

  iox::popo::Listener listener;
  ImgSubscriber img_subscriber;
  img_subscriber.camera_name = sub_camera_name;

  listener
      .attachEvent(subscriber, iox::popo::SubscriberEvent::DATA_RECEIVED,
                   iox::popo::createNotificationCallback(
                       ImgSubscriber::onSampleReceivedCallback, img_subscriber))
      .or_else([](auto) {
        std::cerr << "unable to attach subscriberLeft" << std::endl;
        std::exit(EXIT_FAILURE);
      });

  iox::posix::waitForTerminationRequest();

  listener.detachEvent(subscriber, iox::popo::SubscriberEvent::DATA_RECEIVED);

  return (EXIT_SUCCESS);
}
