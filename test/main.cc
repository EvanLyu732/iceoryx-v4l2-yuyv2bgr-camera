#include <cstdio>

#include "ShmImgBuf.h"
#include "iceoryx_posh/popo/subscriber.hpp"
//! [include subscriber]
#include <chrono>
#include <iceoryx_posh/popo/subscriber_options.hpp>
#include <opencv2/opencv.hpp>

#include "iceoryx_hoofs/posix_wrapper/signal_watcher.hpp"
#include "iceoryx_posh/popo/listener.hpp"
#include "iceoryx_posh/runtime/posh_runtime.hpp"

constexpr char APP_NAME[] = "iox-cpp-subscriber";

void showImg(void* img_buf) {
  cv::Mat bgr_img(1080, 1920, CV_8UC3, img_buf);
  cv::imshow("output_img", bgr_img);
  cv::waitKey(1);
}

void onSampleReceivedCallback(iox::popo::Subscriber<BgrImgBuf>* subscriber) {
  subscriber->take()
      .and_then([](auto& sample) {
        // showImg((void*)(sample->buf.c_str()));
        auto finish_time =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count();
        std::cout << "cost time from sending to showing image: "
                  << finish_time - sample->send_ts << std::endl;
      })
      .or_else([](auto& result) {
        if (result != iox::popo::ChunkReceiveResult::NO_CHUNK_AVAILABLE) {
          std::cout << "Error receiving chunk." << std::endl;
        }
      });
}

int main() {
  iox::runtime::PoshRuntime::initRuntime(APP_NAME);

  iox::popo::SubscriberOptions subscriberoptions;
  subscriberoptions.queueCapacity = 1;
  subscriberoptions.historyRequest = 0;
  subscriberoptions.requiresPublisherHistorySupport = false;
  subscriberoptions.nodeName = "sub_node";
  subscriberoptions.queueFullPolicy =
      iox::popo::QueueFullPolicy::DISCARD_OLDEST_DATA;

  iox::popo::Subscriber<BgrImgBuf> subscriber(
      {"Radar1", "FrontLeft", "Object"});

  iox::popo::Listener listener;

  listener
      .attachEvent(
          subscriber, iox::popo::SubscriberEvent::DATA_RECEIVED,
          iox::popo::createNotificationCallback(onSampleReceivedCallback))
      .or_else([](auto) {
        std::cerr << "unable to attach subscriberLeft" << std::endl;
        std::exit(EXIT_FAILURE);
      });

  iox::posix::waitForTerminationRequest();

  listener.detachEvent(subscriber, iox::popo::SubscriberEvent::DATA_RECEIVED);

  return (EXIT_SUCCESS);
}
