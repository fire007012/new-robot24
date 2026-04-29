#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>

#include <algorithm>
#include <cerrno>
#include <csignal>
#include <cctype>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <utility>
#include <vector>

namespace {

struct Config {
  std::string image_topic;
  std::string rtsp_url;
  std::string ffmpeg_path = "ffmpeg";
  std::string output_codec = "h264";
  std::string rtsp_transport = "tcp";
  int fps = 30;
  int bitrate_kbps = 2500;
};

std::string trim(const std::string& value) {
  const auto begin = std::find_if_not(value.begin(), value.end(), [](unsigned char ch) {
    return std::isspace(ch);
  });
  const auto end = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char ch) {
    return std::isspace(ch);
  }).base();
  if (begin >= end) {
    return "";
  }
  return std::string(begin, end);
}

std::string stripInlineComment(const std::string& value) {
  bool in_single_quote = false;
  bool in_double_quote = false;
  for (size_t i = 0; i < value.size(); ++i) {
    const char ch = value[i];
    if (ch == '\'' && !in_double_quote) {
      in_single_quote = !in_single_quote;
    } else if (ch == '"' && !in_single_quote) {
      in_double_quote = !in_double_quote;
    } else if (ch == '#' && !in_single_quote && !in_double_quote) {
      return value.substr(0, i);
    }
  }
  return value;
}

std::string unquote(std::string value) {
  value = trim(stripInlineComment(value));
  if (value.size() >= 2) {
    const char first = value.front();
    const char last = value.back();
    if ((first == '"' && last == '"') || (first == '\'' && last == '\'')) {
      return value.substr(1, value.size() - 2);
    }
  }
  return value;
}

std::string toLower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return value;
}

int parsePositiveInt(const std::string& key, const std::string& value) {
  try {
    const int parsed = std::stoi(value);
    if (parsed <= 0) {
      throw std::invalid_argument("must be positive");
    }
    return parsed;
  } catch (const std::exception&) {
    throw std::runtime_error(key + " must be a positive integer");
  }
}

Config loadConfig(const std::string& path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("cannot open config: " + path);
  }

  Config config;
  std::string line;
  int line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    line = trim(stripInlineComment(line));
    if (line.empty() || line[0] == '#') {
      continue;
    }

    const size_t colon = line.find(':');
    if (colon == std::string::npos) {
      throw std::runtime_error("invalid YAML line " + std::to_string(line_number) + ": " + line);
    }

    const std::string key = trim(line.substr(0, colon));
    const std::string value = unquote(line.substr(colon + 1));
    if (key == "image_topic") {
      config.image_topic = value;
    } else if (key == "rtsp_url") {
      config.rtsp_url = value;
    } else if (key == "ffmpeg_path") {
      config.ffmpeg_path = value.empty() ? config.ffmpeg_path : value;
    } else if (key == "output_codec") {
      config.output_codec = value.empty() ? config.output_codec : value;
    } else if (key == "rtsp_transport") {
      config.rtsp_transport = value;
    } else if (key == "fps") {
      config.fps = parsePositiveInt(key, value);
    } else if (key == "bitrate_kbps") {
      config.bitrate_kbps = parsePositiveInt(key, value);
    } else {
      ROS_WARN("ignoring unknown config key: %s", key.c_str());
    }
  }

  if (config.image_topic.empty()) {
    throw std::runtime_error("image_topic is required");
  }
  if (config.rtsp_url.empty()) {
    throw std::runtime_error("rtsp_url is required");
  }
  return config;
}

bool looksLikeJpegFormat(const std::string& format) {
  const std::string lower = toLower(format);
  return lower.find("jpeg") != std::string::npos ||
         lower.find("jpg") != std::string::npos ||
         lower.find("mjpeg") != std::string::npos;
}

bool writeAll(int fd, const uint8_t* data, size_t size) {
  size_t written = 0;
  while (written < size) {
    const ssize_t ret = ::write(fd, data + written, size - written);
    if (ret > 0) {
      written += static_cast<size_t>(ret);
      continue;
    }
    if (ret < 0 && errno == EINTR) {
      continue;
    }
    return false;
  }
  return true;
}

std::vector<std::string> buildFfmpegCommand(const Config& config) {
  const int safe_fps = std::max(config.fps, 1);

  std::vector<std::string> command = {
      config.ffmpeg_path,
      "-hide_banner",
      "-loglevel",
      "warning",
      "-fflags",
      "nobuffer",
      "-f",
      "mjpeg",
      "-r",
      std::to_string(safe_fps),
      "-i",
      "pipe:0",
      "-an",
  };

  const std::string codec = toLower(config.output_codec);
  if (codec == "copy" || codec == "passthrough") {
    command.insert(command.end(), {"-c:v", "copy"});
  } else if (codec == "h264" || codec == "libx264") {
    command.insert(command.end(), {
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        "-profile:v",
        "baseline",
        "-preset",
        "ultrafast",
        "-tune",
        "zerolatency",
        "-g",
        std::to_string(safe_fps),
        "-keyint_min",
        std::to_string(safe_fps),
        "-sc_threshold",
        "0",
        "-x264-params",
        "repeat-headers=1",
        "-b:v",
        std::to_string(std::max(config.bitrate_kbps, 1)) + "k",
    });
  } else {
    command.insert(command.end(), {
        "-c:v",
        config.output_codec,
        "-b:v",
        std::to_string(std::max(config.bitrate_kbps, 1)) + "k",
    });
  }

  if (!config.rtsp_transport.empty()) {
    command.insert(command.end(), {"-rtsp_transport", config.rtsp_transport});
  }
  command.insert(command.end(), {"-f", "rtsp", config.rtsp_url});
  return command;
}

std::string joinCommand(const std::vector<std::string>& command) {
  std::string result;
  for (const auto& part : command) {
    if (!result.empty()) {
      result += " ";
    }
    result += part;
  }
  return result;
}

}  // namespace

class RosImageRtspNode {
 public:
  explicit RosImageRtspNode(Config config) : config_(std::move(config)) {}

  ~RosImageRtspNode() {
    stopFfmpeg();
  }

  void start() {
    subscriber_ = nh_.subscribe(
        config_.image_topic,
        1,
        &RosImageRtspNode::onImage,
        this,
        ros::TransportHints().tcpNoDelay());

    ROS_INFO("ros_image_rtsp_node subscribing %s -> %s",
             config_.image_topic.c_str(),
             config_.rtsp_url.c_str());
  }

 private:
  bool ensureFfmpegRunning() {
    if (child_pid_ > 0) {
      int status = 0;
      const pid_t ret = ::waitpid(child_pid_, &status, WNOHANG);
      if (ret == 0) {
        return stdin_fd_ >= 0;
      }
      if (ret == child_pid_) {
        ROS_WARN("ffmpeg exited, status=%d", status);
      }
      closePipe();
      child_pid_ = -1;
    }

    return startFfmpeg();
  }

  bool startFfmpeg() {
    int pipe_fds[2] = {-1, -1};
    if (::pipe(pipe_fds) != 0) {
      ROS_ERROR("pipe failed: %s", std::strerror(errno));
      return false;
    }

    const std::vector<std::string> command = buildFfmpegCommand(config_);

    const pid_t pid = ::fork();
    if (pid < 0) {
      ROS_ERROR("fork failed: %s", std::strerror(errno));
      ::close(pipe_fds[0]);
      ::close(pipe_fds[1]);
      return false;
    }

    if (pid == 0) {
      ::close(pipe_fds[1]);
      if (::dup2(pipe_fds[0], STDIN_FILENO) < 0) {
        _exit(127);
      }
      ::close(pipe_fds[0]);

      std::vector<char*> argv;
      argv.reserve(command.size() + 1);
      for (const auto& item : command) {
        argv.push_back(const_cast<char*>(item.c_str()));
      }
      argv.push_back(nullptr);
      ::execvp(argv[0], argv.data());
      _exit(127);
    }

    ::close(pipe_fds[0]);
    stdin_fd_ = pipe_fds[1];
    child_pid_ = pid;
    ROS_INFO("started ffmpeg pid=%d: %s", static_cast<int>(pid), joinCommand(command).c_str());
    return true;
  }

  void onImage(const sensor_msgs::CompressedImageConstPtr& msg) {
    if (msg->data.empty()) {
      return;
    }
    if (!looksLikeJpegFormat(msg->format)) {
      if (!warned_format_) {
        ROS_WARN("dropping non-JPEG compressed image format=%s", msg->format.c_str());
        warned_format_ = true;
      }
      return;
    }
    if (!ensureFfmpegRunning()) {
      return;
    }
    if (!writeAll(stdin_fd_, msg->data.data(), msg->data.size())) {
      ROS_WARN("failed to write image to ffmpeg stdin: %s", std::strerror(errno));
      stopFfmpeg();
    }
  }

  void closePipe() {
    if (stdin_fd_ >= 0) {
      ::close(stdin_fd_);
      stdin_fd_ = -1;
    }
  }

  void stopFfmpeg() {
    closePipe();
    if (child_pid_ <= 0) {
      return;
    }
    int status = 0;
    if (::waitpid(child_pid_, &status, WNOHANG) == 0) {
      ::kill(child_pid_, SIGTERM);
      for (int i = 0; i < 10; ++i) {
        if (::waitpid(child_pid_, &status, WNOHANG) == child_pid_) {
          child_pid_ = -1;
          return;
        }
        ::usleep(100000);
      }
      ::kill(child_pid_, SIGKILL);
      ::waitpid(child_pid_, &status, 0);
    }
    child_pid_ = -1;
  }

  Config config_;
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  bool warned_format_ = false;

  pid_t child_pid_ = -1;
  int stdin_fd_ = -1;
};

int main(int argc, char** argv) {
  ::signal(SIGPIPE, SIG_IGN);
  ros::init(argc, argv, "ros_image_rtsp_node");

  std::string config_path;
  ros::NodeHandle private_nh("~");
  private_nh.param<std::string>("config", config_path, "");
  if (argc > 1 && std::string(argv[1]).find(":=") == std::string::npos) {
    config_path = argv[1];
  }
  if (config_path.empty()) {
    ROS_ERROR("usage: rosrun ros1_bridge ros_image_rtsp_node /path/to/ros_image_rtsp.yaml");
    return 1;
  }

  try {
    RosImageRtspNode node(loadConfig(config_path));
    node.start();
    ros::spin();
  } catch (const std::exception& exc) {
    ROS_ERROR("%s", exc.what());
    return 1;
  }

  return 0;
}
