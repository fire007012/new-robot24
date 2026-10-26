#pragma once
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <memory>
#include <string>
#include <functional>
#include <queue>
#include <atomic>

using boost::asio::ip::tcp;

class TcpServer {
public:
    using MessageCallback = std::function<void(const std::string&)>;
    using ConnectionCallback = std::function<void()>;

    TcpServer(boost::asio::io_context& io_context, int port);
    ~TcpServer();

    void start();
    void stop();
    void sendMessage(const std::string& json_message);
    void setMessageCallback(MessageCallback callback);
    void setConnectionCallback(ConnectionCallback callback);

    bool isConnected() const { return connected_; }

private:
    void startAccept();
    void handleAccept(std::shared_ptr<tcp::socket> socket,
                      const boost::system::error_code& error);
    void startRead(std::shared_ptr<tcp::socket> socket);
    void handleRead(std::shared_ptr<tcp::socket> socket,
                    const boost::system::error_code& error,
                    size_t bytes_transferred);
    void startWrite();
    void handleWrite(std::shared_ptr<tcp::socket> socket,
                     const boost::system::error_code& error);
    void closeClientSocket(std::shared_ptr<tcp::socket> socket);
    void handleDisconnect(std::shared_ptr<tcp::socket> socket,
                          const std::string& reason);

    boost::asio::io_context& io_context_;
    boost::asio::strand<boost::asio::io_context::executor_type> strand_;
    tcp::acceptor acceptor_;
    std::shared_ptr<tcp::socket> client_socket_;
    std::queue<std::string> send_queue_;
    boost::asio::streambuf receive_buffer_;
    MessageCallback message_callback_;
    ConnectionCallback connection_callback_;
    std::atomic<bool> connected_;
    int port_;

    static constexpr size_t MAX_MESSAGE_SIZE = 65536;
};
