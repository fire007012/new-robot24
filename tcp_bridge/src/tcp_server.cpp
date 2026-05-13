#include "tcp_bridge/tcp_server.h"
#include <ros/ros.h>

TcpServer::TcpServer(boost::asio::io_context& io_context, int port)
    : io_context_(io_context),
      strand_(boost::asio::make_strand(io_context_)),
      acceptor_(io_context, tcp::endpoint(tcp::v4(), port)),
      receive_buffer_(MAX_MESSAGE_SIZE),
      connected_(false),
      port_(port) {
}

TcpServer::~TcpServer() {
    stop();
}

void TcpServer::start() {
    ROS_INFO_STREAM("[TCP] Server listening on port " << port_);
    startAccept();
}

void TcpServer::stop() {
    connected_ = false;
    if (client_socket_ && client_socket_->is_open()) {
        boost::system::error_code ec;
        client_socket_->close(ec);
    }
    acceptor_.close();
}

void TcpServer::setMessageCallback(MessageCallback callback) {
    message_callback_ = std::move(callback);
}

void TcpServer::setConnectionCallback(ConnectionCallback callback) {
    connection_callback_ = std::move(callback);
}

void TcpServer::startAccept() {
    auto socket = std::make_shared<tcp::socket>(io_context_);
    acceptor_.async_accept(*socket,
        boost::asio::bind_executor(
            strand_,
            boost::bind(&TcpServer::handleAccept, this, socket,
                        boost::asio::placeholders::error)));
}

void TcpServer::handleAccept(std::shared_ptr<tcp::socket> socket,
                              const boost::system::error_code& error) {
    if (!error) {
        ROS_INFO_STREAM("[TCP] Client connected: "
                        << socket->remote_endpoint().address().to_string());

        if (client_socket_ && client_socket_->is_open()) {
            closeClientSocket(client_socket_);
        }

        client_socket_ = socket;
        connected_ = true;
        receive_buffer_.consume(receive_buffer_.size());
        std::queue<std::string> empty;
        send_queue_.swap(empty);

        startRead(socket);

        if (connection_callback_) {
            connection_callback_();
        }
    }

    startAccept();
}

void TcpServer::startRead(std::shared_ptr<tcp::socket> socket) {
    boost::asio::async_read_until(*socket, receive_buffer_, '\n',
        boost::asio::bind_executor(
            strand_,
            boost::bind(&TcpServer::handleRead, this, socket,
                        boost::asio::placeholders::error,
                        boost::asio::placeholders::bytes_transferred)));
}

void TcpServer::handleRead(std::shared_ptr<tcp::socket> socket,
                            const boost::system::error_code& error,
                            size_t /*bytes_transferred*/) {
    if (socket != client_socket_) {
        return;
    }

    if (!error) {
        std::istream is(&receive_buffer_);
        std::string message;
        std::getline(is, message);

        if (!message.empty() && message_callback_) {
            message_callback_(message);
        }

        startRead(socket);
    } else {
        if (error == boost::asio::error::not_found &&
            receive_buffer_.size() >= MAX_MESSAGE_SIZE) {
            handleDisconnect(socket, "incoming message exceeded maximum size");
        } else if (error != boost::asio::error::operation_aborted) {
            handleDisconnect(socket, "read error: " + error.message());
        }
    }
}

void TcpServer::sendMessage(const std::string& json_message) {
    boost::asio::post(strand_, [this, json_message]() {
        if (!connected_ || !client_socket_ || !client_socket_->is_open()) {
            return;
        }

        bool write_in_progress = !send_queue_.empty();
        send_queue_.push(json_message + "\n");

        if (!write_in_progress) {
            startWrite();
        }
    });
}

void TcpServer::startWrite() {
    if (send_queue_.empty() || !client_socket_ || !client_socket_->is_open()) {
        return;
    }

    const std::string& message = send_queue_.front();
    boost::asio::async_write(*client_socket_,
        boost::asio::buffer(message),
        boost::asio::bind_executor(
            strand_,
            boost::bind(&TcpServer::handleWrite, this, client_socket_,
                        boost::asio::placeholders::error)));
}

void TcpServer::handleWrite(std::shared_ptr<tcp::socket> socket,
                            const boost::system::error_code& error) {
    if (socket != client_socket_) {
        return;
    }

    if (!error) {
        if (!send_queue_.empty()) {
            send_queue_.pop();
        }

        if (!send_queue_.empty()) {
            startWrite();
        }
    } else {
        if (error != boost::asio::error::operation_aborted) {
            handleDisconnect(socket, "write error: " + error.message());
        }
    }
}

void TcpServer::closeClientSocket(std::shared_ptr<tcp::socket> socket) {
    if (!socket || !socket->is_open()) {
        return;
    }

    boost::system::error_code ec;
    socket->shutdown(tcp::socket::shutdown_both, ec);
    socket->close(ec);
}

void TcpServer::handleDisconnect(std::shared_ptr<tcp::socket> socket,
                                 const std::string& reason) {
    if (socket != client_socket_) {
        return;
    }

    ROS_WARN_STREAM("[TCP] " << reason);
    connected_ = false;
    closeClientSocket(socket);
    if (socket == client_socket_) {
        client_socket_.reset();
    }

    receive_buffer_.consume(receive_buffer_.size());
    if (!send_queue_.empty()) {
        std::queue<std::string> empty;
        send_queue_.swap(empty);
    }
}
