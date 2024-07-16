#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <vector>

using boost::asio::ip::tcp;

class PointCloudReceiver {
public:
    PointCloudReceiver(boost::asio::io_service& io_service, const std::string& host, const std::string& port)
        : resolver_(io_service), socket_(io_service) {
        tcp::resolver::query query(host, port);
        resolver_.async_resolve(query,
            boost::bind(&PointCloudReceiver::handleResolve, this,
                boost::asio::placeholders::error,
                boost::asio::placeholders::iterator));
    }

private:
    void handleResolve(const boost::system::error_code& err, tcp::resolver::iterator endpoint_iterator) {
        if (!err) {
            boost::asio::async_connect(socket_, endpoint_iterator,
                boost::bind(&PointCloudReceiver::handleConnect, this,
                    boost::asio::placeholders::error));
        } else {
            std::cerr << "Resolve error: " << err.message() << std::endl;
        }
    }

    void handleConnect(const boost::system::error_code& err) {
        if (!err) {
            startReceive();
        } else {
            std::cerr << "Connect error: " << err.message() << std::endl;
        }
    }

    void startReceive() {
        boost::asio::async_read(socket_,
            boost::asio::buffer(&dataSize_, sizeof(dataSize_)),
            boost::bind(&PointCloudReceiver::handleReadHeader, this,
                boost::asio::placeholders::error));
    }

    void handleReadHeader(const boost::system::error_code& err) {
        if (!err) {
            data_.resize(dataSize_);
            boost::asio::async_read(socket_,
                boost::asio::buffer(data_),
                boost::bind(&PointCloudReceiver::handleReadData, this,
                    boost::asio::placeholders::error));
        } else {
            std::cerr << "Read header error: " << err.message() << std::endl;
        }
    }

    void handleReadData(const boost::system::error_code& err) {
        if (!err) {
            std::string receivedData(data_.begin(), data_.end());
            std::cout << "Received data: " << receivedData << std::endl;

            // 继续接收下一个数据包
            startReceive();
        } else {
            std::cerr << "Read data error: " << err.message() << std::endl;
        }
    }

    tcp::resolver resolver_;
    tcp::socket socket_;
    uint32_t dataSize_;
    std::vector<char> data_;
};

int main() {
    try {
        boost::asio::io_service io_service;
        PointCloudReceiver receiver(io_service, "127.0.0.1", "12345");
        io_service.run();
    } catch (std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }

    return 0;
}

