#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <iostream>

using boost::asio::ip::tcp;

class PointCloudSender {
public:
    PointCloudSender(boost::asio::io_service& io_service, short port)
        : acceptor_(io_service, tcp::endpoint(tcp::v4(), port)), socket_(io_service) {
        startAccept();
    }

private:
    void startAccept() {
        acceptor_.async_accept(socket_,
            boost::bind(&PointCloudSender::handleAccept, this,
                boost::asio::placeholders::error));
    }

    void handleAccept(const boost::system::error_code& error) {
        if (!error) {
            startSend();
        } else {
            std::cerr << "Accept error: " << error.message() << std::endl;
        }
    }

    void startSend() {
        // 模拟点云数据
        std::string data = "This is a point cloud data";

        // 发送数据大小
        dataSize_ = data.size();
        boost::asio::async_write(socket_,
            boost::asio::buffer(&dataSize_, sizeof(dataSize_)),
            boost::bind(&PointCloudSender::handleWriteDataSize, this,
                boost::asio::placeholders::error, data));
    }

    void handleWriteDataSize(const boost::system::error_code& error, const std::string& data) {
        if (!error) {
            // 发送数据
            boost::asio::async_write(socket_,
                boost::asio::buffer(data),
                boost::bind(&PointCloudSender::handleWriteData, this,
                    boost::asio::placeholders::error));
        } else {
            std::cerr << "Write data size error: " << error.message() << std::endl;
        }
    }

    void handleWriteData(const boost::system::error_code& error) {
        if (!error) {
            // 继续发送下一个数据包
            startSend();
        } else {
            std::cerr << "Write data error: " << error.message() << std::endl;
        }
    }

    tcp::acceptor acceptor_;
    tcp::socket socket_;
    uint32_t dataSize_;
};

int main() {
    try {
        boost::asio::io_service io_service;
        PointCloudSender sender(io_service, 12345);
        io_service.run();
    } catch (std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
    }

    return 0;
}

