#ifndef SYNAPSE_GZ_PROTO_UDP_LINK_HPP__
#define SYNAPSE_GZ_PROTO_UDP_LINK_HPP__

#include <boost/asio.hpp>
#include <boost/asio/signal_set.hpp>
#include <boost/date_time/posix_time/posix_time_config.hpp>

class GzClient;

#define CEREBRI_PORT 4243
#define GZ_PORT 4241

class UDPLink {
private:
    static const uint32_t rx_buf_length_ = 1024;
    std::mutex guard_rx_buf_;
    uint8_t rx_buf_[rx_buf_length_];
    boost::asio::io_context io_context_ {};
    boost::asio::ip::udp::socket sock_ {
        boost::asio::ip::udp::socket(io_context_,
            boost::asio::ip::udp::endpoint(
                boost::asio::ip::udp::v4(), GZ_PORT))
    };
    boost::asio::ip::udp::endpoint remote_endpoint_;
    boost::asio::ip::udp::endpoint my_endpoint_;
    GzClient* gz_;

public:
    UDPLink(std::string host, int port, GzClient* gz);
    void run_for(std::chrono::seconds sec);
    void write(const uint8_t* buf, uint32_t len);

private:
    void timeout_handler();
    void tx_handler(const boost::system::error_code& error, std::size_t bytes_transferred);
    void rx_handler(const boost::system::error_code& error, std::size_t bytes_transferred);
};

// vi: ts=4 sw=4 et

#endif // SYNAPSE_GZ_PROTO_UDP_LINK_HPP__
