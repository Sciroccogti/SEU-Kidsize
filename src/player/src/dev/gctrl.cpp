#include "gctrl.hpp"
#include <ros/ros.h>

using namespace std;
using boost::asio::ip::udp;

boost::asio::io_service gc_service;

GameCtrl::GameCtrl():
    recv_socket_ (gc_service, udp::endpoint(udp::v4(), GAMECONTROLLER_DATA_PORT))
{

}

bool GameCtrl::start()
{
    td_ = std::move(std::thread([this]()
    {
        this->receive();
        gc_service.run();
    }));
    return true;
}

void GameCtrl::receive()
{
    if(!ros::ok()) return;
    recv_socket_.async_receive_from(boost::asio::buffer((char *)&data_, gc_data_size), recv_point_,
                                    [this](boost::system::error_code ec, std::size_t bytes_recvd)
    {
        if (!ec && bytes_recvd > 0)
        {
            string recv_header;
            recv_header.append(data_.header, sizeof(RoboCupGameControlData::header));
            if (recv_header == GAMECONTROLLER_STRUCT_HEADER)
            {
                
            }
        }
        receive();
    });
}

void GameCtrl::stop()
{
    recv_socket_.cancel();
    recv_socket_.close();
    gc_service.stop();
}

GameCtrl::~GameCtrl()
{
    if (td_.joinable())
    {
        td_.join();
    }
}