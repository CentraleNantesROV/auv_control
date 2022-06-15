#ifndef AUV_CONTROL_SERVICE_SYNC_H
#define AUV_CONTROL_SERVICE_SYNC_H

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

template <class ServiceT>
class ServiceNodeSync
{
  typedef typename ServiceT::Request RequestT;
  typedef typename ServiceT::Response ResponseT;
public:
  ServiceNodeSync(std::string name): node(std::make_shared<rclcpp::Node>(name))
  { }

  void init(std::string service)
  {
    client = node->create_client<ServiceT>(service);
    client->wait_for_service();
  }

  [[maybe_unused]] ResponseT sendRequest(const RequestT &req)
  {
    return sendRequest(std::make_shared<RequestT>(req));
  }

  [[maybe_unused]] ResponseT sendRequest(std::shared_ptr<RequestT> &req_ptr)
  {
    auto result = client->async_send_request(req_ptr);
    rclcpp::spin_until_future_complete(node, result);
    return *result.get();
  }

protected:
  rclcpp::Node::SharedPtr node;
  typename rclcpp::Client<ServiceT>::SharedPtr client;
};

#endif