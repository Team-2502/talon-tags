#include <string>
#include "../../HTTPRequest/include/HTTPRequest.hpp"
#include <iostream>

void sendData(std::string endpoint, std::string json) {
    try
    {
        http::Request request{"http://10.25.2.2:8089/" + endpoint};
        const auto response = request.send("POST", json, {
                {"Content-Type", "application/json"}
        });
        std::cout << std::string{response.body.begin(), response.body.end()} << '\n';
    }
    catch (const std::exception& e)
    {
        std::cerr << "Request failed, error: " << e.what() << '\n';
    }
}