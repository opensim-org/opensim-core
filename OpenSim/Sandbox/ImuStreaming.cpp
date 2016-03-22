#include <sys/socket.h>
#include <netinet/in.h>

#include <iostream>
#include <stdexcept>
#include <cstring>

int main() {
    constexpr short PORT{5555};
    constexpr unsigned BUFFSIZE{8192};

    auto sock = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock < 0)
        throw std::runtime_error{"Could not create Socket."};

    sockaddr_in saddr{};
    std::memset(&saddr, 0, sizeof(saddr));
    saddr.sin_family      = AF_INET;
    saddr.sin_port        = htons(PORT);
    saddr.sin_addr.s_addr = htonl(INADDR_ANY);

    if(bind(sock, (struct sockaddr*)&saddr, sizeof(saddr)) < 0)
        throw std::runtime_error{"Could not bind Socket."};

    size_t row{};
    while(true) {
        unsigned char buffer[BUFFSIZE];
        auto bytes = recvfrom(sock, buffer, BUFFSIZE, 0, 0, 0);

        if(bytes > 0)
            std::cout << "Row " << row << ": " << buffer << std::endl;
        else
            std::cout << "skipping....." << std::endl;
    }

    return 0;
}
