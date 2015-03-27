// Copyright 2015 Google, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
//     distributed under the License is distributed on an "AS IS" BASIS,
//     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "viz_client.h"
#include "debug.h"

#include <stdexcept>

#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <netdb.h>

using namespace std;

viz_client::viz_client() : sd(-1) {
}
viz_client::~viz_client() {
  close(sd);
}

void viz_client::connect(const std::string &host, short port) {
}
void viz_client::disconnect() {
}

// Use a UDP broadcast to find a server at port.
string viz_client::find_host(short port, int timeout_ms) {
  dbg(1) << "finding visualization host on port " << port << "..." << endl;
  int sd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sd < 0)
    throw runtime_error(strerror(errno));

  char response[64];
  sockaddr_storage from;
  size_t from_size = sizeof(from);
  try {
    // Set the socket to broadcast.
    int broadcast = 1;
    if (setsockopt(sd, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0)
      throw runtime_error(strerror(errno));
    dbg(2) << "  set socket broadcast=" << broadcast << endl;

    // Set the timeout.
    struct timeval timeout;
    timeout.tv_sec = timeout_ms/1000;
    timeout.tv_usec = (timeout_ms%1000)*1000;
    if (setsockopt (sd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout)) < 0)
      throw runtime_error(strerror(errno));
    dbg(2) << "  set socket timeout=" << timeout_ms << " ms" << endl;

    // Set up the broadcast address.
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof addr);
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(0xFFFFFFFF);
    addr.sin_port = htons(port);

    // Send out a broadcast asking for a host.
    const char *request = "viz_client find_host";
    if (sendto(sd, request, strlen(request), 0, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
      throw runtime_error(strerror(errno));
    dbg(2) << "  broadcasted request='" << request << "'" << endl;

    // Try to receive a response.
    ssize_t response_size = recvfrom(sd, response, sizeof(response) - 1, 0, reinterpret_cast<sockaddr*>(&from), &from_size);
    if (response_size <= 0)
      throw runtime_error(strerror(errno));
    response[response_size] = '\0';

    dbg(2) << "  broadcast response='" << response << "'" << endl;

  } catch(...) {
    close(sd);
    throw;
  }

  close(sd);

  if (strncmp(response, "viz_server here", sizeof(response)) != 0)
    throw runtime_error(std::string("response out of protocol: ") + response);

  char hostname[NI_MAXHOST];
  if (getnameinfo(reinterpret_cast<sockaddr*>(&from), from_size, hostname, sizeof(hostname), NULL, 0, 0) != 0)
    throw runtime_error(strerror(errno));

  return hostname;
}
