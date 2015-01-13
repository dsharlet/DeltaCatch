#ifndef VIZ_CLIENT_H
#define VIZ_CLIENT_H

#include <string>

// Talks to a visualization server.
class viz_client {
public:
  viz_client();
  ~viz_client();

  void connect(const std::string &host, short port);
  void disconnect();

  // Use a UDP broadcast to find a server at port.
  static std::string find_host(short port, int timeout_ms = 3000);

private:
  int sd;
};

#endif