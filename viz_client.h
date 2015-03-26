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
