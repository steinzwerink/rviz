#include <iostream>
#include <vector>
#include <string>
#include <thread>

namespace client
{
class client
{
  public:
    client();
    virtual ~client();
    std::vector<uint64_t> parseMessage(std::string message);
};
} // namespace client