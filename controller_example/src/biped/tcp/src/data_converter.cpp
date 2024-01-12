#include "data_converter.hpp"


using namespace nlohmann;
using namespace std;


int bin2int(const char *int_bin)
{
    return ntohl(*(int32_t *)int_bin);
}

std::string int2binstr(int n)
{
    int32_t n_int32 = htonl(n);
    return string((char *)&n_int32, 4);
}

json bin2json(const char *json_bin)
{
    return json::parse(json_bin);
}

std::string json2binstr(const nlohmann::json &jdata)
{
    return jdata.dump();
}