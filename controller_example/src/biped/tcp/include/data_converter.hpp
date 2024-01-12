#ifndef _SIMPLE_DATA_CONVERTER_H
#define _SIMPLE_DATA_CONVERTER_H

#include <stdlib.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h> 
#include <iostream>

#include "json.hpp"

int bin2int(const char *int_bin);
std::string int2binstr(int n);

nlohmann::json bin2json(const char *json_bin);
std::string json2binstr(const nlohmann::json &jdata);

#endif