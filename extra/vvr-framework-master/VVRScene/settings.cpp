#include "settings.h"

#include <string>
#include <map>
#include <cstdio>
#include <iostream>
#include <algorithm>

#define MAX_LINE_LEN 1024

using namespace vvr;
using namespace std;

Settings::Settings(string filename)
{
    FILE *file;
    file = fopen(filename.c_str(), "r");
    if (!file) throw string("Cannot open config file <")+filename+">";

    char Line[MAX_LINE_LEN], Key[128], Val[MAX_LINE_LEN-128];
    while (fgets(Line, MAX_LINE_LEN, file)) {
        if (Line[0] == '#') continue;
        if(sscanf(Line, "%s = %s\n", Key, Val)==2){
            string key(Key);
            transform(key.begin(), key.end(), key.begin(), ::tolower);
            m_map[key] = Val;
        }
    }

    fclose(file);
}

void Settings::set(const string &Key, const string &newVal, bool create_if_not_exist)
{
    string key(Key);
    transform(key.begin(), key.end(), key.begin(), ::tolower);
    if (m_map.find(key) == m_map.end() && create_if_not_exist == false)
        throw string("Setting <") + key + "> not in map.";

    m_map[key] = newVal;
}

bool Settings::exists(const string &key) const {
    return m_map.count(key);
}

void Settings::getKeys(vector<string> &keys) const {
    for(map<string,string>::const_iterator it = m_map.begin(); it != m_map.end(); ++it) {
      keys.push_back(it->first);
    }
}

void Settings::write(const string &filename) const
{
    FILE *file;
    file = fopen(filename.c_str(), "w");

    for(map<string,string>::const_iterator it = m_map.begin(); it != m_map.end(); ++it) {
        fprintf(file, "%s = %s \n", it->first.c_str(), it->second.c_str());
    }

    fclose(file);
}

string Settings::getStr(const string &Key) const {
    string key(Key);
    transform(key.begin(), key.end(), key.begin(), ::tolower);
    if (m_map.find(key) == m_map.end())
        throw string("Setting <") + key + "> not in map.";

    return m_map.at(key);
}

double Settings::getDbl(const string &key) const {
    return atof(getStr(key).c_str());
}

bool Settings::getBool(const string &key) const {
    return getInt(key);
}

int Settings::getInt(const string &key) const {
    return atoi(getStr(key).c_str());
}
