#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <map>

class CsvWriter {
private:
    std::string file_name;
    std::string file_path;
    std::vector<std::string> columns;

    std::ofstream file_stream;

    const static std::string end_row;
    const static std::string column_separator;

    std::string build_full_path();

public:
    CsvWriter(std::string fileName, std::string filePath, std::vector<std::string> columns);

    void writeTitle(const std::string& title);

    void writeRow(const std::map<std::string, std::string> &values);
};