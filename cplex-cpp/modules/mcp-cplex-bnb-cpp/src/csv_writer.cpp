
#include <string>
#include <vector>
#include <include/csv_writer.h>

const std::string CsvWriter::end_row = "\n";
const std::string CsvWriter::column_separator = ",";

CsvWriter::CsvWriter(std::string fileName, std::string filePath,
                     std::vector<std::string> columns) : file_name(std::move(fileName)), file_path(std::move(filePath)),
                                                         columns(std::move(columns)) {
    file_stream.open(this->build_full_path(), std::ios::app);

    for (const std::string &column: this->columns) {
        file_stream << column << column_separator;
    }
    file_stream << end_row;

    file_stream.close();
}

void CsvWriter::writeRow(const std::map<std::string, std::string> &values) {
    file_stream.open(this->build_full_path(), std::ios::app);

    for (const std::string &column: this->columns) {
        try {
            const std::string &column_value = values.at(column);
            file_stream << column_value << column_separator;
        } catch (std::out_of_range &e) {
            std::cout << e.what() << std::endl;
            file_stream << column_separator;
        }
    }
    file_stream << end_row;

    file_stream.close();
}

std::string CsvWriter::build_full_path() {
    return this->file_path + "/" + this->file_name;
}

void CsvWriter::writeTitle(const std::string &title) {
    file_stream.open(this->build_full_path(), std::ios::app);
    file_stream << title;
    file_stream << end_row;
    file_stream.close();
}
