#ifndef CSV_PARAM_LOADER_H
#define CSV_PARAM_LOADER_H

#include <string>
#include <vector>
#include <map>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <sstream>
#include "utils.h"

class CSVParamLoader
{
public:
    CSVParamLoader(const std::string &filename, const char delimiter = ',');

    bool LoadFromFile(const std::string &filename);
    bool reload();
    bool HasParameter(const std::string &paramName) const;

    Eigen::VectorXd GetParameter(const std::string &paramName) const;
    Eigen::VectorXd GetParameter(int index) const;
    int size();

private:
    std::map<std::string, Eigen::VectorXd> params_;
    char delimiter_;
    std::vector<std::string> params_names;
    std::string filename_;
};

class CsvLogLoader
{
public:
    CsvLogLoader(const std::string &filename, char delimiter = ',');

    bool LoadFromFile(const std::string &filename);
    bool Reload();

    bool HasVariable(const std::string &variableName) const;
    std::vector<Eigen::VectorXd> GetVariable(const std::string &variableName) const;
    Eigen::VectorXd GetVariableByIndex(const std::string &variableName, uint64_t index) const;

    std::vector<std::string> getNames();

private:
    std::map<std::string, std::vector<Eigen::VectorXd>> data_;
    char delimiter_;
    std::string filename_;
    std::vector<std::string> variableOrder_;
    std::map<std::string, std::string> baseNames_map;
    std::map<std::string, uint64_t> value_num_map;

    bool ParseHeader(const std::string &headerLine);
    void ParseDataLine(const std::string &dataLine);
};

#endif // CSV_PARAM_LOADER_H
