#ifndef _AETHER_APP_FILE_PATH_FINDER_HPP_
#define _AETHER_APP_FILE_PATH_FINDER_HPP_

#include <fstream>
#include <sstream>
#include <string>

#include "aether/path_finder.hpp"
#include "aether/robot_config.hpp"
#include "aether/types.hpp"

class FilePathFinder : public PathFinder<FilePathFinder> {
public:
    FilePathFinder() {}

    void load_file(const std::string &path_path) {
        std::ifstream path_file(path_path);
        if (!path_file.is_open()) {
            throw std::runtime_error("Could not open path file");
        }

        uint32_t i = 0;
        while (path_file) {
            std::string line;
            std::getline(path_file, line);
            if (line.empty()) {
                continue;
            }
            std::stringstream ss(line);
            float x, y;
            ss >> x >> y;
            path[i++] = {x, y};
        }
    }

    void find_path(const CellCoords &start, const CellCoords &goal) {
        (void)start;
        (void)goal;
    }
};

#endif // _AETHER_APP_FILE_PATH_FINDER_HPP_
