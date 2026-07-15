#ifndef PARAM_MANAGER_H
#define PARAM_MANAGER_H

#include <yaml-cpp/yaml.h>
#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <algorithm>

template<class T>
struct Matrix2D {
    std::vector<std::vector<T>> data;

    bool operator==(const Matrix2D<T>& other) const {
        return other.data.size() == data.size() &&
               (data.empty() || other.data[0].size() == data[0].size());
    }

    friend std::ostream& operator<<(std::ostream& os, const Matrix2D<T>& d) {
        for (const auto& row : d.data) {
            for (const T& value : row) {
                os << value << " ";
            }
            os << "\n";
        }
        return os;
    }
};

// Triangle structure to hold 3 vertices with x,y coordinates
struct Triangle2D {
    std::array<std::array<float, 2>, 3> vertices; // 3 vertices, each with x,y coordinates

    Triangle2D() = default;
    Triangle2D(const std::array<std::array<float, 2>, 3>& verts) : vertices(verts) {}

    bool operator==(const Triangle2D& other) const {
        return vertices == other.vertices;
    }

    friend std::ostream& operator<<(std::ostream& os, const Triangle2D& triangle) {
        os << "Triangle: [";
        for (size_t i = 0; i < 3; ++i) {
            os << "[" << triangle.vertices[i][0] << ", " << triangle.vertices[i][1] << "]";
            if (i < 2) os << ", ";
        }
        os << "]";
        return os;
    }
};

namespace YAML {
    template<class T>
    struct convert<Matrix2D<T>> {
    static Node encode(const Matrix2D<T>& rhs) {
        Node node;
        for (const auto& d : rhs.data) {
            node.push_back(d);
        }
        return node;
    }



    static bool decode(const Node& node, Matrix2D<T>& rhs) {
        if (!node.IsSequence()) return false;
        rhs.data.clear();
        for (const auto& temp : node) {
            if (!temp.IsSequence()) return false;
            std::vector<T> cols;
            for (const auto& item : temp) {
                cols.push_back(item.as<T>());
            }
            rhs.data.push_back(cols);
        }
        return true;
    }
};
}


namespace YAML {
    template<>
    struct convert<Triangle2D> {
        static Node encode(const Triangle2D& rhs) {
            Node node;
            for (size_t i = 0; i < 3; ++i) {
                Node vertex;
                vertex.push_back(rhs.vertices[i][0]);
                vertex.push_back(rhs.vertices[i][1]);
                node.push_back(vertex);
            }
            return node;
        }

        static bool decode(const Node& node, Triangle2D& rhs) {
            if (!node.IsSequence() || node.size() != 3) {
                return false;
            }

            try {
                for (size_t i = 0; i < 3; ++i) {
                    if (!node[i].IsSequence() || node[i].size() != 2) {
                        return false;
                    }
                    rhs.vertices[i][0] = node[i][0].as<float>();
                    rhs.vertices[i][1] = node[i][1].as<float>();
                }
                return true;
            } catch (const YAML::Exception&) {
                return false;
            }
        }
    };
}

class param_manager : public std::enable_shared_from_this<param_manager> {
public:
    explicit param_manager(const std::string& file) : config_(YAML::LoadFile(file)) {}

    bool has_param(const std::string& field) const {
        return config_[field].IsDefined();
    }

    template<class T>
    T get_param(const std::string& field) {

        if(has_param(field))
            return config_[field].template as<T>();
        std::cerr << "Error: Parameter '" << field << "' not found in configuration." << std::endl;
        throw std::runtime_error("Parameter '" + field + "' not found in configuration.");
    }

    template<class T>
    T get_param(const std::string& field1, const std::string& field2) {
        return config_[field1][field2].template as<T>();
    }

    template<class T>
    T get_param(const std::string& field1, const std::string& field2, const std::string& field3) {
        return config_[field1][field2][field3].template as<T>();
    }

    template<class T>
    void get_obstacles(T& result) {
        Matrix2D<float> obstacles = config_["obstacles"].as<Matrix2D<float>>();
        std::copy(obstacles.data.begin(), obstacles.data.end(), std::back_inserter(result));
    }

    template<class T>
    std::vector<std::vector<T>> get_ndarray(const std::string& field) {
        auto result = config_[field].as<Matrix2D<T>>();
        return result.data;
    }

    std::shared_ptr<param_manager> getSharedPtr() {
        return shared_from_this();
    }

    std::vector<Triangle2D> get_triangles(const std::string& field="triangles") {
        std::vector<Triangle2D> triangles;

        try {
            if (!config_[field]) {
                throw std::runtime_error("Field '" + field + "' not found in YAML configuration");
            }

            const YAML::Node& triangles_node = config_[field];

            if (!triangles_node.IsSequence()) {
                throw std::runtime_error("Field '" + field + "' is not a sequence");
            }

            triangles.reserve(triangles_node.size());

            for (size_t i = 0; i < triangles_node.size(); ++i) {
                try {
                    Triangle2D triangle = triangles_node[i].as<Triangle2D>();
                    triangles.push_back(triangle);
                } catch (const YAML::Exception& e) {
                    std::cerr << "Warning: Failed to parse triangle " << i << ": " << e.what() << std::endl;
                    continue; // Skip invalid triangles
                }
            }

            std::cout << "Successfully loaded " << triangles.size() << " triangles from field '" << field << "'" << std::endl;

        } catch (const std::exception& e) {
            std::cerr << "Error reading triangles: " << e.what() << std::endl;
            throw;
        }

        return triangles;
    }



private:
    YAML::Node config_;
};

using ParamPtr = std::shared_ptr<param_manager>;

#endif // PARAM_MANAGER_H
