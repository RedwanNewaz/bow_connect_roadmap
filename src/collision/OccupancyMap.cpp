//
// Created by airlab on 1/23/26.
//

#include "CollisionChecker.hh"
#include "common/image_parser.h"

namespace occupancy {

struct OccupancyMap::Impl {

    Impl(const ParamPtr& pm) {
        map_resolution_ = pm->get_param<double>("map_resolution");
        threshold_ = pm->get_param<int>("color_threshold");

        auto img_path = pm->get_param<std::string>("map_image_path");
        int dilation_level = pm->get_param<int>("map_inflation");

        image_parser_ = std::make_unique<img_parser::image_parser>(img_path);
        image_parser_->dilateMaze(dilation_level, threshold_);


        auto origin = pm->get_param<std::vector<double>>("origin");

        // Use the origin from the configuration file
        origin_x_ = origin[0];
        origin_y_ = origin[1];
    }

    double getMapResolution() const {
        return map_resolution_;
    }

    int map_width() const {
        return image_parser_->getWidth();
    }

    int map_height() const {
        return image_parser_->getHeight();
    }

    void setOrigin(double origin_x, double origin_y) {
        origin_x_ = origin_x;
        origin_y_ = origin_y;
    }

    std::pair<int, int> worldToMap(double x, double y) const {
        int map_x = static_cast<int>((x - origin_x_) / map_resolution_);
        int map_y = static_cast<int>((y - origin_y_) / map_resolution_);
        return {map_x, map_y};
    }

    std::pair<double, double> mapToWorld(int map_x, int map_y) const {
        double x = map_x * map_resolution_ + origin_x_;
        double y = map_y * map_resolution_ + origin_y_;
        return {x, y};
    }

    bool mapSpaceValid(int map_x, int map_y) const {
        // Check for out-of-bounds
        if (map_x < 0 || map_x >= map_width() || map_y < 0 || map_y >= map_height()) {
            return false;
        }

        auto pixel_value = image_parser_->getPixelValue(map_y, map_x);
        // Assuming black pixels represent obstacles
        return !(pixel_value[0] < threshold_ && pixel_value[1] < threshold_ && pixel_value[2] < threshold_);
    
    }

    bool isWorkspaceCollision(double x, double y) const {
        auto [map_x, map_y] = worldToMap(x, y);
        //        std::cout << "[occupancy] isWorkspaceCollision at world (" << x << ", " << y << ") -> map (" << map_x << ", " << map_y << ")" << std::endl;
        // Check for out-of-bounds
        return mapSpaceValid(map_x, map_y) == false;
    }

private:
    std::unique_ptr<img_parser::image_parser> image_parser_;
    double map_resolution_;
    int threshold_;
    double origin_x_{0.0};
    double origin_y_{0.0};

};

OccupancyMap::OccupancyMap(const ParamPtr &pm)
    : BaseCollisionChecker(pm) {
    pimpl_ = std::make_unique<Impl>(pm);
}

OccupancyMap::~OccupancyMap() = default;

bool OccupancyMap::isCollision(const std::vector<Eigen::VectorXd> &trajectory) const {
    for (int j = trajectory.size(); j-- > 0;) {
        auto state = trajectory[j];
        float wx = state(0);
        float wy = state(1);
        if(pimpl_->isWorkspaceCollision(wx, wy))
        {
            //                std::cout << "[occupancy] Collision at (" << wx << ", " << wy << ")" << std::endl;
            return true;
        }
    }
    return false;
}

} // occupancy