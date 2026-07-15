#ifndef COLLISIONCHECKER_H
#define COLLISIONCHECKER_H

#include <memory>
#include <vector>
#include <Eigen/Dense>
#include "common/ParamManager.h"

// Forward declarations
class BaseCollisionChecker;
using CCPtr = std::shared_ptr<BaseCollisionChecker>;

class BaseCollisionChecker : public std::enable_shared_from_this<BaseCollisionChecker> {
public:
    explicit BaseCollisionChecker(const ParamPtr& pm) : pm_(pm) {}
    virtual ~BaseCollisionChecker() = default;

    virtual bool isCollision(const std::vector<Eigen::VectorXd>& trajectory) const = 0;

    CCPtr getSharedPtr() { return shared_from_this(); }

protected:
    ParamPtr pm_;
};

// Template for common Pimpl pattern in derived classes
#define DECLARE_PIMPL_CLASS(ClassName, Namespace) \
namespace Namespace { \
    class ClassName : public BaseCollisionChecker { \
    public: \
        explicit ClassName(const ParamPtr &pm); \
        ~ClassName(); \
        bool isCollision(const std::vector<Eigen::VectorXd> &trajectory) const override; \
    private: \
        struct Impl; \
        std::unique_ptr<Impl> pimpl_; \
    }; \
}

// Declare all concrete implementations
DECLARE_PIMPL_CLASS(OccupancyMap, occupancy)


#undef DECLARE_PIMPL_CLASS

#endif // COLLISIONCHECKER_H