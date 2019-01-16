#ifndef MOCO_CASADIPROBLEM_H
#define MOCO_CASADIPROBLEM_H

#include <string>
#include <unordered_map>

#include <casadi/casadi.hpp>

namespace casocp {

struct Iterate {
    casadi::DM time;
    casadi::DM states;
    casadi::DM controls;
};

struct Solution : public Iterate {

};

class Solver;
class TrapezoidalSolver;

class Problem {
    public:
    struct Bounds {
        double lower;
        double upper;
    };
    struct StateInfo {
        std::string name;
        Bounds bounds;
        Bounds initialBounds;
        Bounds finalBounds;
    };
    struct ControlInfo {
        std::string name;
        Bounds bounds;
        Bounds initialBounds;
        Bounds finalBounds;
    };
    void addState(std::string name, Bounds bounds,
            Bounds initialBounds,
            Bounds finalBounds) {
        m_state_infos[name] = {
                std::move(name),
                std::move(bounds),
                std::move(initialBounds),
                std::move(finalBounds)
        };
    }
    void addControl(std::string name, Bounds bounds,
            Bounds initialBounds,
            Bounds finalBounds) {
        m_control_infos[name] = {
                std::move(name),
                std::move(bounds),
                std::move(initialBounds),
                std::move(finalBounds)
        };
    }
    template <typename FunctionType, typename... Args>
    void setIntegralCost(Args&&... args) {
        m_integral_cost_func =
                std::unique_ptr<FunctionType>(std::forward<Args>(args)...);
    }
    template <typename FunctionType, typename... Args>
    void setEndpointCost(Args&&... args) {
        m_endpoint_cost_func =
                std::unique_ptr<FunctionType>(std::forward<Args>(args)...);
    }
    template <typename FunctionType, typename... Args>
    void setDifferentialEquations(Args&&... args) {
        m_diffeq_func =
                std::unique_ptr<FunctionType>(std::forward<Args>(args)...);
    }
    template <typename FunctionType, typename... Args>
    void setPathConstraints(Args&&... args) {
        m_path_func =
                std::unique_ptr<FunctionType>(std::forward<Args>(args)...);
    }
private:
    std::unordered_map<std::string, StateInfo> m_state_infos;
    std::unordered_map<std::string, ControlInfo> m_control_infos;
    std::unique_ptr<casadi::Function> m_integral_cost_func;
    std::unique_ptr<casadi::Function> m_endpoint_cost_func;
    std::unique_ptr<casadi::Function> m_diffeq_func;
    std::unique_ptr<casadi::Function> m_path_func;
    friend Solver;
    // TODO this is sloppy:
    friend TrapezoidalSolver;
};

class Solver {
public:
    Solution solve() const {
        return solveImpl();
    }
protected:
    casadi::Opti m_opti;
private:
    virtual Solution solveImpl() const = 0;


};

/*
class TrapezoidalSolver : public Solver {
public:
private:
    Solution solveImpl() const override {
        casadi::MX path_error;
        for (int itime = 0; itime < m_numTimes; ++itime) {
            pathError = prob.m_path_func({time, state, control})
            m_opti.subject_to(prob.m_pathLowerBounds <
        }
        m_opti.subje

    }

};
*/

} // namespace casocp

#endif // MOCO_CASADIPROBLEM_H
