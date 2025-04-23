#include "bezier.hpp"

namespace bezier { 

std::ostream& operator<<(std::ostream& os, const NodeData& node) {
    os << "NodeData{start:(" << node.start_point[0] << "," << node.start_point[1] 
       << "), heading:" << node.heading 
       << ", speed:" << node.speed 
       << ", r_min:" << node.r_min << "}";
    return os;
}

std::ostream& operator<<(std::ostream& os, const InitData& init) {
    os << "InitData{target:(" << init.target_point[0] << "," << init.target_point[1] 
       << "), node_num:" << init.node_num << ", nodes:[";
    for (int i = 0; i < init.node_num && i < init.nodes.size(); ++i) {
        os << "\n  " << i << ": " << init.nodes[i];
        if (i < init.node_num - 1 && i < init.nodes.size() - 1) {
            os << ",";
        }
    }
    os << "\n]}";
    return os;
}

const char* nloptAlgorithmToString(nlopt::algorithm alg) {
    switch (alg) {
        // 无梯度全局优化算法 (GN_)
        case nlopt::GN_DIRECT: return "GN_DIRECT";
        case nlopt::GN_DIRECT_L: return "GN_DIRECT_L";
        case nlopt::GN_DIRECT_L_RAND: return "GN_DIRECT_L_RAND";
        case nlopt::GN_DIRECT_NOSCAL: return "GN_DIRECT_NOSCAL";
        case nlopt::GN_DIRECT_L_NOSCAL: return "GN_DIRECT_L_NOSCAL";
        case nlopt::GN_DIRECT_L_RAND_NOSCAL: return "GN_DIRECT_L_RAND_NOSCAL";
        case nlopt::GN_ORIG_DIRECT: return "GN_ORIG_DIRECT";
        case nlopt::GN_ORIG_DIRECT_L: return "GN_ORIG_DIRECT_L";
        case nlopt::GN_CRS2_LM: return "GN_CRS2_LM";
        case nlopt::GN_MLSL: return "GN_MLSL";
        case nlopt::GN_MLSL_LDS: return "GN_MLSL_LDS";
        case nlopt::GN_ISRES: return "GN_ISRES";
        case nlopt::GN_ESCH: return "GN_ESCH";
        case nlopt::GN_AGS: return "GN_AGS";
        
        // 无梯度局部优化算法 (LN_)
        case nlopt::LN_COBYLA: return "LN_COBYLA";
        case nlopt::LN_BOBYQA: return "LN_BOBYQA";
        case nlopt::LN_NEWUOA: return "LN_NEWUOA";
        case nlopt::LN_NEWUOA_BOUND: return "LN_NEWUOA_BOUND";
        case nlopt::LN_NELDERMEAD: return "LN_NELDERMEAD";
        case nlopt::LN_SBPLX: return "LN_SBPLX";
        case nlopt::LN_AUGLAG: return "LN_AUGLAG";
        case nlopt::LN_AUGLAG_EQ: return "LN_AUGLAG_EQ";
        case nlopt::LN_PRAXIS: return "LN_PRAXIS";
        
        // 带梯度全局优化算法 (GD_)
        case nlopt::GD_STOGO: return "GD_STOGO";
        case nlopt::GD_STOGO_RAND: return "GD_STOGO_RAND";
        case nlopt::GD_MLSL: return "GD_MLSL";
        case nlopt::GD_MLSL_LDS: return "GD_MLSL_LDS";
        
        // 带梯度局部优化算法 (LD_)
        case nlopt::LD_LBFGS: return "LD_LBFGS";
        case nlopt::LD_VAR1: return "LD_VAR1";
        case nlopt::LD_VAR2: return "LD_VAR2";
        case nlopt::LD_TNEWTON: return "LD_TNEWTON";
        case nlopt::LD_TNEWTON_RESTART: return "LD_TNEWTON_RESTART";
        case nlopt::LD_TNEWTON_PRECOND: return "LD_TNEWTON_PRECOND";
        case nlopt::LD_TNEWTON_PRECOND_RESTART: return "LD_TNEWTON_PRECOND_RESTART";
        case nlopt::LD_SLSQP: return "LD_SLSQP";
        case nlopt::LD_MMA: return "LD_MMA";
        case nlopt::LD_CCSAQ: return "LD_CCSAQ";
        case nlopt::LD_AUGLAG: return "LD_AUGLAG";
        case nlopt::LD_AUGLAG_EQ: return "LD_AUGLAG_EQ";
        
        default: return "UNKNOWN";
    }
}

const char* nloptResultToString(nlopt::result result) {
    switch(static_cast<int>(result)) {
        case 1: return "SUCCESS";
        case 2: return "STOPVAL_REACHED";
        case 3: return "FTOL_REACHED";
        case 4: return "XTOL_REACHED";
        case 5: return "MAXEVAL_REACHED";
        case -1: return "FAILURE";
        case -2: return "INVALID_ARGS";
        case -3: return "OUT_OF_MEMORY";
        case -4: return "ROUNDOFF_LIMITED";
        case -5: return "FORCED_STOP";
        default: return "UNKNOWN";
    }
}

std::ostream& operator<<(std::ostream& os, const OptParms& opt) {
    os << "OptParms{layer:" << (opt.layer ? "true" : "false") 
       << ", opt_type:" << opt.opt_type
       << ", num_samples:" << opt.num_samlpes
       << ", target_length:" << opt.target_length
       << ", target_radius:" << opt.target_radius
       << ", fixed_angle:" << opt.fixed_angle;
    
    os << ", algo_first:" << nloptAlgorithmToString(opt.algo_first)
       << ", algo_second:" << nloptAlgorithmToString(opt.algo_second);
    
    os << "\n  bounds_first:{ lower:[";
    for (size_t i = 0; i < opt.lower_bounds_first.size(); ++i) {
        os << opt.lower_bounds_first[i];
        if (i < opt.lower_bounds_first.size() - 1) os << ",";
    }
    os << "], upper:[";
    for (size_t i = 0; i < opt.upper_bounds_first.size(); ++i) {
        os << opt.upper_bounds_first[i];
        if (i < opt.upper_bounds_first.size() - 1) os << ",";
    }
    os << "] }";
    
    os << "\n  bounds_second:{ lower:[";
    for (size_t i = 0; i < opt.lower_bounds_second.size(); ++i) {
        os << opt.lower_bounds_second[i];
        if (i < opt.lower_bounds_second.size() - 1) os << ",";
    }
    os << "], upper:[";
    for (size_t i = 0; i < opt.upper_bounds_second.size(); ++i) {
        os << opt.upper_bounds_second[i];
        if (i < opt.upper_bounds_second.size() - 1) os << ",";
    }
    os << "] }";
    
    os << "}";
    return os;
}

}