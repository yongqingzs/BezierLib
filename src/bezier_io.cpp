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
        case nlopt::LN_COBYLA: return "LN_COBYLA";
        case nlopt::LN_BOBYQA: return "LN_BOBYQA";
        case nlopt::LN_NELDERMEAD: return "LN_NELDERMEAD";
        case nlopt::LN_SBPLX: return "LN_SBPLX";
        case nlopt::LD_SLSQP: return "LD_SLSQP";
        case nlopt::LD_MMA: return "LD_MMA";
        case nlopt::GN_DIRECT: return "GN_DIRECT";
        case nlopt::GN_CRS2_LM: return "GN_CRS2_LM";
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