#ifndef PATH_PLANNER_HPP
#define PATH_PLANNER_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "camera_nodes/get_roi_detections.hpp"
#include <vector>
#include <unordered_map>
#include <map>

// Remove RoiType enum and StageDetections struct entirely
// get_roi_detections.hpp already defines StageDetections

enum class RoiType
{
    REAL,
    MR,
    FAKE,
    NONE
};

class PlanPath : public BT::SyncActionNode
{
public:
    PlanPath(
        const std::string & name,
        const BT::NodeConfig & config,
        const rclcpp::Node::SharedPtr & node);

    std::vector<int> ar_nodes;
    std::vector<int> mr_nodes;
    std::vector<int> fake_nodes;
    std::vector<int> path; 

    std::map<int, std::vector<std::pair<int,int>>> base_graph_;

    int start_node_;
    int goal_node_;

    enum Team 
    {   red, 
        blue 
    }; 

    Team team; 

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    rclcpp::Node::SharedPtr node_;
    void node_from_stage(const StageDetections &stage, int base_node,
                            int &ar_idx, int &mr_idx,int &fake_idx );
  
    void applyweights();
    bool check_nbr(int nbr,const std::vector<int>& nbrs);
    void dijkstra(int start_node
        ,std::unordered_map<int,int>& dist,
         std::unordered_map<int,int>& prev );
    std::vector<int> buildPath(int goal_node
        ,const std::unordered_map<int,int>& prev);
    void path_planner();
};

#endif