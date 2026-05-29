#include "tree_flow_nodes/decision_nodes/node_analysis.hpp"

using namespace std::placeholders;

NodeAnalysis::NodeAnalysis(
    const std::string & name,
    const BT::NodeConfig & config,
    const rclcpp::Node::SharedPtr & node
)
:BT::SyncActionNode(name, config),
  node_(node)
{

}
BT::PortsList NodeAnalysis::providedPorts()
{
    return{
        BT::InputPort<std::vector<int>>("path"),
        BT::InputPort<std::vector<int>>("ar_nodes"),
        BT::OutputPort<std::vector<int>>("merged_path"), 
        BT::OutputPort<std::map<int, int>>("node_state")
       
    };
}
BT::NodeStatus NodeAnalysis::tick()
{
    auto ar_nodes =  getInput<std::vector<int>>("ar_nodes");
    auto path =  getInput<std::vector<int>>("path");
   


    if(!ar_nodes || !path)
    {
        RCLCPP_WARN(node_->get_logger(),
                "Waiting for values...");
        return BT::NodeStatus::FAILURE;
    }
    ar_scrolls = ar_nodes.value();
    planned_path = path.value();

    printf("ar_scrolls: ");
    for (int n : ar_scrolls)
    {
        printf("%d ", n);
    }
    printf("\n");
    
    scroll_path();   
    merge_path();   
    printf("node_state:\n");
    
    for (auto& [node, state] : node_state)
    {
        printf("  node %d = %d\n", node, (int)state);
    }
    std::map<int, int> node_state_int;
    for (const auto& [key, val] : node_state)
    {
        node_state_int[key] = static_cast<int>(val);
    }
    setOutput("node_state", node_state_int);
    setOutput("merged_path",merged_path);

    printf("merged_path: ");
    for (int n : merged_path)
    {
        printf("%d ", n);
    }
    printf("\n");
    printf("nodeanalysis: success\n");

    return BT::NodeStatus::SUCCESS;
}
void NodeAnalysis::scroll_path()
{
    for(size_t i = 0; i<planned_path.size(); i++)
    {
        int node = planned_path[i];
        bool found = false;
        for( size_t j = 0; j<ar_scrolls.size(); j++)
        {
            if (node == ar_scrolls[j])
            {
                node_state[node] = SAC;
                found = true;
                break;
            }
        }
        if(!found)
        {
            node_state[node] = C;
        }
    }
    for(size_t j = 0; j< ar_scrolls.size(); j++)
    {
        int node = ar_scrolls[j];
        bool found = false;
        for(size_t i = 0; i<planned_path.size(); i++)
        {
            if(node == planned_path[i])
            {
                found = true;
                break;
            }
        }
        if(!found)
        {
            node_state[node] = S;
        }
    }

    // ScrollState scroll_state = node_state[node];
}
void NodeAnalysis::merge_path()
{
    int sac_count = 0;

    for (int node : planned_path)
    {
        if (node_state[node] == SAC)
            sac_count++;
    }

    merged_path.resize(planned_path.size() + 2);
    int j = 0;
    int s_count = 0;

    if (sac_count >= 2)
    {
        for (size_t i = 0; i < planned_path.size(); i++)
        {
            merged_path[j] = planned_path[i];
            j++;
        }
    }
    else if (sac_count == 1)
    {
        for (size_t i = 0; i < planned_path.size(); i++)
        {
            int curr_node = planned_path[i];
            merged_path[j] = curr_node;
            j++;

            if (s_count < 1 && i < planned_path.size() - 1)
            {
                int s_node = find_nearby_S(curr_node);
                if (s_node != -1)
                {
                    printf("inserting s_node=%d after curr=%d\n", s_node, curr_node);
                    merged_path[j] = s_node;
                    j++;
                    s_count++;
                }
            }
        }
    }
    else  // sac_count == 0, need 2 S nodes
    {
        for (size_t i = 0; i < planned_path.size(); i++)
        {
            int curr_node = planned_path[i];
            merged_path[j] = curr_node;
            j++;

            if (s_count < 2 && i < planned_path.size() - 1)
            {
                int s_node = find_nearby_S(curr_node);
                if (s_node != -1)
                {
                    printf("inserting s_node=%d after curr=%d\n", s_node, curr_node);
                    merged_path[j] = s_node;
                    j++;
                    s_count++;
                }
            }
        }
    }

    merged_path.resize(j);
}
int NodeAnalysis::find_nearby_S(int curr_node)
{
    std::vector<int> candidates = {
        curr_node - 1,
        curr_node + 1,
        curr_node + 3
    };

    for (int node : candidates)
    {
        if (node < 1 || node > 12)
            continue;

        if (node_state.find(node) != node_state.end() && node_state.at(node) == S)
        {
            return node;
        }
    }

    return -1;
}
    
    // for (size_t i = 0; i < planned_path.size() - 1; i++) 
    // {
    //     curr_node = planned_path[i];
    //     next_node = planned_path[i + 1];
    //     if (node_state[curr_node] == SAC || node_state[next_node] == SAC)
    //     {
    //         merged_path[i] = planned_path[i];
    //         break;
    //     }
    // }



