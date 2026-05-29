#include "tree_flow_nodes/decision_nodes/path_planner.hpp"

using namespace std::placeholders;

// Helper to convert string -> RoiType
static RoiType roiFromString(const std::string& s)
{
    if (s == "real") return RoiType::REAL;
    if (s == "mr")   return RoiType::MR;
    if (s == "fake") return RoiType::FAKE;
    return RoiType::NONE;
}

PlanPath::PlanPath(
    const std::string & name,
    const BT::NodeConfig & config,
    const rclcpp::Node::SharedPtr & node)
: BT::SyncActionNode(name, config),
  node_(node)
{
    base_graph_= {
        {2,  {{5,1},{1,1},{3,1}}},
        {1,  {{4,1},{2,1}}},
        {3,  {{6,1},{2,1}}},
        {4,  {{5,1},{7,1}}},
        {6,  {{5,1},{9,1}}},
        {9,  {{12,1},{8,1}}},
        {8,  {{11,1},{9,1},{7,1}}},
        {11, {{12,1},{10,1}}},
        {7,  {{10,1},{8,1}}},
        {5,  {{6,1},{4,1},{8,1}}},
        {10, {{11,1}}},
        {12, {{11,1}}}
    };
    team = red;
    start_node_ = 2;
   
    if (team == red)
        goal_node_ = 12;
    else
        goal_node_ = 10;
}

BT::PortsList PlanPath::providedPorts()
{
    return {
        BT::InputPort<StageDetections>("detection_s1"),
        BT::InputPort<StageDetections>("detection_s2"),
        BT::InputPort<StageDetections>("detection_s3"),
        BT::InputPort<StageDetections>("detection_s4"),
        BT::OutputPort<std::vector<int>>("path"),
        BT::OutputPort<std::vector<int>>("ar_nodes")
    };
}

BT::NodeStatus PlanPath::tick()
{
    auto s1 = getInput<StageDetections>("detection_s1");
    auto s2 = getInput<StageDetections>("detection_s2");
    auto s3 = getInput<StageDetections>("detection_s3");
    auto s4 = getInput<StageDetections>("detection_s4");

    StageDetections stage1, stage2, stage3, stage4;

    if(!s1 || !s2 || !s3 || !s4)
    {
        RCLCPP_WARN(node_->get_logger(), "No detections, using test defaults");

        // Fake test data — use strings to match StageDetections fields
        stage1.roi1 = "real"; stage1.roi2 = "fake"; stage1.roi3 = "none";
        stage2.roi1 = "mr";   stage2.roi2 = "real"; stage2.roi3 = "none";
        stage3.roi1 = "none"; stage3.roi2 = "real"; stage3.roi3 = "fake";
        stage4.roi1 = "real"; stage4.roi2 = "mr";   stage4.roi3 = "mr";
    }
    else
    {
        stage1 = s1.value();
        stage2 = s2.value();
        stage3 = s3.value();
        stage4 = s4.value();
    }

    ar_nodes.clear();
    mr_nodes.clear();
    fake_nodes.clear();

    ar_nodes.resize(12);
    mr_nodes.resize(12);
    fake_nodes.resize(12);

    int ar_idx = 0, mr_idx = 0, fake_idx = 0;

    node_from_stage(stage1, 1, ar_idx, mr_idx, fake_idx);
    node_from_stage(stage2, 4, ar_idx, mr_idx, fake_idx);
    node_from_stage(stage3, 7, ar_idx, mr_idx, fake_idx);
    node_from_stage(stage4, 10, ar_idx, mr_idx, fake_idx);

    path_planner();

    setOutput<std::vector<int>>("path", path);
    setOutput<std::vector<int>>("ar_nodes", ar_nodes);

    printf("PlanPath:sucess\n");
    return BT::NodeStatus::SUCCESS;
}

void PlanPath::node_from_stage(const StageDetections &stage, int base_node,
                               int &ar_idx, int &mr_idx, int &fake_idx)
{
    // Convert string fields to RoiType
    RoiType rois[3] = {
        roiFromString(stage.roi1),
        roiFromString(stage.roi2),
        roiFromString(stage.roi3)
    };

    for (int i = 0; i < 3; i++)
    {
        int node_number = base_node + i;
        switch(rois[i])
        {
            case RoiType::REAL:
                ar_nodes[ar_idx] = node_number;
                ar_idx++;
                break;
            case RoiType::MR:
                mr_nodes[mr_idx] = node_number;
                mr_idx++;
                break;
            case RoiType::FAKE:
                fake_nodes[fake_idx] = node_number;
                fake_idx++;
                break;
            case RoiType::NONE:
                break;
        }
    }
}

// Everything below is unchanged
void PlanPath::applyweights()
{
    for (auto & [node, edges] : base_graph_) {
        for (auto & edge : edges) {
            int nbr = edge.first;
            if (check_nbr(nbr, ar_nodes))        edge.second = 50;
            else if (check_nbr(nbr, mr_nodes))   edge.second = 20;
            else if (check_nbr(nbr, fake_nodes)) edge.second = 100;
            else                                 edge.second = 1;
        }
    }
}

bool PlanPath::check_nbr(int nbr, const std::vector<int>& nbrs)
{
    for (size_t i = 0; i < nbrs.size(); i++)
        if (nbr == nbrs[i]) return true;
    return false;
}

void PlanPath::dijkstra(int start_node,
    std::unordered_map<int,int>& dist,
    std::unordered_map<int,int>& prev)
{
    int INF = 1000000000;
    for (auto pair : base_graph_) { dist[pair.first] = INF; prev[pair.first] = -1; }
    dist[start_node] = 0;

    std::vector<int> unvisited;
    for (auto pair : base_graph_) unvisited.push_back(pair.first);

    while (!unvisited.empty())
    {
        int current = -1, best_dist = INF;
        for (size_t i = 0; i < unvisited.size(); i++)
            if (dist[unvisited[i]] < best_dist) { best_dist = dist[unvisited[i]]; current = unvisited[i]; }

        if (current == -1) break;

        for (size_t i = 0; i < unvisited.size(); i++)
            if (unvisited[i] == current) { unvisited.erase(unvisited.begin() + i); break; }

        for (auto& edge : base_graph_[current])
        {
            int new_dist = dist[current] + edge.second;
            if (new_dist < dist[edge.first]) { dist[edge.first] = new_dist; prev[edge.first] = current; }
        }
    }
}

void PlanPath::path_planner()
{
    applyweights();
    std::unordered_map<int, int> dist, prev;
    dijkstra(start_node_, dist, prev);
    path = buildPath(goal_node_, prev);
}

std::vector<int> PlanPath::buildPath(int goal_node,
    const std::unordered_map<int,int>& prev)
{
    std::vector<int> path;
    int node = goal_node;
    while (node != -1) { path.insert(path.begin(), node); node = prev.at(node); }

    printf("Path: ");
    for (int n : path) printf("%d ", n);
    printf("\n");
    return path;
}