#include <stack>

#include "depth_first_search2d.h"

namespace game_engine {
  // Anonymous namespace. Put any file-local functions or variables in here
  namespace {
    // Helper struct that functions as a linked list with data. The linked
    // list represents a path. Data members are a node and a cost to reach
    // that node.
    struct NodeWrapper {
      std::shared_ptr<struct NodeWrapper> parent;
      std::shared_ptr<Node2D> node_ptr;
      double cost;

      // Equality operator
      bool operator==(const NodeWrapper& other) const {
        return *(this->node_ptr) == *(other.node_ptr);
      }
    };

    using NodeWrapperPtr = std::shared_ptr<NodeWrapper>;
    // Returns true if the NodeWrapper pointed to by nwPtr is found among
    // those pointed to by the elements of nwPtrVec; otherwise returns false.
    bool is_present(const NodeWrapperPtr nwPtr,
                    const std::vector<NodeWrapperPtr>& nwPtrVec) {
        for(auto n : nwPtrVec) {
          if(*n == *nwPtr)
            return true;
        }
        return false;
    }
  }

  PathInfo DepthFirstSearch2D::Run(
      const Graph2D& graph, 
      const std::shared_ptr<Node2D> start_ptr, 
      const std::shared_ptr<Node2D> end_ptr) {
    using NodeWrapperPtr = std::shared_ptr<NodeWrapper>;

    Timer timer;
    timer.Start();

    // Declare data structures
    std::stack<NodeWrapperPtr> to_explore;
    std::vector<NodeWrapperPtr> explored;
    NodeWrapperPtr node_to_explore;
    PathInfo path_info;
  
    // Initilize nw_ptr with start_ptr
    NodeWrapperPtr nw_ptr = std::make_shared<NodeWrapper>();
    nw_ptr->parent = nullptr;
    nw_ptr->node_ptr = start_ptr;
    nw_ptr->cost = 0;
    to_explore.push(nw_ptr);

    while(true) {
      if(to_explore.empty())  {
        std::cout << "No path to destination found" << std::endl;
        return path_info;
      }
      node_to_explore = to_explore.top(); to_explore.pop();
      if(is_present(node_to_explore, explored)) {
        continue;
      }
      if(*node_to_explore->node_ptr == *end_ptr)  {
        break;
      }
      else {
        explored.push_back(node_to_explore);
        auto edges = graph.Edges(node_to_explore->node_ptr);
        for(auto edge : edges) {
          auto& neighbor = edge.Sink();
          NodeWrapperPtr nw_ptr = std::make_shared<NodeWrapper>();
          nw_ptr->parent = node_to_explore;
          nw_ptr->node_ptr = neighbor;
          nw_ptr->cost = node_to_explore->cost + edge.Cost();
          to_explore.push(nw_ptr);
        }
      }
    }

    path_info.details.path_cost = node_to_explore->cost;
    // End node not counted among those explored
    path_info.details.num_nodes_explored = explored.size();
    path_info.path.push_back(node_to_explore->node_ptr);
    path_info.details.path_length = 1;
    while(!(*node_to_explore->node_ptr == *start_ptr))  {
      node_to_explore = node_to_explore->parent;
      path_info.path.push_back(node_to_explore->node_ptr);
      path_info.details.path_length += 1;
    }
    // Reverse ordering of path so that it goes from start to end
    std::reverse(path_info.path.begin(), path_info.path.end());
    path_info.details.run_time = timer.Stop();

    return path_info;
  }
  
}
