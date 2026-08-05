
#include <fstream>
#include <iostream>
#include "cbs_assignment.hpp"
#include <boost/heap/d_ary_heap.hpp>

using libMultiRobotPlanning::CBS_Assignment;


struct Constraint{
  std::string agent;
  std::set<std::string> taskSet;

  bool operator<(const Constraint& other) const {
    if (agent < other.agent) return true;
    if (agent > other.agent) return false;
    return taskSet < other.taskSet;
  }

  friend std::ostream& operator<<(std::ostream& os, const Constraint& c) {
    os << "Agent: " << c.agent << ", Tasks: ";
    for (const std::string& task : c.taskSet) {
      os << task << " ";
    }
    os << std::endl;
    return os;
  }
};

struct HighLevelNode {
  std::map<std::string, std::set<std::string>> solution;
  std::set<Constraint> constraints;

  long cost;
  int id;

  typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                    boost::heap::mutable_<true> >::handle_type
      handle;

  bool operator<(const HighLevelNode& n) const {
    if (solution.size() != n.solution.size()){
      return solution.size() < n.solution.size(); // Nodes with more pairs come first
    }
    if (cost != n.cost){
      return cost > n.cost;
    }
    return id > n.id;
  }

  friend std::ostream& operator<<(std::ostream& os, const HighLevelNode& c) {
    os << "id: " << c.id << " cost: " << c.cost<< " Solution size: " << c.solution.size() << std::endl;
    
    if (c.solution.empty()) {
      os << "No sets in the solution map." << std::endl;
    }
    else{
      os << "solution:\n";
      for (const auto& s : c.solution) {
        os << s.first << ": ";
        for (const auto& element : s.second) {
          os << element << " ";
        }
        os << std::endl;
      }
    }
    // if (c.constraints.empty()) {
    //   os << "No constraints." << std::endl;
    // } else {
    //   os << "Constraints:" << std::endl;
    //   for (const auto& constraint : c.constraints) {
    //     os << constraint;
    //   }
    // }
    return os;
  }
};

struct CBS_InputData {
  std::string agent;
  long cost;
  std::set<std::string> taskSet;

  bool operator<(const CBS_InputData& other) const {
    if (agent != other.agent) {
      return agent < other.agent;
    }
    if (cost != other.cost) {
      return cost < other.cost;
    }
    return taskSet < other.taskSet;
  }
  friend std::ostream& operator<<(std::ostream& os, const CBS_InputData& c) {
    os << "Agent: " << c.agent << ", Cost: " << c.cost << ", Tasks: ";
    for (const std::string& task : c.taskSet) {
      os << task << " ";
    }
    os << std::endl;
    return os;
  }
};

void processInputFile(const std::string& inputFile, std::set<CBS_InputData>& inputData) {
    std::ifstream input(inputFile);
    for (std::string line; getline(input, line);) {
        std::stringstream stream(line);
        CBS_InputData data;
        stream >> data.agent;
        stream >> data.cost;
        std::string task;
        bool skipLine = false;
        while (stream >> task) {
            if (data.taskSet.find(task) != data.taskSet.end()) {
                skipLine = true;
                break;
            }
            data.taskSet.insert(task);
        }
        
        if (!skipLine) {
            inputData.insert(data);
        }
    }
}

bool getFirstConflict(
    const std::map<std::string, std::set<std::string>>& solution,
    std::string& conflict_task) {
  std::unordered_map<std::string, int> taskCounts; 
  for (auto iter = solution.begin(); iter != solution.end(); ++iter) {
    std::set<std::string> current_set = iter->second;
    for (const std::string& task : current_set){
      taskCounts[task]++;
      if (taskCounts[task] > 1){
        // std::cout << "Element appearing more than once: task" << task << std::endl;
        conflict_task = task;
        return true;
      }
    }
  }    
  return false;
}

void createConstraintsFromConflict(
    const std::map<std::string, std::set<std::string>>& solution,
    const std::string& conflict_task, 
    std::set<std::set<Constraint>>& new_constraints){
  int count = 0;
  std::set<Constraint> all_constraints;
  for (const auto& pair : solution) {
    std::set<std::string> current_set = pair.second;
    for (const std::string& task : current_set){ 
      if (task == conflict_task){
        Constraint con;
        con.agent = pair.first;
        con.taskSet = pair.second;
        all_constraints.insert(con);
      }
    }
  }
  for (const auto& constraint : all_constraints) {
    std::cout <<"constraint:" << std::endl;
    std::cout << constraint;
    std::set<Constraint> constraint_set(all_constraints);
    constraint_set.erase(constraint);
    new_constraints.insert(constraint_set);
  }
  
  for (const auto& constraint_set : new_constraints) {
    std::cout <<"constraint_set:" << std::endl;
    for (const auto& constraint : constraint_set) {
        std::cout << constraint;
    }
  }

}

void LowLevelSearch(
    const std::set<Constraint>& new_constraint_set,
    const std::set<CBS_InputData>& inputData,
    const HighLevelNode& P,
    HighLevelNode& newNode,
    int& id){
  newNode.id = id;
  ++id;
  newNode.constraints = P.constraints;
  for (const auto& constraint : new_constraint_set) {
      newNode.constraints.insert(constraint);
  }
  CBS_Assignment<std::string, std::string> CBS_assignment;
  for (const auto& data : inputData) {
    bool skipData = false;
    for (const auto& constraint : newNode.constraints) {
      if (data.agent == constraint.agent && data.taskSet == constraint.taskSet) {
        skipData = true;
        break;
      }
    }
    if (!skipData) {
      CBS_assignment.setCost(data.agent, data.taskSet, data.cost);
    }
  }

  std::map<std::string, std::set<std::string>> solution;
  int64_t cost = CBS_assignment.solve(solution);

  newNode.cost = cost;
  newNode.solution = solution;
}