#include <fstream>
#include <iostream>
#include <regex>
#include <chrono>
#include <filesystem>
#include <boost/program_options.hpp>
#include <boost/heap/d_ary_heap.hpp>

#include "cbs_group_constraint.hpp"


using libMultiRobotPlanning::CBS_Assignment;

int main(int argc, char* argv[]) {
  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");
  std::string inputFile;
  std::string outputFile;
  desc.add_options()("help", "produce help message")(
      "input,i", po::value<std::string>(&inputFile)->required(),
      "input cost (txt)")
      ("output,o",po::value<std::string>(&outputFile)->required(),
      "output file (YAML)");
  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") != 0u) {
      std::cout << desc << "\n";
      return 0;
    }
  } catch (po::error& e) {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

  std::set<CBS_InputData> inputData;
  // std::vector<InputData> inputData;
  processInputFile(inputFile, inputData);

  // std::cout << "-----low level search: loading data, set cost;solve assignment;put solution into a HighLevelNode------" << std::endl;
  CBS_Assignment<std::string, std::string> CBS_assignment;
  for (const auto& data : inputData) {
    CBS_assignment.setCost(data.agent, data.taskSet, data.cost);
  }
  std::map<std::string, std::set<std::string>> solution;
  int64_t cost = CBS_assignment.solve(solution);
  HighLevelNode start;
  start.id = 0;
  start.cost = cost;
  start.solution = solution;
  std::cout << "The start HLN: ";
  std::cout << start;
  typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                    boost::heap::mutable_<true> >
      open;

  auto handle = open.push(start);
  (*handle).handle = handle;

  bool outputToFile = false; 
  solution.clear();
  int id = 1;
  HighLevelNode P;
  int m_highLevelExpanded = 0; 
  int m_lowLevelExpanded = 0;
  int last_cost = 0;
  std::map<std::string, std::set<std::string>> last_solution;
  int duplicate = 0;
  while (!open.empty()) {
    m_highLevelExpanded++;
    std::cout << "=========" << m_highLevelExpanded<< " Loop ==========="  << std::endl;
    P = open.top();
    open.pop();

    if (P.cost == last_cost && P.solution.size() == last_solution.size()){  
      duplicate ++;
    }

    last_cost = P.cost;
    last_solution = P.solution;

    if (P.solution.empty()) {
      std::cout << "Cannot find a solution!" << std::endl;
    }

    std::string conflict_task;
    if (!getFirstConflict(P.solution,conflict_task)) {
      // std::cout << "done; cost: " << P.cost << std::endl;
      // solution = P.solution;
      // return true;
      std::cout << "no conflict_task, Breaking out of the loop.\n";
      outputToFile = true; 
      break;
    }

    std::cout << P;
    // std::cout << "need to find the new solution"<< std::endl;

    
    // std::cout << "conflict_task: " << conflict_task << std::endl;  
    // std::vector<Constraint> new_constraints;  // need to be set of constraints 
    std::set<std::set<Constraint>> new_constraints;
    createConstraintsFromConflict(P.solution,conflict_task,new_constraints);
    // std::cout << "new constraints: " << std::endl;
    // for (const auto& constraint : new_constraints) {
    //   std::cout << constraint;
    // }

    for (const auto& new_constraint_set : new_constraints) {
      HighLevelNode newNode;
      LowLevelSearch(new_constraint_set,inputData,P,newNode,id);
      auto handle = open.push(newNode);
      (*handle).handle = handle;
    }

  }

  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();  
  std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>( t2-t1 );
  std::cout << "Input file: " << inputFile << std::endl;
  std::cout << "Output file: " << outputFile << std::endl;
  std::cout << "runtime: " << time_used.count() << " seconds" << std::endl;
  std::cout << "highLevelExpanded: " << m_highLevelExpanded << std::endl;
  std::cout << "duplicate: " << duplicate << std::endl;

  if (outputToFile) {
    std::cout << P;
    std::ofstream out(outputFile);
    out << "cost: " << P.cost << std::endl;
    out << "assignment:" << std::endl;
    for (const auto& s : P.solution) {
      out << "  " << s.first << ": ";
      for (const auto& element : s.second) {
        out << element << " ";
      }
      out << std::endl;
    }
    out << "runtime: " << time_used.count() << std::endl;
    out << "highLevelExpanded: " << m_highLevelExpanded << std::endl;
    out << "lowLevelExpanded: " << id << std::endl;
    out << "duplicate: " << duplicate << std::endl;
  }
  else{
    std::cout << "didn't find the result!" << std::endl;
  }

}
