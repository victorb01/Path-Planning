#include <cstdlib>
#include <vector>

#include "a_star2d.h"
#include "occupancy_grid2d.h"
#include "path_info.h"
#include "polynomial_solver.h"
#include "polynomial_sampler.h"
#include "gnuplot-iostream.h"
#include "gui2d.h"

using namespace game_engine;

PathInfo RunAStar(
    const Graph2D& graph,
    const OccupancyGrid2D* occupancy_grid,
    const std::shared_ptr<Node2D>& start_node,
    const std::shared_ptr<Node2D>& end_node);

int main(int argc, char** argv) {
  if(argc != 6) {
    std::cerr << "Usage: ./full_stack_planning occupancy_grid_file row1 col1 row2 col2" << std::endl;
    return EXIT_FAILURE;
  }

  // Parsing input
  const std::string occupancy_grid_file = argv[1];
  const std::shared_ptr<Node2D> start_ptr = std::make_shared<Node2D>(
      Eigen::Vector2d(std::stoi(argv[2]),std::stoi(argv[3])));
  const std::shared_ptr<Node2D> end_ptr = std::make_shared<Node2D>(
      Eigen::Vector2d(std::stoi(argv[4]),std::stoi(argv[5])));

  // Load an occupancy grid from a file
  OccupancyGrid2D occupancy_grid;
  occupancy_grid.LoadFromFile(occupancy_grid_file);

  // Transform an occupancy grid into a graph
  const Graph2D graph = occupancy_grid.AsGraph();

  const PathInfo path_info = RunAStar(graph, &occupancy_grid, start_ptr, end_ptr);

  int num_of_waypoints = path_info.details.path_length;
  std::vector<double> times;
  std::vector<p4::NodeEqualityBound> node_equality_bounds;
  node_equality_bounds.push_back(p4::NodeEqualityBound(0,0,1,0));
  node_equality_bounds.push_back(p4::NodeEqualityBound(1,0,1,0));
  node_equality_bounds.push_back(p4::NodeEqualityBound(0,0,2,0));
  node_equality_bounds.push_back(p4::NodeEqualityBound(1,0,2,0));
  std::vector<std::shared_ptr<Node2D>> path = path_info.path;
  std::shared_ptr<Node2D>& node_ptr = path.back();
  for(int i = 0; i < num_of_waypoints; i++) {
    times.push_back(i);
    node_ptr = path.back(); path.pop_back();
    node_equality_bounds.push_back(p4::NodeEqualityBound(0,i,0,node_ptr->Data().x()));
    node_equality_bounds.push_back(p4::NodeEqualityBound(1,i,0,node_ptr->Data().y()));
  }

  // Options to configure the polynomial solver with
  p4::PolynomialSolver::Options solver_options;
  solver_options.num_dimensions = 2;     // 2D
  solver_options.polynomial_order = 8;   // Fit an 8th-order polynomial
  solver_options.continuity_order = 4;   // Require continuity to the 4th order
  solver_options.derivative_order = 2;   // Minimize acceleration

  osqp_set_default_settings(&solver_options.osqp_settings);
  solver_options.osqp_settings.polish = true;       // Polish the solution, getting the best answer possible
  solver_options.osqp_settings.verbose = false;     // Suppress the printout

  // Use p4::PolynomialSolver object to solve for polynomial trajectories
  p4::PolynomialSolver solver(solver_options);
  const p4::PolynomialSolver::Solution solution_path
    = solver.Run(
        times, 
        node_equality_bounds, 
        {}, 
        {});

  // Sampling and Plotting
  { // Plot 2D position
    // Options to configure the polynomial sampler with
    p4::PolynomialSampler::Options sampler_options;
    sampler_options.frequency = 200;             // Number of samples per second
    sampler_options.derivative_order = 0;        // Derivative to sample (0 = pos)

    // Use this object to sample a trajectory
    p4::PolynomialSampler sampler(sampler_options);
    Eigen::MatrixXd samples = sampler.Run(times, solution_path);

    // Plotting tool requires vectors
    std::vector<double> t_hist, x_hist, y_hist, x_hist_v, y_hist_v, x_hist_a, y_hist_a;
    for(size_t time_idx = 0; time_idx < samples.cols(); ++time_idx) {
      t_hist.push_back(samples(0,time_idx));
      x_hist.push_back(samples(1,time_idx));
      y_hist.push_back(samples(2,time_idx));
    }
    // gnu-iostream plotting library
    // Utilizes gnuplot commands with a nice stream interface
    {
      Gnuplot gp;
      gp << "plot '-' using 1:2 with lines title 'Trajectory'" << std::endl;
      gp.send1d(boost::make_tuple(y_hist, x_hist));
      gp << "set grid" << std::endl;
      gp << "set xlabel 'X'" << std::endl;
      gp << "set ylabel 'Y'" << std::endl;
      gp << "replot" << std::endl;
    }
    {
      Gnuplot gp;
      gp << "plot '-' using 1:2 with lines title 'X-Profile'" << std::endl;
      gp.send1d(boost::make_tuple(t_hist, x_hist));
      gp << "set grid" << std::endl;
      gp << "set xlabel 'Time (s)'" << std::endl;
      gp << "set ylabel 'X-Profile'" << std::endl;
      gp << "replot" << std::endl;
    }
    {
      Gnuplot gp;
      gp << "plot '-' using 1:2 with lines title 'Y-Profile'" << std::endl;
      gp.send1d(boost::make_tuple(t_hist, y_hist));
      gp << "set grid" << std::endl;
      gp << "set xlabel 'Time (s)'" << std::endl;
      gp << "set ylabel 'Y-Profile'" << std::endl;
      gp << "replot" << std::endl;
    }
  
    // Velocity and acceleration
    sampler_options.derivative_order = 1;        // Derivative to sample (1 = vel)

    // Use this object to sample a trajectory
    p4::PolynomialSampler sampler_v(sampler_options);
    Eigen::MatrixXd samples_v = sampler_v.Run(times, solution_path);

    // Plotting tool requires vectors
    t_hist.clear();
    for(size_t time_idx = 0; time_idx < samples_v.cols(); ++time_idx) {
      t_hist.push_back(samples_v(0,time_idx));
      x_hist_v.push_back(samples_v(1,time_idx));
      y_hist_v.push_back(samples_v(2,time_idx));
    }

    {
      Gnuplot gp;
      gp << "plot '-' using 1:2 with lines title 'X-velocity-Profile'" << std::endl;
      gp.send1d(boost::make_tuple(t_hist, x_hist_v));
      gp << "set grid" << std::endl;
      gp << "set xlabel 'Time (s)'" << std::endl;
      gp << "set ylabel 'X-velocity-Profile'" << std::endl;
      gp << "replot" << std::endl;
    }
    {
      Gnuplot gp;
      gp << "plot '-' using 1:2 with lines title 'Y-velocity-Profile'" << std::endl;
      gp.send1d(boost::make_tuple(t_hist, y_hist_v));
      gp << "set grid" << std::endl;
      gp << "set xlabel 'Time (s)'" << std::endl;
      gp << "set ylabel 'Y-velocity-Profile'" << std::endl;
      gp << "replot" << std::endl;
    }

    sampler_options.derivative_order = 2;        // Derivative to sample (2 = acc)

    // Use this object to sample a trajectory
    p4::PolynomialSampler sampler_a(sampler_options);
    Eigen::MatrixXd samples_a = sampler_a.Run(times, solution_path);

    // Plotting tool requires vectors
    t_hist.clear();
    for(size_t time_idx = 0; time_idx < samples_a.cols(); ++time_idx) {
      t_hist.push_back(samples_a(0,time_idx));
      x_hist_a.push_back(samples_a(1,time_idx));
      y_hist_a.push_back(samples_a(2,time_idx));
    }

    {
      Gnuplot gp;
      gp << "plot '-' using 1:2 with lines title 'X-acceleration-Profile'" << std::endl;
      gp.send1d(boost::make_tuple(t_hist, x_hist_a));
      gp << "set grid" << std::endl;
      gp << "set xlabel 'Time (s)'" << std::endl;
      gp << "set ylabel 'X-acceleration-Profile'" << std::endl;
      gp << "replot" << std::endl;
    }
    {
      Gnuplot gp;
      gp << "plot '-' using 1:2 with lines title 'Y-acceleration-Profile'" << std::endl;
      gp.send1d(boost::make_tuple(t_hist, y_hist_a));
      gp << "set grid" << std::endl;
      gp << "set xlabel 'Time (s)'" << std::endl;
      gp << "set ylabel 'Y-acceleration-Profile'" << std::endl;
      gp << "replot" << std::endl;
    }
  
  std::vector<std::pair<std::string, std::vector<double>>> outputToFile = {
    {"Time", t_hist},
    {"X position", x_hist}, {"Y position", y_hist},
    {"X velocity", x_hist_v}, {"Y velocity", y_hist_v},
    {"X acceleration", x_hist_a}, {"Y acceleration", y_hist_a}};
  std::ofstream myFile("Path_output.csv");
  for(int j = 0; j < outputToFile.size(); ++j)
    {
        myFile << outputToFile.at(j).first;
        if(j != outputToFile.size() - 1) myFile << ","; // No comma at end of line
    }
    myFile << "\n";
    
    // Send data to the stream
    for(int i = 0; i < outputToFile.at(0).second.size(); ++i)
    {
        for(int j = 0; j < outputToFile.size(); ++j)
        {
            myFile << outputToFile.at(j).second.at(i);
            if(j != outputToFile.size() - 1) myFile << ","; // No comma at end of line
        }
        myFile << "\n";
    }
    
    // Close the file
    myFile.close();
  }
  return EXIT_SUCCESS;
}

PathInfo RunAStar(
    const Graph2D& graph,
    const OccupancyGrid2D* occupancy_grid,
    const std::shared_ptr<Node2D>& start_node,
    const std::shared_ptr<Node2D>& end_node) {
  
  std::cout << "============================================" << std::endl;
  std::cout << "=============  RUNNING A Star  =============" << std::endl;
  std::cout << "============================================" << std::endl;

  // Run A*
  AStar2D a_star;
  PathInfo path_info = a_star.Run(graph, start_node, end_node);

  // Display the solution
  Gui2D gui;
  gui.LoadOccupancyGrid(occupancy_grid);
  gui.LoadPath(path_info.path);
  gui.Display();

  // Print the solution
  path_info.details.Print();

  std::cout << "=====  PATH   =====" << std::endl;
  for(const std::shared_ptr<Node2D>& node: path_info.path) {
    std::cout << "[" << node->Data().transpose() << "]" << std::endl;
  }

  std::cout << std::endl;

  return path_info;

}
