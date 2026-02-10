#include <atomic>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <thread>
#include <unordered_set>
#include <vector>

#include <domus/core/csv.hpp>
#include <domus/core/graph/file_loader.hpp>
#include <domus/core/graph/graph.hpp>
#include <domus/core/utils.hpp>
#include <domus/orthogonal/drawing_stats.hpp>

#include "ogdf-drawer.hpp"

std::unordered_set<std::string> graphs_already_in_csv;

void save_stats(
    std::ofstream& csv_filename,
    const OrthogonalStats& stats,
    double ogdf_time,
    const std::string& graph_name) {
    csv_filename << graph_name << "," << stats.crossings << "," << stats.bends
                 << "," << stats.area << "," << stats.total_edge_length << ","
                 << stats.max_edge_length << "," << stats.max_bends_per_edge
                 << "," << stats.edge_length_stddev << "," << stats.bends_stddev
                 << "," << ogdf_time << std::endl;
}

void compute_stats_ogdf(
    std::string& folder_path,
    std::ofstream& csv_filename,
    std::string& svgs_folder) {
    auto txt_files = collect_txt_files(folder_path);
    std::atomic<int> number_of_processed_graphs{0};
    std::mutex input_output_lock;
    std::atomic<int> index{0};
    // unsigned num_threads = std::thread::hardware_concurrency();
    unsigned num_threads = 1;
    std::vector<std::thread> threads;
    for (unsigned i = 0; i < num_threads; ++i) {
        threads.emplace_back([&]() {
            while (true) {
                int current = index.fetch_add(1, std::memory_order_relaxed);
                if (current >= txt_files.size())
                    break;
                const auto& entry_path = txt_files[current];
                const std::string graph_filename =
                    std::filesystem::path(entry_path).stem().string();
                int current_number = number_of_processed_graphs.fetch_add(
                    1, std::memory_order_relaxed);
                if (graphs_already_in_csv.contains(graph_filename))
                    continue;
                std::unique_ptr<UndirectedSimpleGraph> graph =
                    load_graph_from_txt_file(entry_path);
                {
                    std::lock_guard<std::mutex> lock(input_output_lock);
                    std::cout << "Processing graph #" << current_number << " - "
                              << graph_filename << std::endl;
                }
                const std::string svg_filename =
                    svgs_folder + graph_filename + "_ogdf.svg";
                auto [drawing, time] =
                    make_orthogonal_drawing_ogdf(*graph, svg_filename);
                OrthogonalStats stats = compute_all_orthogonal_stats(drawing);
                {
                    std::lock_guard<std::mutex> lock(input_output_lock);
                    save_stats(csv_filename, stats, time, graph_filename);
                }
            }
        });
    }
    for (auto& t : threads)
        if (t.joinable())
            t.join();
    std::cout << "All graphs processed.\nThreads used: " << num_threads
              << "\nTotal graphs: " << number_of_processed_graphs.load()
              << std::endl;
}

void initialize_csv_file(std::ofstream& result_file) {
    if (!result_file.is_open())
        throw std::runtime_error("Error: Could not open result file");
    result_file << "graph_name,crossings,bends,area,total_edge_length,"
                << "max_edge_length,max_bends_per_edge,edge_length_stddev,"
                << "bends_stddev,time" << std::endl;
}

void run_stats() {
    std::cout << "Running stats ogdf..." << std::endl;
    std::string csv_filename = "test_results.csv";
    std::ofstream result_file;
    if (std::filesystem::exists(csv_filename)) {
        std::cout << "File " << csv_filename << " already exists." << std::endl
                  << "What do you want to do?" << std::endl
                  << "1. Overwrite the file" << std::endl
                  << "2. Append to the file" << std::endl
                  << "3. Abort" << std::endl
                  << "Please enter your choice (1/2/3): ";
        int choice;
        std::cin >> choice;
        if (choice == 1) {
            std::filesystem::remove(csv_filename);
            result_file.open(csv_filename);
            initialize_csv_file(result_file);
        } else if (choice == 2) {
            CSVData csv_data = parse_csv(csv_filename);
            for (const auto& row : csv_data.rows)
                if (row.size() > 0)
                    graphs_already_in_csv.insert(row[0]);
            result_file.open(csv_filename, std::ios_base::app);
        } else {
            std::cout << "Aborting." << std::endl;
            return;
        }
    } else {
        result_file.open(csv_filename);
        initialize_csv_file(result_file);
    }
    std::string svgs_folder = "output-svgs/";
    if (!std::filesystem::exists(svgs_folder))
        if (!std::filesystem::create_directories(svgs_folder)) {
            std::cerr << "Error: Could not create directory " << svgs_folder
                      << std::endl;
            return;
        }
    std::string graphs_folder = "rome_graphs/";
    compute_stats_ogdf(graphs_folder, result_file, svgs_folder);
    std::cout << std::endl;
    result_file.close();
}

int main() {
    run_stats();
    return 0;
}
