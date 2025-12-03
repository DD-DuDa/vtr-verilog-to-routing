/**
 * @file main.cpp
 * @brief Fast RR Graph transformation tool
 * 
 * This tool transforms RR graph XML files by removing edges connected to specific nodes.
 * 
 * Removes edges where sink_node or src_node equals the ID of:
 * - CHANY nodes when xhigh="0" and xlow="0"
 * - CHANX nodes when yhigh="0" and ylow="0"
 * 
 * The nodes themselves are kept in the graph.
 * 
 * Usage: rr_graph_transform input.xml output.xml
 */

#include <iostream>
#include <fstream>
#include <unordered_set>
#include <vector>
#include <string>
#include <chrono>
#include <cstring>

#include "pugixml.hpp"

/**
 * @brief Transform RR graph by removing edges connected to boundary nodes
 * 
 * @param input_file Path to input XML file
 * @param output_file Path to output XML file
 * @return true if transformation succeeded
 */
bool transform_rr_graph(const char* input_file, const char* output_file) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Parse the XML file
    pugi::xml_document doc;
    pugi::xml_parse_result result = doc.load_file(input_file);
    
    if (!result) {
        std::cerr << "Error: Failed to parse " << input_file << ": " << result.description() << std::endl;
        return false;
    }
    
    auto parse_time = std::chrono::high_resolution_clock::now();
    std::cout << "XML parsing completed in " 
              << std::chrono::duration<double>(parse_time - start_time).count() 
              << " seconds" << std::endl;
    
    // Get the rr_graph root
    pugi::xml_node rr_graph = doc.child("rr_graph");
    if (!rr_graph) {
        std::cerr << "Error: No rr_graph root element found" << std::endl;
        return false;
    }
    
    // Set to store IDs of nodes whose edges should be removed
    std::unordered_set<std::string> target_node_ids;
    
    // Find rr_nodes section
    pugi::xml_node rr_nodes = rr_graph.child("rr_nodes");
    if (!rr_nodes) {
        std::cerr << "Error: No rr_nodes section found" << std::endl;
        return false;
    }
    
    // First pass: identify target nodes and collect their IDs
    for (pugi::xml_node node = rr_nodes.child("node"); node; node = node.next_sibling("node")) {
        const char* node_type = node.attribute("type").value();
        const char* node_id = node.attribute("id").value();
        
        if (!node_type || !node_id || node_type[0] == '\0' || node_id[0] == '\0') {
            continue;
        }
        
        pugi::xml_node loc = node.child("loc");
        if (!loc) {
            continue;
        }
        
        const char* xhigh = loc.attribute("xhigh").value();
        const char* xlow = loc.attribute("xlow").value();
        const char* yhigh = loc.attribute("yhigh").value();
        const char* ylow = loc.attribute("ylow").value();
        
        // Check CHANY nodes with xhigh="0" and xlow="0"
        if (strcmp(node_type, "CHANY") == 0 && 
            strcmp(xhigh, "0") == 0 && strcmp(xlow, "0") == 0) {
            target_node_ids.insert(node_id);
        }
        // Check CHANX nodes with yhigh="0" and ylow="0"
        else if (strcmp(node_type, "CHANX") == 0 && 
                 strcmp(yhigh, "0") == 0 && strcmp(ylow, "0") == 0) {
            target_node_ids.insert(node_id);
        }
    }
    
    std::cout << "Found " << target_node_ids.size() << " target nodes to process" << std::endl;
    
    auto scan_time = std::chrono::high_resolution_clock::now();
    std::cout << "Node scanning completed in " 
              << std::chrono::duration<double>(scan_time - parse_time).count() 
              << " seconds" << std::endl;
    
    // Find rr_edges section and remove edges connected to target nodes
    pugi::xml_node rr_edges = rr_graph.child("rr_edges");
    if (!rr_edges) {
        std::cerr << "Error: No rr_edges section found" << std::endl;
        return false;
    }
    
    // Collect edges to remove
    std::vector<pugi::xml_node> edges_to_remove;
    
    for (pugi::xml_node edge = rr_edges.child("edge"); edge; edge = edge.next_sibling("edge")) {
        const char* sink_node = edge.attribute("sink_node").value();
        const char* src_node = edge.attribute("src_node").value();
        
        // Check if either sink or source node is in the target set
        if (target_node_ids.count(sink_node) > 0 || target_node_ids.count(src_node) > 0) {
            edges_to_remove.push_back(edge);
        }
    }
    
    std::cout << "Found " << edges_to_remove.size() << " edges to remove" << std::endl;
    
    auto edge_scan_time = std::chrono::high_resolution_clock::now();
    std::cout << "Edge scanning completed in " 
              << std::chrono::duration<double>(edge_scan_time - scan_time).count() 
              << " seconds" << std::endl;
    
    // Remove the identified edges
    for (pugi::xml_node& edge : edges_to_remove) {
        rr_edges.remove_child(edge);
    }
    
    auto remove_time = std::chrono::high_resolution_clock::now();
    std::cout << "Edge removal completed in " 
              << std::chrono::duration<double>(remove_time - edge_scan_time).count() 
              << " seconds" << std::endl;
    
    // Write the transformed XML to output file
    if (!doc.save_file(output_file, "  ", pugi::format_default | pugi::format_write_bom, pugi::encoding_utf8)) {
        std::cerr << "Error: Failed to write output file " << output_file << std::endl;
        return false;
    }
    
    auto write_time = std::chrono::high_resolution_clock::now();
    std::cout << "XML writing completed in " 
              << std::chrono::duration<double>(write_time - remove_time).count() 
              << " seconds" << std::endl;
    
    std::cout << "\n=== Summary ===" << std::endl;
    std::cout << "Removed " << edges_to_remove.size() << " edges" << std::endl;
    std::cout << "Total time: " 
              << std::chrono::duration<double>(write_time - start_time).count() 
              << " seconds" << std::endl;
    
    return true;
}

void print_usage(const char* program_name) {
    std::cout << "Usage: " << program_name << " <input.xml> <output.xml>" << std::endl;
    std::cout << std::endl;
    std::cout << "Transform RR graph by removing edges connected to boundary nodes:" << std::endl;
    std::cout << "  - CHANY nodes with xhigh=0 and xlow=0" << std::endl;
    std::cout << "  - CHANX nodes with yhigh=0 and ylow=0" << std::endl;
}

int main(int argc, char** argv) {
    if (argc != 3) {
        print_usage(argv[0]);
        return 1;
    }
    
    const char* input_file = argv[1];
    const char* output_file = argv[2];
    
    std::cout << "RR Graph Transform Tool" << std::endl;
    std::cout << "Input:  " << input_file << std::endl;
    std::cout << "Output: " << output_file << std::endl;
    std::cout << std::endl;
    
    if (transform_rr_graph(input_file, output_file)) {
        std::cout << "\nTransformation completed successfully!" << std::endl;
        return 0;
    } else {
        std::cerr << "\nTransformation failed!" << std::endl;
        return 1;
    }
}

