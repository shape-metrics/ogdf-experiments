#include "ogdf-drawer.hpp"

#include <ogdf/basic/Graph.h>
#include <ogdf/basic/GraphAttributes.h>
#include <ogdf/basic/GraphList.h>
#include <ogdf/basic/LayoutStatistics.h>
#include <ogdf/fileformats/GraphIO.h>
#include <ogdf/orthogonal/OrthoLayout.h>
#include <ogdf/planarity/EmbedderMinDepthMaxFaceLayers.h>
#include <ogdf/planarity/PlanarSubgraphFast.h>
#include <ogdf/planarity/PlanarizationLayout.h>
#include <ogdf/planarity/RemoveReinsertType.h>
#include <ogdf/planarity/SubgraphPlanarizer.h>
#include <ogdf/planarity/VariableEmbeddingInserter.h>

#include <cassert>
#include <chrono>
#include <cmath>
#include <domus/core/utils.hpp>
#include <domus/orthogonal/shape/shape.hpp>
#include <map>
#include <string>

constexpr double grid = 100.0;

int snap_coordinate(double v) {
    return static_cast<int>(std::round(v * grid));
}

Shape compute_shape(
    const UndirectedSimpleGraph& augmented_graph,
    const GraphAttributes& attributes) {
    Shape shape;
    for (const GraphEdge& edge : augmented_graph.get_edges()) {
        const int from_id = edge.get_from_id();
        const int to_id = edge.get_to_id();
        const int x_from = attributes.get_position_x(from_id);
        const int y_from = attributes.get_position_y(from_id);
        const int x_to = attributes.get_position_x(to_id);
        const int y_to = attributes.get_position_y(to_id);
        if (x_from == x_to) {
            assert(y_from != y_to);
            if (y_from < y_to) {
                shape.set_direction(from_id, to_id, Direction::UP);
                shape.set_direction(to_id, from_id, Direction::DOWN);
            } else {
                shape.set_direction(from_id, to_id, Direction::DOWN);
                shape.set_direction(to_id, from_id, Direction::UP);
            }
        } else {
            assert(x_from != x_to);
            assert(y_from == y_to);
            if (x_from < x_to) {
                shape.set_direction(from_id, to_id, Direction::RIGHT);
                shape.set_direction(to_id, from_id, Direction::LEFT);
            } else {
                shape.set_direction(from_id, to_id, Direction::LEFT);
                shape.set_direction(to_id, from_id, Direction::RIGHT);
            }
        }
    }
    return shape;
}

void compute_augmented_graph_and_positions(
    const UndirectedSimpleGraph& graph,
    const ogdf::GraphAttributes& GA,
    const ogdf::Graph& G,
    std::unordered_map<int, int>& ogdf_index_to_nodeid,
    UndirectedSimpleGraph& augmented_graph,
    std::unordered_map<int, std::pair<int, int>>& id_to_ogdf_positions) {
    for (const int node_id : graph.get_nodes_ids())
        augmented_graph.add_node(node_id);
    for (ogdf::node v : G.nodes) {
        const int x = snap_coordinate(GA.x(v));
        const int y = snap_coordinate(GA.y(v));
        id_to_ogdf_positions[ogdf_index_to_nodeid[v->index()]] = {x, y};
    }
    for (ogdf::edge e : G.edges) {
        if (GA.bends(e).size() <= 2) {  // handling edges without bends
            const int from_id = ogdf_index_to_nodeid[e->source()->index()];
            const int to_id = ogdf_index_to_nodeid[e->target()->index()];
            augmented_graph.add_edge(from_id, to_id);
        } else {  // handling edges with bends
            int from_id = ogdf_index_to_nodeid[e->source()->index()];
            const int to_id = ogdf_index_to_nodeid[e->target()->index()];
            std::vector<ogdf::DPoint> bend_vec;
            for (auto& elem : GA.bends(e))
                bend_vec.push_back(elem);
            for (int j = 1; j < bend_vec.size() - 1; ++j) {
                const int node_id = augmented_graph.add_node().get_id();
                augmented_graph.add_edge(from_id, node_id);
                from_id = node_id;
                const int x = snap_coordinate(bend_vec[j].m_x);
                const int y = snap_coordinate(bend_vec[j].m_y);
                id_to_ogdf_positions[node_id] = {x, y};
            }
            augmented_graph.add_edge(from_id, to_id);
        }
    }
}

auto compute_coordinate(
    const std::map<int, std::vector<int>>& coordinate_to_ids) {
    constexpr int THRESHOLD = 7500;
    int last_ogdf_coordinate = 0;
    int coordinate_to_use = 0;
    std::unordered_map<int, int> new_positions_coordinate;
    for (const auto& [coordinate, ids] : coordinate_to_ids) {
        if (coordinate - last_ogdf_coordinate > THRESHOLD)
            coordinate_to_use += grid;
        for (const int id : ids)
            new_positions_coordinate[id] = coordinate_to_use;
        last_ogdf_coordinate = coordinate;
    }
    return new_positions_coordinate;
}

GraphAttributes compute_graph_attributes(
    const UndirectedSimpleGraph& graph,
    UndirectedSimpleGraph& augmented_graph,
    std::unordered_map<int, std::pair<int, int>>& id_to_ogdf_positions) {
    std::map<int, std::vector<int>> x_coor_to_ids, y_coor_to_ids;
    for (const int node_id : augmented_graph.get_nodes_ids()) {
        const int x = id_to_ogdf_positions[node_id].first;
        const int y = id_to_ogdf_positions[node_id].second;
        x_coor_to_ids[x].push_back(node_id);
        y_coor_to_ids[y].push_back(node_id);
    }
    auto new_positions_x = compute_coordinate(x_coor_to_ids);
    auto new_positions_y = compute_coordinate(y_coor_to_ids);
    GraphAttributes attributes;
    attributes.add_attribute(Attribute::NODES_POSITION);
    attributes.add_attribute(Attribute::NODES_COLOR);
    for (const int node_id : augmented_graph.get_nodes_ids()) {
        attributes.set_position(
            node_id, new_positions_x[node_id], new_positions_y[node_id]);
        if (graph.has_node(node_id))  // is a true node
            attributes.set_node_color(node_id, Color::BLACK);
        else  // is a bend
            attributes.set_node_color(node_id, Color::RED);
    }
    return attributes;
}

void make_shifts_overlapping_edges(
    UndirectedSimpleGraph& augmented_graph,
    const GraphAttributes& attributes,
    Shape& shape) {
    // TODO
    // std::map<int, std::vector<int>> x_coor_to_ids;
    // std::map<int, std::vector<int>> y_coor_to_ids;
    // for (const int node_id : augmented_graph.get_nodes_ids()) {
    //     const int x = attributes.get_position_x(node_id);
    //     const int y = attributes.get_position_y(node_id);
    //     x_coor_to_ids[x].push_back(node_id);
    //     y_coor_to_ids[y].push_back(node_id);
    // }
    // for (const auto& [x, nodes_ids] : x_coor_to_ids) {
    //     std::sort(
    //         nodes_ids.begin(),
    //         nodes_ids.end(),
    //         [&attributes](int node_1_id, int node_2_id) {
    //             return attributes.get_position_y(node_1_id) <
    //                    attributes.get_position_y(node_2_id);
    //         });
    //     std::unordered_map<int, std::vector<int>>
    //     order_of_nodes_per_x_offset; for (const node_id : nodes_ids) {
    //         if (attributes.get_node_color(node_id) == Color::BLACK)
    //             order_of_nodes_per_x_offset[0].push_back(node_id);
    //         else {
    //         }
    //     }
    // }
}

OrthogonalDrawing convert_ogdf_result(
    const ogdf::GraphAttributes& GA,
    const ogdf::Graph& G,
    const UndirectedSimpleGraph& graph,
    std::unordered_map<int, int>& ogdf_index_to_nodeid) {
    auto augmented_graph = std::make_unique<UndirectedSimpleGraph>();
    std::unordered_map<int, std::pair<int, int>> id_to_ogdf_positions;
    compute_augmented_graph_and_positions(
        graph,
        GA,
        G,
        ogdf_index_to_nodeid,
        *augmented_graph,
        id_to_ogdf_positions);
    GraphAttributes attributes =
        compute_graph_attributes(graph, *augmented_graph, id_to_ogdf_positions);
    Shape shape = compute_shape(*augmented_graph, attributes);
    make_shifts_overlapping_edges(*augmented_graph, attributes, shape);
    return {std::move(augmented_graph), attributes, shape};
}

std::pair<OrthogonalDrawing, double> make_orthogonal_drawing_ogdf(
    const UndirectedSimpleGraph& graph,
    const std::string& svg_output_filename) {
    ogdf::Graph G;
    ogdf::GraphAttributes GA(
        G,
        ogdf::GraphAttributes::nodeGraphics | ogdf::GraphAttributes::nodeType |
            ogdf::GraphAttributes::edgeGraphics |
            ogdf::GraphAttributes::edgeType | ogdf::GraphAttributes::nodeLabel |
            ogdf::GraphAttributes::nodeStyle |
            ogdf::GraphAttributes::nodeTemplate);
    std::unordered_map<int, ogdf::node> nodeid_to_ogdf_node;
    std::unordered_map<int, int> ogdf_index_to_nodeid;
    for (const int node_id : graph.get_nodes_ids()) {
        nodeid_to_ogdf_node[node_id] = G.newNode(node_id);
        ogdf_index_to_nodeid[nodeid_to_ogdf_node[node_id]->index()] = node_id;
    }
    for (const GraphEdge& edge : graph.get_edges()) {
        const int i = edge.get_from_id();
        const int j = edge.get_to_id();
        G.newEdge(nodeid_to_ogdf_node[i], nodeid_to_ogdf_node[j]);
    }
    for (ogdf::node v : G.nodes)
        GA.label(v) = std::to_string(v->index());

    auto start = std::chrono::high_resolution_clock::now();

    ogdf::PlanarizationLayout pl;
    ogdf::SubgraphPlanarizer* crossMin = new ogdf::SubgraphPlanarizer;
    ogdf::PlanarSubgraphFast<int>* ps = new ogdf::PlanarSubgraphFast<int>;
    ps->runs(100);
    ogdf::VariableEmbeddingInserter* ves = new ogdf::VariableEmbeddingInserter;
    ves->removeReinsert(ogdf::RemoveReinsertType::All);

    crossMin->setSubgraph(ps);
    crossMin->setInserter(ves);
    pl.setCrossMin(crossMin);

    ogdf::EmbedderMinDepthMaxFaceLayers* emb =
        new ogdf::EmbedderMinDepthMaxFaceLayers;
    pl.setEmbedder(emb);

    ogdf::OrthoLayout* ol = new ogdf::OrthoLayout;
    ol->separation(100.0);
    ol->cOverhang(0.2);
    pl.setPlanarLayouter(ol);

    ogdf::setSeed(0);
    pl.call(GA);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;

    ogdf::GraphIO::write(GA, svg_output_filename, ogdf::GraphIO::drawSVG);
    OrthogonalDrawing result =
        convert_ogdf_result(GA, G, graph, ogdf_index_to_nodeid);
    return std::make_pair(std::move(result), elapsed.count());
}