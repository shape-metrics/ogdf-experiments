#ifndef OGDF_DRAWER_H
#define OGDF_DRAWER_H

#include <domus/core/graph/graph.hpp>
#include <domus/orthogonal/drawing.hpp>
#include <string>
#include <utility>

std::pair<OrthogonalDrawing, double> make_orthogonal_drawing_ogdf(
    const UndirectedSimpleGraph& graph,
    const std::string& svg_output_filename);

#endif
