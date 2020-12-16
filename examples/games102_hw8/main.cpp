// Contribution by: Abe Tusk https://github.com/abetusk
// To compile:
// gcc -Wall -Weverything -Wno-float-equal src/examples/simple.c -Isrc -o simple
//
// About:
//
// This example outputs 10 random 2D coordinates, and all the generated edges, to standard output.
// Note that the edges have duplicates, but you can easily filter them out.
//

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define JC_VORONOI_IMPLEMENTATION
#include "jc_voronoi.h"
#include <vector>

int main(int argc, char** argv)
{
    std::vector<jcv_point> points(20);
    srand(0);
    for (size_t i = 0; i < points.size(); i++)
    {
        points[i].x = (float)(rand() / (1.0f + RAND_MAX));
        points[i].y = (float)(rand() / (1.0f + RAND_MAX));
    }

    jcv_diagram diagram;
    memset(&diagram, 0, sizeof(jcv_diagram));
    jcv_rect bounding_box = {{0.0f, 0.0f}, {1.0f, 1.0f}};
    jcv_diagram_generate(points.size(), points.data(), &bounding_box, 0, &diagram);





    const jcv_site* sites;
    jcv_graphedge* graph_edge;
    printf("# Edges\n");
    sites = jcv_diagram_get_sites(&diagram);
    for (size_t i = 0; i < diagram.numsites; i++)
    {
        graph_edge = sites[i].edges;
        while (graph_edge)
        {
            // This approach will potentially print shared edges twice
            printf("< %f %f , ", (double)graph_edge->pos[0].x, (double)graph_edge->pos[0].y);
            printf("%f %f >\t", (double)graph_edge->pos[1].x, (double)graph_edge->pos[1].y);
            graph_edge = graph_edge->next;
        }
        printf("\n");
    }

    jcv_diagram_free(&diagram);
}
