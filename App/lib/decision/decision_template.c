#include "decision_template.h"

#include <stddef.h>

static const DecisionFixedLayout Figure2CanonicalLayout = {
    .piece_count = 4U,
    .pieces = {
        {
            .id = DECISION_FIGURE2_PIECE_LARGE_TRIANGLE,
            .vertex_count = 3U,
            .target_vertices = {{20.0f, 0.0f},
                                {100.0f, 0.0f},
                                {100.0f, 60.0f}}
        },
        {
            .id = DECISION_FIGURE2_PIECE_TOP_LEFT,
            .vertex_count = 4U,
            .target_vertices = {{0.0f, 0.0f},
                                {20.0f, 0.0f},
                                {36.0f, 12.0f},
                                {0.0f, 20.0f}}
        },
        {
            .id = DECISION_FIGURE2_PIECE_MIDDLE,
            .vertex_count = 4U,
            .target_vertices = {{0.0f, 20.0f},
                                {36.0f, 12.0f},
                                {76.0f, 42.0f},
                                {0.0f, 30.0f}}
        },
        {
            .id = DECISION_FIGURE2_PIECE_BOTTOM,
            .vertex_count = 4U,
            .target_vertices = {{0.0f, 30.0f},
                                {76.0f, 42.0f},
                                {100.0f, 60.0f},
                                {0.0f, 60.0f}}
        }
    }
};

void DecisionTemplate_GetFigure2Layout(DecisionPoint target_center,
                                       DecisionFixedLayout *layout)
{
    float offset_x;
    float offset_y;
    uint8_t piece_index;

    if (layout == NULL) {
        return;
    }

    *layout = Figure2CanonicalLayout;
    offset_x = target_center.x_mm - 0.5f * DECISION_FIGURE2_WIDTH_MM;
    offset_y = target_center.y_mm - 0.5f * DECISION_FIGURE2_HEIGHT_MM;

    for (piece_index = 0U; piece_index < layout->piece_count; ++piece_index) {
        uint8_t vertex_index;

        for (vertex_index = 0U;
             vertex_index < layout->pieces[piece_index].vertex_count;
             ++vertex_index) {
            layout->pieces[piece_index].target_vertices[vertex_index].x_mm +=
                offset_x;
            layout->pieces[piece_index].target_vertices[vertex_index].y_mm +=
                offset_y;
        }
    }
}
