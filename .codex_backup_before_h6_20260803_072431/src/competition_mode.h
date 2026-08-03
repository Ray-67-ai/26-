#ifndef COMPETITION_MODE_H
#define COMPETITION_MODE_H

/* Runtime question selection. One firmware image serves all questions. */
typedef enum {
    COMPETITION_MODE_NONE = 0,
    COMPETITION_MODE_H2 = 2,
    COMPETITION_MODE_H3 = 3,
    COMPETITION_MODE_H4 = 4,
    COMPETITION_MODE_H5 = 5
} competition_mode_t;

#endif
