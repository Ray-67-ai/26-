#ifndef COMPETITION_MODE_H
#define COMPETITION_MODE_H

/* Runtime question selection. This H4 integration serves H2/H3/H4. */
typedef enum {
    COMPETITION_MODE_NONE = 0,
    COMPETITION_MODE_H2 = 2,
    COMPETITION_MODE_H3 = 3,
    COMPETITION_MODE_H4 = 4
} competition_mode_t;

#endif
