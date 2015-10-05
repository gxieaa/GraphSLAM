#include "util_functs.h"

double max_from_four (double n1, double n2, double n3, double n4) {
    return fmax(fmax (n1, n2), fmax(n3, n4));
}
