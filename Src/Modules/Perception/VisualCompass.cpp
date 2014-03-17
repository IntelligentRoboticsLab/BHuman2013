#include "VisualCompass.h"

// For debugging
#include "Tools/Debugging/Debugging.h"

MAKE_MODULE(VisualCompass, Perception)

void VisualCompass::update(VisualPole&) {
    OUTPUT(idText, text, "ViCTOriA here!");
}
