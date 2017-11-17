#include <chrono>
#include "MapPredictor.h"
#include "MapDrawer.h"

int main() {
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();

    MapPredictor predictor = MapPredictor("../../HTWK_MapPredictor/signs.xml");
    predictor.getPredictedMap();

    MapDrawer drawer = MapDrawer(predictor.getSignsMap(), predictor.getTilesMap(), predictor.getBounds());
    drawer.drawMap(true);

    // measure runtime
    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
    std::cout << std::endl << duration / 1000.0 << " ms" << std::endl;
}