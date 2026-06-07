#include <iostream>
#include "spatialmp4/writer.h"

int main() {
    std::cout << "Hello, World!" << std::endl;
    MP4Writer *writer = new MP4Writer();
    writer->addTrack("video/raw");
    writer->writeSample();
    delete writer;
    return 0;
}
