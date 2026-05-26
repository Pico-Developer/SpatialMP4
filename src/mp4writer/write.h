#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#include <filesystem>

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <libavutil/pixfmt.h>
#include <libavutil/pixdesc.h>
#include <libavutil/log.h>
#include <libavutil/dict.h>
}

class MP4Writer {
public:
    MP4Writer();
    ~MP4Writer();
    int Write(const std::string& input_file, const std::string& output_file);
    void addTrack(char* mime_type);
    void closeFile();
    void writeSample();
private:
    AVFormatContext *fmt_ctx;
};