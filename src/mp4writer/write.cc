#include "mp4writer/write.h"


using namespace std;

string output_file = "output.mp4";
uint8_t tmp_data1[10] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09};
uint8_t tmp_data2[10] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09};



uint8_t distortion[40] = {0x52, 0x83, 0x9A, 0x90, 0x14, 0x39, 0x6E,
        0x3F, 0xAD, 0x56, 0x07, 0x0A, 0xBF, 0xE0, 0xAA, 0x3F, 0xF2, 0xEF, 
        0x6C, 0xB3, 0x70, 0xA7, 0x36, 0x3F, 0xA1, 0x4F, 0x5E, 0xFF, 0xAB, 
        0x18, 0x3A, 0xBF, 0x7C, 0xE7, 0xE4, 0x76, 0xF2, 0xA5, 0xAB, 0xBF};


uint8_t ecam[] = {
            0xE4, 0xD3, 0x60, 0x20, 0xE7, 0x63, 0x79, 0xBF, 0x46, 0x56, 0x53, 0x11, 0xCD, 0xFF, 0xEF, 
            0xBF, 0xA6, 0xD2, 0x65, 0x3B, 0x6D, 0x18, 0x6A, 0x3F, 0x88, 0x80, 0x25, 0x45, 0x60, 0xA6, 
            0x95, 0xBF, 0x76, 0xE7, 0xA7, 0xB1, 0xC8, 0xFF, 0xEF, 0xBF, 0x1A, 0x8A, 0xCA, 0x86, 0x76, 
            0x70, 0x79, 0x3F, 0x69, 0x31, 0x63, 0xCE, 0x35, 0xD6, 0x6E, 0x3F, 0x3B, 0xCA, 0xBF, 0x04, 
            0x5D, 0x46, 0xB5, 0x3F, 0x30, 0xF7, 0xA8, 0x50, 0x82, 0xFF, 0x6E, 0xBF, 0x40, 0x3B, 0x94, 
            0xE2, 0x50, 0xE7, 0x69, 0xBF, 0xF6, 0x5A, 0x74, 0x80, 0xE6, 0xFF, 0xEF, 0xBF, 0x18, 0xC1, 
            0x5C, 0x53, 0x72, 0x21, 0x91, 0xBF, 0xE5, 0xD3, 0x60, 0x20, 0xE7, 0x63, 0x79, 0xBF, 0x43, 
            0x56, 0x53, 0x11, 0xCD, 0xFF, 0xEF, 0xBF, 0xA6, 0xD2, 0x65, 0x3B, 0x6D, 0x18, 0x6A, 0x3F, 
            0x9C, 0x78, 0x64, 0x2D, 0x79, 0x0E, 0x96, 0xBF, 0x74, 0xE7, 0xA7, 0xB1, 0xC8, 0xFF, 0xEF, 
            0xBF, 0x17, 0x8A, 0xCA, 0x86, 0x76, 0x70, 0x79, 0x3F, 0x67, 0x31, 0x63, 0xCE, 0x35, 0xD6, 
            0x6E, 0x3F, 0xC4, 0xA9, 0xD6, 0xDF, 0xBB, 0x80, 0x93, 0x3F, 0x2D, 0xF7, 0xA8, 0x50, 0x82, 
            0xFF, 0x6E, 0xBF, 0x3F, 0x3B, 0x94, 0xE2, 0x50, 0xE7, 0x69, 0xBF, 0xF1, 0x5A, 0x74, 0x80, 
            0xE6, 0xFF, 0xEF, 0xBF, 0x7E, 0x70, 0xFD, 0xB1, 0xFD, 0x60, 0x91, 0xBF };
uint8_t icam[] = {
            0xD6, 0x8D, 0x3A, 0x03, 0x2E, 0x23, 0x70, 0x40, 0x01, 0x91, 0x3D, 0x13, 0xDF, 0x52, 0x70, 
            0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x69, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 
            0x63, 0x40, 0xD6, 0x8D, 0x3A, 0x03, 0x2E, 0x23, 0x70, 0x40, 0x01, 0x91, 0x3D, 0x13, 0xDF, 
            0x52, 0x70, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x69, 0x40, 0x00, 0x00, 0x00, 0x00, 
            0x00, 0x30, 0x63, 0x40 };   

static void log_callback_test2(void *ptr, int level, const char *fmt, va_list vl) {
    va_list vl2;
    char *line = (char *) malloc(128 * sizeof(char));
    static int print_prefix = 1;
    va_copy(vl2, vl);
    av_log_format_line(ptr, level, fmt, vl2, line, 128, &print_prefix);
    va_end(vl2);
    line[127] = '\0';
    cout << line << endl;
    free(line);
}
MP4Writer::MP4Writer() {


    av_log_set_callback(log_callback_test2);

    AVOutputFormat *ofmt = NULL;
    if(avformat_alloc_output_context2(&fmt_ctx, NULL, NULL, output_file.c_str())) {
        cout << "Could not create output context" << endl;
        
    }
    cout << "create output context success" << endl;
}

void MP4Writer::writeSample(){
    AVPacket *packet = av_packet_alloc();
    packet->stream_index = 0;
    packet->data = tmp_data1;
    packet->size = 10;
    packet->pts = 1000;
    packet->dts = 1000;
    packet->flags = AV_PKT_FLAG_KEY;
    av_interleaved_write_frame(fmt_ctx, packet);
    av_packet_free(&packet);

    AVPacket *packet2 = av_packet_alloc();
    packet2->stream_index = 1;
    packet2->data = tmp_data2;
    packet2->size = 10;
    packet2->pts = 1000;
    packet2->dts = 1000;
    packet2->flags = AV_PKT_FLAG_KEY;
    av_interleaved_write_frame(fmt_ctx, packet2);
    av_packet_free(&packet2);

}

void MP4Writer::addTrack(char* mime_type){
    if (fmt_ctx) {
        // raw1 track
        AVStream *stream1 = avformat_new_stream(fmt_ctx, NULL);
        if (!stream1) {
            cout << "Failed to create new stream" << endl;
        }
        AVCodecParameters *codecpar = stream1->codecpar;
        codecpar->codec_type = AVMEDIA_TYPE_VIDEO;
        //codecpar->codec_id = AV_CODEC_ID_H264;
        //codecpar->format = AV_PIX_FMT_YUV420P;
        codecpar->codec_tag = MKTAG('r', 'a', 'w', '1');
        codecpar->width = 1920;
        codecpar->height = 1080;

        av_dict_set_int(&stream1->metadata, "data_accuracy", 2, 0);
        av_dict_set_int(&stream1->metadata, "depth_legal_range", 1000, 0);
        av_dict_set(&stream1->metadata, "depth_data_precision", "dtmm", 0);
        av_dict_set_int(&stream1->metadata, "hfov", 87, 0);
        av_dict_set_int(&stream1->metadata, "vfov", 71, 0);

                  
        //av_dict_set(&stream1->metadata, "distortion_coefficients", (char*)distortion, 0);
        av_dict_set(&stream1->metadata, "distortion_model", "brown", 0);
        av_dict_set(&stream1->metadata, "camera_model", "pinhole", 0);
        //av_dict_set(&stream1->metadata, "ecam", (char*)ecam, 0);
        //av_dict_set(&stream1->metadata, "icam", (char*)icam, 0);
        av_dict_set(&stream1->metadata, "cam_count", "2", 0);
        
        AVPacketSideData *side_data_ecam = 
        av_packet_side_data_new(&stream1->codecpar->coded_side_data, &stream1->codecpar->nb_coded_side_data,
        AV_PKT_DATA_ECAM, sizeof(ecam), 0);
        memcpy(side_data_ecam->data, ecam, sizeof(ecam));
        AVPacketSideData *side_data_icam = 
        av_packet_side_data_new(&stream1->codecpar->coded_side_data, &stream1->codecpar->nb_coded_side_data,
        AV_PKT_DATA_ICAM, sizeof(icam), 0);
        memcpy(side_data_icam->data, icam, sizeof(icam));
        AVPacketSideData *side_data_distortion = 
        av_packet_side_data_new(&stream1->codecpar->coded_side_data, &stream1->codecpar->nb_coded_side_data,
        AV_PKT_DATA_DISTORTION_COEFFICIENTS, sizeof(distortion), 0);
        memcpy(side_data_distortion->data, distortion, sizeof(distortion));
        


        // pose track

        AVStream *stream2 = avformat_new_stream(fmt_ctx, NULL);
        if (!stream2) {
            cout << "Failed to create new stream" << endl;
        }
        AVCodecParameters *codecpar2 = stream2->codecpar;
        codecpar2->codec_type = AVMEDIA_TYPE_DATA;
        codecpar2->codec_tag = MKTAG('m', 'e', 't', 't');
        //codecpar2->codec_id = AV_CODEC_ID_METT_DISPARITY;

        
        av_dict_set_int(&stream2->metadata, "pose_coordinate", 1, 0);
        av_dict_set_int(&stream2->metadata, "data_accuracy", 2, 0);
        av_dict_set(&stream2->metadata, "pose_position", "head", 0);
        av_dict_set(&stream2->metadata, "mime_type", "application/pose", 0);
        av_dict_set_int(&stream2->metadata, "track_base_time", 1748504166622934, 0);
        

        if(avio_open(&fmt_ctx->pb, output_file.c_str(), AVIO_FLAG_WRITE) < 0) {
            cout << "Could not open output file" << endl;
            return;
        }
        if(avformat_write_header(fmt_ctx, NULL) < 0) {
            cout << "write header failed" << endl;
            return;
        }
    }
}

MP4Writer::~MP4Writer() {
    if(fmt_ctx) {
        cout << "write trailer" << endl;
        av_write_trailer(fmt_ctx);
        cout << "write trailer success" << endl;
        avio_close(fmt_ctx->pb);
        cout << "close io success" << endl;
        avformat_free_context(fmt_ctx);
        cout << "free context success" << endl;
    }
}

   using namespace std;



