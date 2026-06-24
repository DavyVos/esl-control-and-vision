#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/mman.h>
#include <unistd.h>
#include <pthread.h>
#include <gst/gst.h>
#include <gst/app/app.h>
#include "jiwy_controller.h"
#include "soc_system.h"
#include <glib.h>
#include <string.h>
#include <signal.h>
#include <stdatomic.h>

#define WIDTH 320
#define HEIGHT 240

typedef struct
{
    int fd;
    uint32_t *encoderTilt;
    uint32_t *encoderPan;
    uint32_t *pwmTilt;
    uint32_t *pwmPan;
    Jiwy *jiwy;
} CleanupCtx;

static GstElement *pipeline_global = NULL;
static GstElement *appsrc_global = NULL;

static pthread_t g_ctrl_thread;

static void sigint_handler(int sig)
{
    (void)sig;
    pthread_cancel(g_ctrl_thread); /* triggers cleanup handlers in the thread */

    if (appsrc_global)
        gst_app_src_end_of_stream(GST_APP_SRC(appsrc_global));
    if (pipeline_global)
        gst_element_send_event(pipeline_global, gst_event_new_eos());
}

static atomic_int g_x_deviation = 0;
static atomic_int g_y_deviation = 0;
static atomic_bool g_target_detected = false;

void get_deviation(int *x, int *y)
{
    *x = atomic_load(&g_x_deviation);
    *y = atomic_load(&g_y_deviation);
}

static gboolean bus_call(GstBus *bus, GstMessage *msg, gpointer data)
{
    GMainLoop *loop = (GMainLoop *)data;
    switch (GST_MESSAGE_TYPE(msg))
    {
    case GST_MESSAGE_EOS:
        g_print("End of stream\n");
        g_main_loop_quit(loop);
        break;
    case GST_MESSAGE_ERROR:
    {
        gchar *debug;
        GError *error;
        gst_message_parse_error(msg, &error, &debug);
        g_free(debug);
        g_printerr("Error: %s\n", error->message);
        g_error_free(error);
        g_main_loop_quit(loop);
        break;
    }
    default:
        break;
    }
    return TRUE;
}

typedef union
{
    struct
    {
        unsigned int B : 8;
        unsigned int G : 8;
        unsigned int R : 8;
        unsigned int x : 8;
    };
    unsigned int data : 32;
} pixel_t;

static int is_green(pixel_t p)
{
    return (p.G > p.R + 10) && (p.G > p.B + 10) && (p.G > 30);
}

static int is_red(pixel_t p)
{
    return (p.R > 30) && (p.R > p.B + 10) && (p.R > p.G + 10);
}

static void control_cleanup(void *arg)
{
    CleanupCtx *ctx = (CleanupCtx *)arg;
    printf("Control loop shutting down...\n");
    Jiwy_Disable(ctx->jiwy);
    munmap(ctx->encoderTilt, HPS_0_ARM_A9_0_ENCODER_BUS_0_SPAN);
    munmap(ctx->encoderPan, HPS_0_ARM_A9_0_ENCODER_BUS_1_SPAN);
    munmap(ctx->pwmTilt, HPS_0_ARM_A9_0_PWMBUS_0_SPAN);
    munmap(ctx->pwmPan, HPS_0_ARM_A9_0_PWMBUS_1_SPAN);
    close(ctx->fd);
    printf("Control loop stopped.\n");
}

static void *control_loop(void *arg)
{
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0)
    {
        perror("Couldn't open /dev/mem");
        return NULL;
    }

    uint32_t *encoderTilt = mmap(NULL, HPS_0_ARM_A9_0_ENCODER_BUS_0_SPAN,
                                 PROT_READ | PROT_WRITE, MAP_SHARED, fd,
                                 HPS_0_ARM_A9_0_ENCODER_BUS_0_BASE);
    uint32_t *encoderPan = mmap(NULL, HPS_0_ARM_A9_0_ENCODER_BUS_1_SPAN,
                                PROT_READ | PROT_WRITE, MAP_SHARED, fd,
                                HPS_0_ARM_A9_0_ENCODER_BUS_1_BASE);
    uint32_t *pwmTilt = mmap(NULL, HPS_0_ARM_A9_0_PWMBUS_0_SPAN,
                             PROT_READ | PROT_WRITE, MAP_SHARED, fd,
                             HPS_0_ARM_A9_0_PWMBUS_0_BASE);
    uint32_t *pwmPan = mmap(NULL, HPS_0_ARM_A9_0_PWMBUS_1_SPAN,
                            PROT_READ | PROT_WRITE, MAP_SHARED, fd,
                            HPS_0_ARM_A9_0_PWMBUS_1_BASE);

    if (encoderTilt == MAP_FAILED || encoderPan == MAP_FAILED ||
        pwmTilt == MAP_FAILED || pwmPan == MAP_FAILED)
    {
        perror("Couldn't map peripherals");
        close(fd);
        return NULL;
    }

    Jiwy jiwy;
    Jiwy_Init(&jiwy, encoderPan, encoderTilt, pwmPan, pwmTilt);
    Jiwy_CalibrateTilt(&jiwy);
    Jiwy_CalibratePan(&jiwy);

    jiwy.tilt_target = 0.0;
    jiwy.pan_target = 0.0;

    /* All locals are captured by the cleanup handler via a compound literal */
    CleanupCtx cleanup_args = { fd, encoderTilt, encoderPan, pwmTilt, pwmPan, &jiwy };
    pthread_cleanup_push(control_cleanup, &cleanup_args);

    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
    pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, NULL);

    int i = 0;
    while (1)
    {
        Jiwy_Update(&jiwy);

        if (atomic_load(&g_target_detected))
        {
            jiwy.pan_target += atomic_load(&g_x_deviation) * 0.1;
            jiwy.tilt_target += atomic_load(&g_y_deviation) * 0.1;
        }
        else
        {
            jiwy.pan_target = 0.0;
            jiwy.tilt_target = 0.0;
        }

        if (++i >= 1000)
        {
            printf("Tilt: %+.4f (target: %+.4f)  Pan: %+.4f (target: %+.4f)\n",
                   jiwy.tilt_current, jiwy.tilt_target,
                   jiwy.pan_current, jiwy.pan_target);
            i = 0;
        }

        usleep(100);
    }

    pthread_cleanup_pop(1);
    return NULL;
}

static GstFlowReturn on_new_sample(GstElement *appsink, gpointer user_data)
{
    GstElement *appsrc = (GstElement *)user_data;

    GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(appsink));
    GstBuffer *buffer = gst_sample_get_buffer(sample);
    //GstCaps *caps = gst_sample_get_caps(sample);

    /* Copy the buffer so we can write to it */
    GstBuffer *out_buf = gst_buffer_copy(buffer);
    gst_sample_unref(sample);

    GstMapInfo map;
    gst_buffer_map(out_buf, &map, GST_MAP_READWRITE);

    pixel_t *pixels = (pixel_t *)map.data;

    int x_center_mass = 0;
    int y_center_mass = 0;
    int green_count = 0;
    for (int y = 0; y < HEIGHT; y++)
    {
        for (int x = 0; x < WIDTH; x++)
        {
            pixel_t *p = &pixels[y * WIDTH + x];
            if (is_red(*p))
            {
                green_count++;
                x_center_mass += x;
                y_center_mass += y;
            }
            else
            {
                p->R = 0;
                p->G = 0;
                p->B = 0;
            }
        }
    }

    if (green_count > 0)
    {
        int x_center = x_center_mass / green_count;
        int y_center = y_center_mass / green_count;

        int x_deviation = x_center - WIDTH / 2;
        int y_deviation = y_center - HEIGHT / 2;

        atomic_store(&g_x_deviation, x_deviation);
        atomic_store(&g_y_deviation, y_deviation);
        atomic_store(&g_target_detected, true);

        float ratio = (float)green_count / (WIDTH * HEIGHT) * 100.0f;

        g_print("Red: %.1f%% Center: (%d,%d) Deviation: (%d,%d)\n",
                ratio, x_center, y_center,
                x_deviation, y_deviation);
    }
    else
    {
        g_print("Red: 0.0%% No object detected\n");
        atomic_store(&g_x_deviation, 0);
        atomic_store(&g_y_deviation, 0);
        atomic_store(&g_target_detected, false);
    }

    gst_buffer_unmap(out_buf, &map);

    /* Push masked frame into the encode pipeline */
    GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc), out_buf);

    return ret;
}

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        g_printerr("Usage: %s <device> [test]\n", argv[0]);
        return -1;
    }

    int test_mode = (argc >= 3 && strcmp(argv[2], "test") == 0);

    gst_init(&argc, &argv);

    /* ---- Capture pipeline ---- */
    GstElement *cap_pipeline, *src, *jpegdec, *convert, *appsink;

    cap_pipeline = gst_pipeline_new("capture-pipeline");
    src = gst_element_factory_make("v4l2src", "source");
    jpegdec = gst_element_factory_make("jpegdec", "jpeg-decoder");
    convert = gst_element_factory_make("videoconvert", "converter");
    appsink = gst_element_factory_make("appsink", "appsink");

    if (!cap_pipeline || !src || !jpegdec || !convert || !appsink)
    {
        g_printerr("Failed to create capture elements.\n");
        return -1;
    }

    g_object_set(G_OBJECT(src), "device", argv[1], NULL);

    GstCaps *src_caps = gst_caps_new_simple("image/jpeg",
                                            "width", G_TYPE_INT, WIDTH,
                                            "height", G_TYPE_INT, HEIGHT,
                                            "framerate", GST_TYPE_FRACTION, 10, 1,
                                            NULL);

    GstCaps *app_caps = gst_caps_new_simple("video/x-raw",
                                            "format", G_TYPE_STRING, "BGRx",
                                            "width", G_TYPE_INT, WIDTH,
                                            "height", G_TYPE_INT, HEIGHT,
                                            NULL);

    g_object_set(G_OBJECT(appsink),
                 "emit-signals", TRUE,
                 "drop", TRUE,
                 "max-buffers", 1,
                 "caps", app_caps,
                 NULL);

    /* ---- Encode pipeline ---- */
    GstElement *enc_pipeline, *appsrc, *enc_convert, *encoder, *muxer, *filesink;

    enc_pipeline = gst_pipeline_new("encode-pipeline");
    appsrc = gst_element_factory_make("appsrc", "appsrc");
    enc_convert = gst_element_factory_make("videoconvert", "enc-converter");
    encoder = gst_element_factory_make("x264enc", "encoder");
    muxer = gst_element_factory_make("mp4mux", "muxer");
    filesink = gst_element_factory_make("filesink", "filesink");

    if (!enc_pipeline || !appsrc || !enc_convert || !encoder || !muxer || !filesink)
    {
        g_printerr("Failed to create encode elements.\n");
        return -1;
    }

    g_object_set(G_OBJECT(appsrc),
                 "caps", app_caps, /* BGRx */
                 "format", GST_FORMAT_TIME,
                 "is-live", TRUE,
                 NULL);
    gst_caps_unref(app_caps);

    g_object_set(G_OBJECT(filesink), "location", test_mode ? "test.mp4" : "output.mp4", NULL);
    g_object_set(G_OBJECT(encoder),
                 "tune", 0x00000004,
                 "speed-preset", 3,
                 NULL);

    /* Connect appsink callback, pass appsrc as user_data */
    appsrc_global = appsrc;
    pipeline_global = cap_pipeline;
    signal(SIGINT, sigint_handler);

    g_signal_connect(appsink, "new-sample", G_CALLBACK(on_new_sample), appsrc);

    /* Build capture pipeline */
    gst_bin_add_many(GST_BIN(cap_pipeline), src, jpegdec, convert, appsink, NULL);
    gst_element_link_filtered(src, jpegdec, src_caps);
    gst_caps_unref(src_caps);
    gst_element_link(jpegdec, convert);
    gst_element_link(convert, appsink);

    /* Build encode pipeline */
    gst_bin_add_many(GST_BIN(enc_pipeline), appsrc, enc_convert, encoder, muxer, filesink, NULL);
    gst_element_link(appsrc, enc_convert);
    gst_element_link(enc_convert, encoder);
    gst_element_link(encoder, muxer);
    gst_element_link(muxer, filesink);

    /* Bus on encode pipeline to catch EOS and quit */
    GMainLoop *loop = g_main_loop_new(NULL, FALSE);
    GstBus *bus = gst_pipeline_get_bus(GST_PIPELINE(enc_pipeline));
    gst_bus_add_watch(bus, bus_call, loop);
    gst_object_unref(bus);

    g_print("Recording masked frames to %s (Ctrl+C to stop)\n",
            test_mode ? "test.mp4" : "output.mp4");

    gst_element_set_state(enc_pipeline, GST_STATE_PLAYING);
    gst_element_set_state(cap_pipeline, GST_STATE_PLAYING);

    pthread_create(&g_ctrl_thread, NULL, control_loop, NULL);
    g_main_loop_run(loop);

    /* Exit */
    g_print("Stopping...\n");

    gst_element_set_state(cap_pipeline, GST_STATE_NULL);
    gst_element_set_state(enc_pipeline, GST_STATE_NULL);
    gst_object_unref(cap_pipeline);
    gst_object_unref(enc_pipeline);
    g_main_loop_unref(loop);
    pthread_join(g_ctrl_thread, NULL);

    return 0;
}