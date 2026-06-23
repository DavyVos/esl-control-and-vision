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

/* ── Shared state between camera thread and control loop ── */
typedef struct {
    float       pan_target;   /* normalised: 0.0 = centre */
    float       tilt_target;
    int         valid;        /* 1 once the camera has written at least one frame */
    pthread_mutex_t lock;
} CameraTarget;

static CameraTarget g_target = {
    .pan_target  = 0.0f,
    .tilt_target = 0.0f,
    .valid       = 0,
    .lock        = PTHREAD_MUTEX_INITIALIZER,
};

/* ── Camera thread argument ── */
typedef struct {
    char *device_path;   /* e.g. "/dev/video7" */
} CameraThreadArgs;

/* ── GStreamer context (accessed only inside the camera thread) ── */
typedef struct {
    GMainLoop  *loop;
    GstElement *pipeline;
    guint       bus_watch_id;
} CameraThreadCtx;

static CameraThreadCtx g_cam_ctx = {0};

/* Call from any thread to stop the GStreamer main loop cleanly. */
void stop_camera_thread(void)
{
    if (g_cam_ctx.loop && g_main_loop_is_running(g_cam_ctx.loop))
        g_main_loop_quit(g_cam_ctx.loop);
}

/* ── GStreamer bus handler ── */
static gboolean bus_call(GstBus *bus, GstMessage *msg, gpointer data)
{
    GMainLoop *loop = (GMainLoop *)data;
    switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_EOS:
            g_print("Camera: end of stream\n");
            g_main_loop_quit(loop);
            break;
        case GST_MESSAGE_ERROR: {
            gchar  *debug;
            GError *error;
            gst_message_parse_error(msg, &error, &debug);
            g_free(debug);
            g_printerr("Camera error: %s\n", error->message);
            g_error_free(error);
            g_main_loop_quit(loop);
            break;
        }
        default:
            break;
    }
    return TRUE;
}

/* ── Pixel layout for BGRx ── */
typedef union {
    struct {
        unsigned int B : 8;
        unsigned int G : 8;
        unsigned int R : 8;
        unsigned int x : 8;
    };
    unsigned int data : 32;
} pf;

static int powint_cur(int val, int pow, int ret)
{
    return (pow <= 0) ? ret : powint_cur(val, pow - 1, ret * val);
}
static int powint(int val, int pow) { return powint_cur(val, pow, 1); }

static int MM(pf *data, int width, int height, int p, int q,
              int (*intensity)(pf))
{
    int sum = 0;
    for (int x = 0; x < width; x++) {
        int xp = powint(x, p);
        for (int y = 0; y < height; y++)
            sum += xp * powint(y, q) * intensity(data[y * width + x]);
    }
    return sum;
}

static int red(pf px) { return px.R >= 100 ? px.R : 0; }

/* Compute centroid and push it into g_target under the mutex. */
static void process_frame(guint8 *data, int width, int height)
{
    pf *pixels = (pf *)data;

    int m00 = MM(pixels, width, height, 0, 0, red);
    if (m00 == 0) {
        /* No red object visible — hold last known position. */
        return;
    }

    float x_center = ((float)(MM(pixels, width, height, 1, 0, red) / m00)) / width;
    float y_center = ((float)(MM(pixels, width, height, 0, 1, red) / m00)) / height;

    pthread_mutex_lock(&g_target.lock);
    g_target.pan_target  = x_center;
    g_target.tilt_target = y_center;
    g_target.valid       = 1;
    pthread_mutex_unlock(&g_target.lock);
}

/* ── appsink callback (runs in GStreamer's streaming thread) ── */
static GstFlowReturn on_new_sample(GstElement *appsink, gpointer user_data)
{
    (void)user_data;
    GstSample    *sample = gst_app_sink_pull_sample(GST_APP_SINK(appsink));
    GstBuffer    *buffer = gst_sample_get_buffer(sample);
    GstCaps      *caps   = gst_sample_get_caps(sample);
    GstStructure *s      = gst_caps_get_structure(caps, 0);
    GstMapInfo    map;
    int width, height;

    gst_structure_get_int(s, "width",  &width);
    gst_structure_get_int(s, "height", &height);
    gst_buffer_map(buffer, &map, GST_MAP_READ);
    process_frame(map.data, width, height);
    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

/* ── Camera thread entry point ── */
static void *camera_thread(void *arg)
{
    CameraThreadArgs *args = (CameraThreadArgs *)arg;

    g_cam_ctx.loop = g_main_loop_new(NULL, FALSE);

    GstElement *pipeline     = gst_pipeline_new("video-pipeline");
#ifdef TEST_SRC
    GstElement *src          = gst_element_factory_make("videotestsrc", "src");
#else
    GstElement *src          = gst_element_factory_make("v4l2src",      "src");
#endif
    GstElement *jpegenc      = gst_element_factory_make("jpegenc",      "enc");
    GstElement *jpegdec      = gst_element_factory_make("jpegdec",      "dec");
    GstElement *videoconvert = gst_element_factory_make("videoconvert", "conv");
    GstElement *appsink      = gst_element_factory_make("appsink",      "sink");

    if (!pipeline || !src || !jpegenc || !jpegdec || !videoconvert || !appsink) {
        g_printerr("Camera: failed to create pipeline elements.\n");
        return (void *)-1;
    }

#ifdef TEST_SRC
    g_object_set(G_OBJECT(src), "pattern", 18, NULL);
#else
    g_object_set(G_OBJECT(src), "device", args->device_path, NULL);
#endif

    g_object_set(G_OBJECT(appsink), "emit-signals", TRUE, NULL);
    g_signal_connect(appsink, "new-sample", G_CALLBACK(on_new_sample), NULL);

    /* Caps on the source: constrain resolution and framerate before encoding */
    GstCaps *src_caps = gst_caps_new_simple("video/x-raw",
                             "width",     G_TYPE_INT,        320,
                             "height",    G_TYPE_INT,        240,
                             "framerate", GST_TYPE_FRACTION, 30, 1,
                             NULL);

    /* Caps on the appsink: request BGRx (valid GStreamer format name) */
    GstCaps *raw_caps = gst_caps_new_simple("video/x-raw",
                             "format", G_TYPE_STRING, "BGRx",
                             NULL);

    GstBus *bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
    g_cam_ctx.bus_watch_id = gst_bus_add_watch(bus, bus_call, g_cam_ctx.loop);
    gst_object_unref(bus);

    gst_bin_add_many(GST_BIN(pipeline),
                     src, jpegenc, jpegdec, videoconvert, appsink, NULL);

    gst_element_link_filtered(src, jpegenc, src_caps);   /* constrain at source */
    gst_caps_unref(src_caps);
    gst_element_link(jpegenc, jpegdec);                  /* JPEG enc→dec, no caps needed */
    gst_element_link(jpegdec, videoconvert);
    gst_element_link_filtered(videoconvert, appsink, raw_caps);  /* BGRx output */
    gst_caps_unref(raw_caps);

    g_cam_ctx.pipeline = pipeline;

    g_print("Camera thread: starting pipeline.\n");
    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    g_main_loop_run(g_cam_ctx.loop);   /* blocks until stop_camera_thread() */

    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(GST_OBJECT(pipeline));
    g_source_remove(g_cam_ctx.bus_watch_id);
    g_main_loop_unref(g_cam_ctx.loop);
    g_cam_ctx.loop = NULL;

    g_print("Camera thread: exiting.\n");
    return NULL;
}

/* ─────────────────────────────────────────────────────────── */

int main(int argc, char **argv)
{
    /* GStreamer must be initialised once, before any thread uses it. */
    gst_init(&argc, &argv);

    /* ── Open /dev/mem and map FPGA peripherals ── */
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        perror("Couldn't open /dev/mem");
        return -1;
    }

    uint32_t *encoderTilt = (uint32_t *)mmap(NULL,
        HPS_0_ARM_A9_0_ENCODER_BUS_0_SPAN,
        PROT_READ | PROT_WRITE, MAP_SHARED, fd,
        HPS_0_ARM_A9_0_ENCODER_BUS_0_BASE);
    uint32_t *encoderPan  = (uint32_t *)mmap(NULL,
        HPS_0_ARM_A9_0_ENCODER_BUS_1_SPAN,
        PROT_READ | PROT_WRITE, MAP_SHARED, fd,
        HPS_0_ARM_A9_0_ENCODER_BUS_1_BASE);
    uint32_t *pwmTilt     = (uint32_t *)mmap(NULL,
        HPS_0_ARM_A9_0_PWMBUS_0_SPAN,
        PROT_READ | PROT_WRITE, MAP_SHARED, fd,
        HPS_0_ARM_A9_0_PWMBUS_0_BASE);
    uint32_t *pwmPan      = (uint32_t *)mmap(NULL,
        HPS_0_ARM_A9_0_PWMBUS_1_SPAN,
        PROT_READ | PROT_WRITE, MAP_SHARED, fd,
        HPS_0_ARM_A9_0_PWMBUS_1_BASE);

    if (encoderTilt == MAP_FAILED || encoderPan == MAP_FAILED ||
        pwmTilt     == MAP_FAILED || pwmPan     == MAP_FAILED) {
        perror("Couldn't map peripherals");
        close(fd);
        return -1;
    }

    /* ── Initialise and calibrate the pan/tilt platform ── */
    Jiwy jiwy;
    Jiwy_Init(&jiwy, encoderPan, encoderTilt, pwmPan, pwmTilt);
    Jiwy_CalibrateTilt(&jiwy);
    Jiwy_CalibratePan(&jiwy);

    /* ── Start the camera thread ── */
    CameraThreadArgs cam_args = {
        .device_path = (argc > 1) ? argv[1] : "/dev/video7",
    };
    pthread_t cam_tid;
    if (pthread_create(&cam_tid, NULL, camera_thread, &cam_args) != 0) {
        perror("pthread_create");
        goto cleanup;
    }

    g_print("Waiting for first camera frame...\n");

    /* ── Control loop: 100 µs step → ~10 kHz ── */
    int i = 0;
    while (1) {
        /* Read the latest camera targets under the mutex. */
        pthread_mutex_lock(&g_target.lock);
        if (g_target.valid) {
            jiwy.pan_target  = g_target.pan_target;
            jiwy.tilt_target = g_target.tilt_target;
        }
        pthread_mutex_unlock(&g_target.lock);

        /* Measure, run PID, drive PWM. */
        Jiwy_Update(&jiwy);

        /* Print ~every 100 ms (1000 × 100 µs). */
        if (++i >= 1000) {
            printf("Tilt: %+.4f (target: %+.4f)  Pan: %+.4f (target: %+.4f)\n",
                   jiwy.tilt_current, jiwy.tilt_target,
                   jiwy.pan_current,  jiwy.pan_target);
            i = 0;
        }

        usleep(100);
    }

    /* ── Shutdown (not normally reached; add signal handler to reach it) ── */
    stop_camera_thread();
    pthread_join(cam_tid, NULL);
    Jiwy_Disable(&jiwy);

cleanup:
    munmap(encoderTilt, HPS_0_ARM_A9_0_ENCODER_BUS_0_SPAN);
    munmap(encoderPan,  HPS_0_ARM_A9_0_ENCODER_BUS_1_SPAN);
    munmap(pwmTilt,     HPS_0_ARM_A9_0_PWMBUS_0_SPAN);
    munmap(pwmPan,      HPS_0_ARM_A9_0_PWMBUS_1_SPAN);
    close(fd);
    return 0;
}