#include <pthread.h>
#include <gst/gst.h>
#include <glib.h>
#include <gst/app/app.h>
#include <fcntl.h>  // For O_* constants
#include <sys/mman.h>
#include <sys/stat.h> // For mode constants
#include <unistd.h>   // For ftruncate


typedef struct {
    char *device_path;  // argv[1], or NULL for TEST_SRC
} CameraThreadArgs;

typedef struct {
    GMainLoop    *loop;
    GstElement   *pipeline;
    guint         bus_watch_id;
} CameraThreadContext;

static CameraThreadContext cam_ctx = {0};

// Call this from another thread to stop the camera
void stop_camera_thread(void) {
    if (cam_ctx.loop && g_main_loop_is_running(cam_ctx.loop))
        g_main_loop_quit(cam_ctx.loop);
}

static void *camera_thread(void *arg) {
    CameraThreadArgs *args = (CameraThreadArgs *)arg;

    GstElement *pipeline, *v4l2src, *jpegdec, *videoconvert, *appsink, *filter;
    GstBus     *bus;
    GstCaps    *caps, *convert_caps;

    // GStreamer init is safe to call from a thread if done once globally,
    // but calling again is a no-op — so it's fine here too.
    gst_init(NULL, NULL);

    cam_ctx.loop = g_main_loop_new(NULL, FALSE);

    pipeline     = gst_pipeline_new("video-pipeline");
#ifdef TEST_SRC
    v4l2src      = gst_element_factory_make("videotestsrc",  "v4l2-source");
#else
    v4l2src      = gst_element_factory_make("v4l2src",       "v4l2-source");
#endif
    filter       = gst_element_factory_make("jpegenc",       "filter");
    jpegdec      = gst_element_factory_make("jpegdec",       "jpeg-decoder");
    videoconvert = gst_element_factory_make("videoconvert",  "video-convert");
    appsink      = gst_element_factory_make("appsink",       "video-sink");

    if (!pipeline || !v4l2src || !jpegdec || !videoconvert || !appsink) {
        g_printerr("One element could not be created. Exiting thread.\n");
        return (void *)-1;
    }

#ifdef TEST_SRC
    g_object_set(G_OBJECT(v4l2src), "pattern", 18, NULL);
#else
    g_object_set(G_OBJECT(v4l2src), "device", args->device_path, NULL);
#endif

    g_object_set(G_OBJECT(appsink), "emit-signals", TRUE, NULL);
    g_signal_connect(appsink, "new-sample", G_CALLBACK(on_new_sample_from_sink), NULL);

    caps = gst_caps_new_simple("image/jpeg",
                               "width",     G_TYPE_INT,          160,
                               "height",    G_TYPE_INT,          120,
                               "framerate", GST_TYPE_FRACTION, 30, 1,
                               NULL);

    convert_caps = gst_caps_new_simple("video/x-raw",
                                       "format", G_TYPE_STRING, "xBGR",
                                       NULL);

    bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));
    cam_ctx.bus_watch_id = gst_bus_add_watch(bus, bus_call, cam_ctx.loop);
    gst_object_unref(bus);

    gst_bin_add_many(GST_BIN(pipeline), v4l2src, filter, jpegdec, videoconvert, appsink, NULL);
    gst_element_link(v4l2src, filter);
    gst_element_link_filtered(filter, jpegdec, caps);
    gst_caps_unref(caps);
    gst_element_link(jpegdec, videoconvert);
    gst_element_link_filtered(videoconvert, appsink, convert_caps);
    gst_caps_unref(convert_caps);

    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    g_main_loop_run(cam_ctx.loop);          // blocks here until quit

    // Cleanup
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(GST_OBJECT(pipeline));
    g_source_remove(cam_ctx.bus_watch_id);
    g_main_loop_unref(cam_ctx.loop);
    cam_ctx.loop = NULL;

    return NULL;
}

// int main(int argc, char *argv[]) {
// #ifndef TEST_COM
//     // ... shared memory setup unchanged ...
// #endif

//     gst_init(&argc, &argv);   // init once in main

//     CameraThreadArgs args = { .device_path = argc > 1 ? argv[1] : NULL };

//     pthread_t cam_tid;
//     if (pthread_create(&cam_tid, NULL, camera_thread, &args) != 0) {
//         perror("pthread_create");
//         return 1;
//     }

//     // Your main thread can do other work here...

//     pthread_join(cam_tid, NULL);   // or detach if you don't need to wait

// #ifndef TEST_COM
//     munmap(shared_memory, SHARED_MEM_SIZE);
//     close(shm_fd);
// #endif
//     return 0;
// }