package gst

/*
#cgo pkg-config: gstreamer-1.0 gstreamer-base-1.0 gstreamer-app-1.0 gstreamer-plugins-base-1.0 gstreamer-video-1.0 gstreamer-audio-1.0 gstreamer-plugins-bad-1.0
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>

static GstPipeline *GstCreatePipeline(char *pipelinestr) {
    GError *error = NULL;
    GstPipeline *pipeline = (GstPipeline*)GST_BIN(gst_parse_launch(pipelinestr, &error));
    return pipeline;
}

static void GstPipelineStart(GstPipeline *pipeline, int pipelineId) {
    gst_element_set_state(GST_ELEMENT(pipeline), GST_STATE_PLAYING);
}

static void GstPipelineStop(GstPipeline *pipeline) {
    gst_element_set_state(GST_ELEMENT(pipeline), GST_STATE_NULL);
}

static GstElement *GstPipelineFindElement(GstPipeline *pipeline, char *element) {
    GstElement *e = gst_bin_get_by_name(GST_BIN(pipeline), element);
    if (e != NULL) {
        gst_object_unref(e);
    }
    return e;
}

static void GstSetCaps(GstElement *element, char *capstr) {

    GObject *obj = G_OBJECT(element);
    GstCaps* caps = gst_caps_from_string(capstr);

    if (GST_IS_APP_SRC(obj)) {
        gst_app_src_set_caps(GST_APP_SRC(obj), caps);
    } else if (GST_IS_APP_SINK(obj)) {
        gst_app_sink_set_caps(GST_APP_SINK(obj), caps);
    } else {
        GParamSpec *spec = g_object_class_find_property(G_OBJECT_GET_CLASS(obj), "caps");
        if(spec) {
             g_object_set(obj, "caps", caps, NULL);
        } 
    }
    gst_caps_unref(caps);
}
*/
import "C"
import (
	"errors"
	"sync"
	"unsafe"
)

type Msg struct {
	GstMessage *C.GstMessage
}

type Element struct {
	element *C.GstElement
	out     chan []byte
	stop    bool
	id      int
}

type Pipeline struct {
	pipeline *C.GstPipeline
	msgs chan *Msg
	id       int
}

var (
	pipelines = make(map[int]*Pipeline)
	elements = make(map[int]*Element)
	gstLock sync.Mutex
	gstID = 12345
)

func init() {
	C.gst_init(nil,nil)
}

func New(pipelineStr string) (*Pipeline, error) {
	pipelineStrC := C.CString(pipelineStr)
	defer C.free(unsafe.Pointer(pipelineStrC))
	cpipeline := C.GstCreatePipeline(pipelineStrC)
	if cpipeline == nil {
		return nil, errors.New("Create pipeline error")
	}
	pipeline := &Pipeline{
		pipeline: cpipeline,
	}
	gstLock.Lock()
	defer gstLock.Unlock()
	gstID ++
	pipeline.id = gstID
	pipelines[pipeline.id] = pipeline
	return pipeline, nil
}

func (p *Pipeline) Start() {
	C.GstPipelineStart(p.pipeline, C.int(p.id))
}

func (p *Pipeline) Stop() {
	gstLock.Lock()
	delete(pipelines, p.id)
	gstLock.Unlock()
	if p.msgs != nil {
		close(p.msgs)
	}
	C.GstPipelineStop(p.pipeline)
}

func (p *Pipeline) FindElement(name string) *Element {
	elemName := C.CString(name)
	defer C.free(unsafe.Pointer(elemName))
	gstElement := C.GstPipelineFindElement(p.pipeline, elemName)
	if gstElement == nil {
		return nil
	}
	element := &Element{
		element: gstElement,
	}
	gstLock.Lock()
	defer gstLock.Unlock()
	gstID ++
	element.id = gstID
	elements[element.id] = element
	return element
}

func (e *Element) SetCap(cap string) {
	capStrC := C.CString(cap)
	defer C.free(unsafe.Pointer(capStrC))
	C.GstSetCaps(e.element, capStrC)
}
