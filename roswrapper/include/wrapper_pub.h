 
#ifndef GO_WRAPPER_PUB_H
#define GO_WRAPPER_PUB_H

#ifdef __cplusplus
extern "C" {
#endif
    void call_publish(void*, char* data);
    int publish(void* callback,void* gopublisher, char* topic);
#ifdef __cplusplus
}
#endif 

#endif // GO_WRAPPER_PUB_H