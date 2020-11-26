
#ifndef GO_WRAPPER_SUB_H
#define GO_WRAPPER_SUB_H

#ifdef __cplusplus
extern "C" {
#endif
    int subscribe(void* callback, char* topic, char* msgtype, char* name);
#ifdef __cplusplus
}
#endif    

#endif // GO_WRAPPER_SUB_H