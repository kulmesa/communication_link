 
#ifndef GO_WRAPPER_PUB_H
#define GO_WRAPPER_PUB_H

#ifdef __cplusplus
extern "C" {
#endif
/*    typedef struct callback_ptrs{
        void* object;
        void* memberfn;
        void* callit;
    }callback_struct; */
    void call_publish(void*);
    int publish(int argc, void* callback);
#ifdef __cplusplus
}
#endif 

#endif // GO_WRAPPER_PUB_H