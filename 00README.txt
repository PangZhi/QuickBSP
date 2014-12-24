please install cmake
Please set PROTOBUF_LIBRARY and PROTOBUF_INCLUDE_DIR in CMakeList.txt
TMB is needed.
run program as:
program_name graph_data_path thread_num

/* File Directory */ 
include/ 
    quickbsp/                   headers of thread level bsp
        bsp.h                   api of thread level bsp, include initialization, synchronization, communication and end of all threads.
        common.h                definition of basic structure of thread level bsp. 
                                InitData is global data shared by all threads, used to record the status of all threads. 
                                ThreadData is private data of every thread.
        engine-inl.h            layer between thread level bsp and vertex centric program. 
    
        vertex_layer-inl.h      definition of basic vertex type, edge type, graph type. Users should define its own vertex class which extend basic vertex class and implement
                                functions including Compute/UpdateValue/GetSendEdgeType/GetMessageValue/Reset.

src/                            
    examples/                  specific algorithm examples
    bsp.cc                     implementation of bsp


/* Structure */
*****************************************************************
*vertex centric programming  vertex_layer-inl.h, examples\*.cc  *
*****************************************************************
********** ********** ********** ********** ********** **********  
* vertex * * vertex * * vertex * * vertex * * vertex * * vertex *      
********** ********** ********** ********** ********** **********
*****************************************************************
*every thread handle task vertices and gather msg engine-inl.h  *
*****************************************************************
******************************** ********************************
*  working thread              * *   working thread             * 
******************************** ********************************
*****************************************************************
* thread control, communication, synchronization                *
* common.h, bsp.h                                               *
*****************************************************************

Every thread will be assigned several vertex as task vertices. In every
superstep, thread will go over all its task vertices and call user defined
functions including Compute/UpdateValue/GetSendEdgeType/GetMessageValue/Reset
(this handling is done in engin-inl.h and user define functions by defining 
vertex type, such as the type defined in cc.cc, shortest_path.cc). All threads
do communication and synchronization by using thread-level bsp (common.h and 
bsp.h).
