// Copyright (c) 2014, Jing Fan. All Rights Reserved.

#ifndef QUICKBSP_ENGINE_INL_H_
#define QUICKBSP_ENGINE_INL_H_

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <vector>

#include "quickbsp/bsp.h"
#include "quickbsp/vertex_layer-inl.h"

namespace bsp {

/**
* @brief define partition type
*/
enum class PartitionType {
  // assign vertex to thread use round robin
  kPartitionRoundRobin,
  // assign vertex to thread continuously
  kPartitionContinuous
};

/**
 * @brief layer between thread level bsp and vertex centric program. It contains Spmd() function, which every
 *   working thread will execute. In Spmd(), thread will go over all its task vertices and update vertex value
 *   and send message by calling the function defined in the vertex class.
 * @type GraphType Graph<Class Vertex, EdgeValue>
 * @type VertexType user defined vertex class which extends the BaseVertex<VertexValue, EdgeValue, MessageValue>
 * @type EdgeType Edge<EdgeValue>
 * @type MessageValue message value type
 */
template <typename GraphType, class VertexType, typename EdgeType, typename MessageValue>
class Engine {
 public:
  /**
  * @brief constructor
  * @param thread_num  the num of thread 
  * @param graph_ptr   graph
  * @param partition_type the type of partition, can be continuous or roundrobin
  * @param max_iter    control iteration
  */
  Engine(const int thread_num, GraphType* graph_ptr, const PartitionType partition_type, const int max_iter = 10)
      : thread_num_(thread_num),
        graph_ptr_(std::unique_ptr<GraphType>(graph_ptr)),
        max_iter_(max_iter),
        partition_type_(partition_type) {
    vertex_num_per_thread_ = ceil(static_cast<double>(graph_ptr_->num_vertices())/thread_num_);
    AllocateGraph();
  }

  /**
  * @brief begin bsp threads
  */
  void Run() {
    std::cout << "thread_num: " << thread_num_ << std::endl;
    BspBegin(&Hook, this, thread_num_);
  }

 private:
  /*
  * @brief assign an vertex to working thread
  * @param vertex id
  * @param partition_type
  * @return working thread id
  */
  inline int partition(const int vertex_id, const PartitionType partition_type) const {
    if (partition_type == PartitionType::kPartitionRoundRobin) {
      return vertex_id % thread_num_;
    } else if (partition_type == PartitionType::kPartitionContinuous) {
      return vertex_id / vertex_num_per_thread_;
    }
    return -1;
  }

  /**
  * @brief allocate vertex to threads and store the result in thread_vertices_
  */
  void AllocateGraph() {
    int num_vertices = graph_ptr_->num_vertices();
    thread_vertices_.resize(thread_num_);
    for (int i = 0 ; i < thread_num_; ++i) {
      thread_vertices_[i].reserve(num_vertices / thread_num_ + 1);
    }
    for (int i = 0 ; i < num_vertices; ++i) {
      thread_vertices_[partition(i, partition_type_)].push_back(i);
    }
  }

  /**
  * @brief use hook because when compiling, the first argument of all non-static member functions will be 
  *  "this" pointer by default. So use a static member function to solve this problem. 
  * @param thread_data_ptr thread data of this thread
  * @param this_ptr this pointer of this class instance
  */
  static void Hook(ThreadData* thread_data_ptr, void* this_ptr) {
    reinterpret_cast<Engine<GraphType, VertexType, EdgeType, MessageValue>*>(this_ptr)->Spmd(thread_data_ptr);
  }

  /**
  * @brief every thread will run Spmd()
  * @param thread_data_ptr thread data of this thread
  */
  void Spmd(ThreadData* thread_data_ptr) { 
    std::cout << "Spmd from thread" << thread_data_ptr->pid << std::endl;
    
    // get write buffer and reade buffer
    void **pub_w = thread_data_ptr->pub_w;
    void **pub_r = thread_data_ptr->pub_r;

    // initiate write and read buffer
    for (int i = 0 ; i < thread_num_; ++i) {
      std::vector<graph::VertexMessage<MessageValue> >* tmp = new std::vector<graph::VertexMessage<MessageValue> >;
      tmp->reserve(graph_ptr_->num_vertices());
      pub_w[i]= reinterpret_cast<void*>(tmp);
      tmp = new std::vector<graph::VertexMessage<MessageValue> >;
      tmp->reserve(graph_ptr_->num_vertices());
      pub_r[i]= reinterpret_cast<void*>(tmp);
    }

    while (BspGetSuperstep() < max_iter_) {
      // superstep
      Superstep(thread_data_ptr);
      // barrier synchronization
      BspSync(thread_data_ptr);
      // if all the threads have vote to halt, stop the work
      if (BspGetHalt()) break;
    }

    for (int i = 0 ; i < thread_num_; ++i) {
      delete reinterpret_cast<std::vector<graph::VertexMessage<MessageValue> >*>(pub_w[i]);
      delete reinterpret_cast<std::vector<graph::VertexMessage<MessageValue> >*>(pub_r[i]);
    }
    BspEnd(thread_data_ptr);
  }


  /**
  * @brief super step
  * @param thread_data_ptr thread data of this thread
  */
  void Superstep(ThreadData* thread_data_ptr) {
    std::cout << "superstep from thread " <<  thread_data_ptr->pid << std::endl;

    // clear write buffer
    std::vector<graph::VertexMessage<MessageValue> > **pub_w = reinterpret_cast<std::vector<graph::VertexMessage<MessageValue> >** >(
      thread_data_ptr->pub_w);
    for (int i = 0 ; i < thread_num_; ++i) {
      pub_w[i]->clear();
    }

    std::vector<graph::VertexMessage<MessageValue> >* msgs_ptr = nullptr;

    // while can get message from message bus, deal with messages
    tmb::TaggedMessage tmp_msg;

    while (BspMove(thread_data_ptr, tmp_msg)) {
      const void *receive_msg = tmp_msg.message();
      // the message tranferred is pointer to data
      msgs_ptr = reinterpret_cast<std::vector<graph::VertexMessage<MessageValue> >* >(*(reinterpret_cast<const std::int64_t*>(receive_msg)));

      // deal with all messages by calling vertex.Compute( messagevalue )
      typename std::vector<graph::VertexMessage<MessageValue> >::iterator vertex_msg_iter;
      for (vertex_msg_iter = msgs_ptr->begin(); vertex_msg_iter != msgs_ptr -> end(); ++vertex_msg_iter) {
        int target = vertex_msg_iter->target;
        VertexType& vertex = graph_ptr_->GetVertexByIndex(target);
        // call vertex.Compute(message value) for every message
        vertex.Compute(vertex_msg_iter->message_value);
      }
    }

    std::vector<int>& vertices = thread_vertices_[thread_data_ptr->pid];
    std::vector<int>::iterator iter;
    // go through all the vertex and get messages they want to send
    for (iter = vertices.begin(); iter != vertices.end(); ++iter) {
      VertexType& vertex = graph_ptr_->GetVertexByIndex(*iter);
      vertex.UpdateValue();
      switch ( vertex.GetSendEdgeType() ) {
        case graph::GatherEdgeType::kNoEdges:
        break;
        case graph::GatherEdgeType::kAllInEdges:
        {
          int spos = vertex.in_spos();
          int epos = spos + vertex.in_len();
          for (int i = spos; i < epos ; ++i) {
            pub_w[partition(graph_ptr_->GetInEdgeByIndex(i), partition_type_)]->push_back
            (std::move(graph::VertexMessage<MessageValue>(graph_ptr_->GetInEdgeByIndex(i),
                                                          vertex.GetMessageValue())));
          }
          break;
        }
        case graph::GatherEdgeType::kAllOutEdges:
        {
          int spos = vertex.out_spos();
          int epos = spos + vertex.out_len();
          for (int i = spos; i < epos; ++i) {
            pub_w[partition(graph_ptr_->GetOutEdgeByIndex(i), partition_type_)]->push_back
            (std::move(graph::VertexMessage<MessageValue>(graph_ptr_->GetOutEdgeByIndex(i),
                                                          vertex.GetMessageValue())));
          }
          break;
        }
        case graph::GatherEdgeType::kAllEdges:
        {
          int spos = vertex.in_spos();
          int epos = spos + vertex.in_len();
          for (int i = spos; i < epos ; ++i) {
            pub_w[partition(graph_ptr_->GetInEdgeByIndex(i), partition_type_)]->push_back
            (std::move(graph::VertexMessage<MessageValue>(graph_ptr_->GetInEdgeByIndex(i),
                                                          vertex.GetMessageValue())));
          }
          spos = vertex.out_spos();
          epos = spos + vertex.out_len();
          for (int i = spos; i < epos; ++i) {
            pub_w[partition(graph_ptr_->GetOutEdgeByIndex(i), partition_type_)]->push_back
            (std::move(graph::VertexMessage<MessageValue>(graph_ptr_->GetOutEdgeByIndex(i),
                                                          vertex.GetMessageValue())));
          }
          break;
        }
      }
    }
    bool to_halt = true;
    for (int i = 0 ; i < thread_num_; ++i) {
      if ( pub_w[i]->size() != 0 ) {
        // the content of the msg is the address of corresponding write buffer
        std::int64_t addr = reinterpret_cast<std::int64_t>(pub_w[i]);
        BspSend(thread_data_ptr, i, reinterpret_cast<void*>(&addr), sizeof(addr));
        to_halt = false;
      }
    }
    BspSetHalt(to_halt);
  }

  // num of thread
  int thread_num_;
  // iteration num
  int max_iter_;
  // graph
  std::unique_ptr<GraphType> graph_ptr_;
  // vertices assigned to every thread
  std::vector<std::vector<int> > thread_vertices_;
  // vertex num assigned to every thread, only useful when use
  // continuous partition
  int vertex_num_per_thread_;
  // partition type
  PartitionType partition_type_;
};

}  // namespace bsp

#endif  // QUICKBSP_ENGINE_INL_H_
