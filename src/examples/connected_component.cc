// Copyright (c) 2014, Jing Fan. All Rights Reserved.

#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>

#include "quickbsp/engine-inl.h"
#include "quickbsp/hrtimer.h"
#include "quickbsp/vertex_layer-inl.h"

#define BUF_LEN 1024

static hrtimer_t timer;

struct VertexValue {
  explicit VertexValue(int l) : label(l) {}
  VertexValue() : label(10000000) {}
  int label;
};


/**
 * @brief define vertex type. The class should extend BaseVertex<VertexValue, 
 *    EdgeValue, MessageValue>. It should have function: Compute(), Reset(), 
 *    GetMessageValue(), UpdateValue(), GetSendEdgeType() 
 */
class MVertex : public graph::BaseVertex<VertexValue, void, int> {
 public:
  MVertex(const graph::node_id vertex_id) : BaseVertex(vertex_id) {}

  /**
   * @brief deal with message and update acc_message_value
   * @param message_value the value of message
   */
  void Compute(const int message_value) {
    if (this->acc_message_value() > message_value) {
      set_acc_message_value(message_value);
    }
  }

  /**
   * @brief clear acc_message_value
   */
  void Reset() {
    set_acc_message_value(10000000);
  }

  /**
   * @brief compute the message value to be sent to neighbours
   * @return message value
   */
  double GetMessageValue() {
    return this->vertex_value().label;
  }

  /**
   * @brief update vertex value
   */
  void UpdateValue() {
    if (this->vertex_value().label > this->acc_message_value()) {
      this->set_vertex_value(VertexValue(this->acc_message_value()));
      is_update_ = true;
    } else {
      is_update_ = false;
    }
    this->set_acc_message_value(10000000);
  }
  
  /**
   * @brief get gather type, that is which neighbours to send message to
   * @return gather edge type
   */
  graph::GatherEdgeType GetSendEdgeType() const {
    if (is_update_) {
      return graph::GatherEdgeType::kAllOutEdges;
    } else {
      return graph::GatherEdgeType::kNoEdges;
    }
  }
 private:
  bool is_update_;
};

/**
 * @brief define edge type
 */
typedef graph::Edge<void> MEdgeType;

int main(int argc, char **argv) {
  if (argc != 3) {
    std::cout << "ERROR: argument length should be 3" << std::endl;
    exit(-1);
  }

  const char* filename = argv[1];
  int thread_num = atoi(argv[2]);
  
  typedef graph::Graph<MVertex, void> GraphType;
  GraphType* graph_ptr = (new graph::GraphLoader<GraphType>)->Load(filename);
  
  initTimer(&timer);
  startTimer(&timer);
  graph_ptr->Finalize();
  stopTimer(&timer);
  
  // initialize
  for (int i = 0 ; i < graph_ptr->num_vertices(); ++i){
    MVertex& vertex = graph_ptr->GetVertexByIndex(i);
    vertex.set_acc_message_value(vertex.vertex_id());
  }
  double after_load = (double)(getTimerNs(&timer));
  std::cout << "time is: " << (double)(getTimerNs(&timer)) / 1e9 << " s" << std::endl;
 
  startTimer(&timer);
  typedef bsp::Engine<GraphType, MVertex, MEdgeType, double> Engine;
  std::unique_ptr<Engine> engine_ptr 
      = std::unique_ptr<Engine>(new Engine(thread_num,
                                           graph_ptr,
                                           bsp::PartitionType::kPartitionRoundRobin,
                                           20));
  engine_ptr->Run();
  stopTimer(&timer);
  std::cout << "time is: " << ((double)(getTimerNs(&timer)) ) / 1e9 << " s" << std::endl;
  
  // print value
  for (int i = 0 ; i < 100; i ++) {
    std::cout << graph_ptr->GetVertexByIndex(graph_ptr->GetIndexByNodeId(i)).vertex_value().label << std::endl;
  }
}

